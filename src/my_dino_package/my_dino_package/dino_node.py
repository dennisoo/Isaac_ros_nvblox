import rclpy
import cv2
import numpy as np
import sys
import os
import torch
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory # <--- WICHTIG für dynamische Pfade

user_site = os.path.expanduser("~/.local/lib/python3.12/site-packages")
if user_site not in sys.path:
    sys.path.append(user_site)

from groundingdino.util.inference import load_model, predict
import groundingdino.datasets.transforms as T

# Try MobileSAM first, fallback to regular SAM
from segment_anything import sam_model_registry, SamPredictor

import supervision as sv

# PATHS (Weights bleiben statisch, Config wird dynamisch geladen)
DATA_DIR = "/workspaces/isaac_ros-dev/data/weights"

DINO_CONFIG = os.path.join(DATA_DIR, "GroundingDINO_SwinT_OGC.py")
DINO_CHECKPOINT = os.path.join(DATA_DIR, "groundingdino_swint_ogc.pth")
MOBILE_SAM_CHECKPOINT = os.path.join(DATA_DIR, "mobile_sam.pt")
SAM_CHECKPOINT = os.path.join(DATA_DIR, "sam_vit_h_4b8939.pth")

class SemanticDinoNode(Node):
    def __init__(self):
        super().__init__('semantic_dino_node')
        
        # --- LOAD SEMANTIC CONFIGURATION DYNAMICALLY ---
        # Sucht den Pfad zum installierten Paket 'my_dino_package'
        try:
            package_share_directory = get_package_share_directory('my_dino_package')
            semantic_config_path = os.path.join(package_share_directory, 'config', 'semantic_classes.yaml')
            self.get_logger().info(f"Loading semantic config from: {semantic_config_path}")
        except Exception as e:
            self.get_logger().error(f"Could not find package 'my_dino_package'. Error: {e}")
            # Fallback path (optional, falls lokal entwickelt wird)
            semantic_config_path = "/workspaces/isaac_ros-dev/src/Isaac_ros_nvblox/src/my_dino_package/config/semantic_classes.yaml"

        self.semantic_config = self.load_semantic_config(semantic_config_path)
        
        # --- PARSE CONFIG ---
        self.class_to_id = {name: info['id'] for name, info in self.semantic_config['semantic_classes'].items()}
        # Colors in YAML are RGB, but OpenCV uses BGR - convert them!
        self.class_to_color = {
            name: [info['color'][2], info['color'][1], info['color'][0]]  # RGB -> BGR
            for name, info in self.semantic_config['semantic_classes'].items()
        }
        
        # Build detection prompt from config
        self.common_objects = list(self.class_to_id.keys())
        default_prompt = " .  ".join(self.common_objects)

        # ROS Parameters
        self.declare_parameter('text_prompt', default_prompt) 
        self.declare_parameter('box_threshold', 0.25)  
        self.declare_parameter('text_threshold', 0.25)
        self.declare_parameter('use_instance_ids', True)
        
        # NEW: Parameter for model selection
        # Options: "mobile" (default), "vit_b", "vit_h"
        self.declare_parameter('model_type', 'mobile') 
        
        self.bridge = CvBridge()
        self.latest_camera_info = None
        
        # Throttling to improve performance
        self.frame_count = 0
        self.process_every_n_frames = 1  # Process every frame (change to 5 for speed)

        # --- DEVICE DETECTION ---
        self.target_device = "cpu" 
        if torch.cuda.is_available():
            try:
                dummy = torch.zeros(1).to("cuda")
                _ = dummy + 1
                self.target_device = "cuda"
                self.get_logger().info("Using CUDA")
            except Exception as e:
                self.get_logger().warn(f"CUDA error: {e}. Using CPU.")
                self.target_device = "cpu"
        else:
            self.get_logger().info("ℹ Using CPU")

        # --- LOAD MODELS ---
        self.get_logger().info("Loading Grounding DINO...")
        self.dino_model = load_model(DINO_CONFIG, DINO_CHECKPOINT)
        self.dino_model = self.dino_model.to(self.target_device)
        
        # SAM Model Selection Logic
        model_selection = self.get_parameter('model_type').value.lower()
        self.get_logger().info(f"Selected SAM Model: {model_selection}")
        
        if model_selection == "mobile":
            # MobileSAM
            try:
                from mobile_sam import sam_model_registry as mobile_registry
                self.sam = mobile_registry["vit_t"](checkpoint=MOBILE_SAM_CHECKPOINT)
                self.get_logger().info("Loaded MobileSAM (ViT-T)")
            except ImportError:
                self.get_logger().error("MobileSAM not installed! Falling back to standard SAM ViT-B.")
                model_selection = "vit_b" # Fallback

        if model_selection == "vit_b":
             # Standard SAM Base
             sam_checkpoint = os.path.join(DATA_DIR, "sam_vit_b_01ec64.pth")
             self.sam = sam_model_registry["vit_b"](checkpoint=sam_checkpoint)
             self.get_logger().info("Loaded Standard SAM (ViT-B)")
             
        elif model_selection == "vit_h":
             # Standard SAM Huge
             sam_checkpoint = SAM_CHECKPOINT
             self.sam = sam_model_registry["vit_h"](checkpoint=sam_checkpoint)
             self.get_logger().info("Loaded Standard SAM (ViT-H) - WARNING: Slow!")

        self.sam.to(device=self.target_device)
        self.sam_predictor = SamPredictor(self.sam)
        
        sam_name = "SAM-ViT-H"
        self.get_logger().info(f"Using {sam_name} on {self.target_device}")
        
        # --- ANNOTATORS (supervision 0.18.0 API) ---
        self.box_annotator = sv.BoxAnnotator()
        self.mask_annotator = sv.MaskAnnotator()
        
        # --- ROS SUBSCRIPTIONS ---
        self.sub_rgb = self.create_subscription(
            Image, 'image', self.callback, qos_profile_sensor_data
        )
        self.sub_camera_info = self.create_subscription(
            CameraInfo, 'camera_info', self.camera_info_callback, qos_profile_sensor_data
        )
        
        # --- ROS PUBLISHERS ---
        self.pub_overlay = self.create_publisher(Image, '/dino_sam/result', 10)
        self.pub_semantic_mono8 = self.create_publisher(Image, '/semantic/image_mono8', 10)
        self.pub_semantic_rgb8 = self.create_publisher(Image, '/semantic/image_rgb8', 10)
        self.pub_semantic_info = self.create_publisher(CameraInfo, '/semantic/camera_info', 10)
        
        self.get_logger().info(f"Ready!  Loaded {len(self.common_objects)} semantic classes")

    def load_semantic_config(self, path):
        """Load semantic class configuration from YAML."""
        if not os.path.exists(path):
            self.get_logger().warn(f"Semantic config not found:  {path}")
            return {
                'semantic_classes': {
                    'person': {'id': 1, 'color': [255, 0, 0], 'description': 'Person'},
                    'chair': {'id': 2, 'color': [0, 255, 0], 'description': 'Chair'}
                },
                'settings': {'max_classes': 50, 'unknown_color': [200, 200, 200]}
            }
        
        with open(path, 'r') as f:
            config = yaml.safe_load(f)
        self.get_logger().info(f"📄 Loaded semantic config from {path}")
        return config

    def camera_info_callback(self, msg):
        """Store camera info for republishing."""
        self.latest_camera_info = msg

    def callback(self, msg):
        self.frame_count += 1
        if self.frame_count % self.process_every_n_frames != 0:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            prompt = self.get_parameter('text_prompt').value
            
            # --- GROUNDING DINO ---
            transform = T.Compose([
                T.RandomResize([800], max_size=1333),
                T.ToTensor(),
                T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
            ])
            _, image_tensor = self.load_image_from_cv(cv_image, transform)
            
            boxes, logits, phrases = predict(
                model=self.dino_model,
                image=image_tensor,
                caption=prompt,
                box_threshold=self.get_parameter('box_threshold').value,
                text_threshold=self.get_parameter('text_threshold').value,
                device=self.target_device
            )

            if len(boxes) == 0:
                self.publish_empty_semantic(msg, cv_image)
                self.pub_overlay.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))
                return

            # --- SAM ---
            self.sam_predictor.set_image(cv_image)
            H, W, _ = cv_image.shape
            
            boxes_xyxy = boxes * torch.Tensor([W, H, W, H])
            boxes_xyxy = self.box_convert(boxes=boxes_xyxy).numpy()

            transformed_boxes = self.sam_predictor.transform.apply_boxes_torch(
                torch.as_tensor(boxes_xyxy, device=self.sam.device), 
                cv_image.shape[:2]
            )
            
            masks, _, _ = self.sam_predictor.predict_torch(
                point_coords=None,
                point_labels=None,
                boxes=transformed_boxes,
                multimask_output=False,
            )
            
            # --- SEMANTIC LABELING ---
            semantic_mono8, semantic_rgb8, class_ids = self.create_semantic_images(
                masks.cpu().numpy().squeeze(1), 
                phrases, 
                logits,  
                (H, W), 
                original_image=cv_image
            )
            
            # --- PUBLISH SEMANTIC IMAGES ---
            self.publish_semantic_images(semantic_mono8, semantic_rgb8, msg)
            
            # --- VISUALIZATION ---
            detections = sv.Detections(
                xyxy=boxes_xyxy,
                mask=masks.cpu().numpy().squeeze(1),
                class_id=np.array(class_ids)
            )

            annotated_frame = self.mask_annotator.annotate(
                scene=cv_image.copy(), 
                detections=detections
            )
            
            labels = [
                f"{phrase} (ID:{self.class_to_id.get(phrase.lower(), 0)}) {logit:.2f}"
                for phrase, logit in zip(phrases, logits)
            ]

            annotated_frame = self.box_annotator.annotate(
                scene=annotated_frame,
                detections=detections,
                labels=labels
            )
            
            annotated_frame = self.add_legend(annotated_frame, phrases)
            
            self.pub_overlay.publish(
                self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            )

        except Exception as e:
            self.get_logger().error(f"Error in callback: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

    def create_semantic_images(self, masks, phrases, logits, shape, original_image=None):
        """Create semantic label images using Max-Confidence Arbitration.
        
        Args:
            masks: Array of segmentation masks
            phrases: Detected class names
            logits: Confidence scores for each detection
            shape: (H, W) image shape
            original_image: Original BGR camera image
        """
        H, W = shape
        semantic_mono8 = np.zeros((H, W), dtype=np.uint8)
        
        # Initialize with black background
        semantic_rgb8 = np.zeros((H, W, 3), dtype=np.uint8) 
        
        # --- NEW: CONFIDENCE MAP ---
        # Keeps track of the highest score per pixel to prevent overwriting
        # high-confidence detections with low-confidence noise.
        confidence_map = np.zeros((H, W), dtype=np.float32)

        class_ids = []
        
        # Iterate through all detected objects
        for i, phrase in enumerate(phrases):
            phrase_lower = phrase.lower()
            
            class_id = self.class_to_id.get(phrase_lower, 0)
            color = self.class_to_color.get(phrase_lower, [200, 200, 200])
            score = float(logits[i]) # The probability/confidence of this object
            
            class_ids.append(class_id)
            
            mask = masks[i].astype(bool)
            
            # --- LOGIC: HIGH CONFIDENCE WINS ---
            # We look for pixels that belong to the current mask AND
            # where the new score is higher than the existing score in the confidence_map.
            
            # 1. Where is the mask active?
            # 2. Where is the new score higher than what was there before?
            update_mask = mask & (score > confidence_map)
            
            # Update the image ONLY at these "better" pixels
            if np.any(update_mask):
                semantic_mono8[update_mask] = class_id
                semantic_rgb8[update_mask] = color
                
                # IMPORTANT: Update the confidence map with the new high score
                confidence_map[update_mask] = score
            
        return semantic_mono8, semantic_rgb8, class_ids

    def publish_semantic_images(self, semantic_mono8, semantic_rgb8, original_msg):
        """Publish semantic label images."""
        mono8_msg = self.bridge.cv2_to_imgmsg(semantic_mono8, encoding="mono8")
        mono8_msg.header = original_msg.header
        self.pub_semantic_mono8.publish(mono8_msg)
        
        # Convert BGR to RGB for nvblox (nvblox expects rgb8 encoding)
        semantic_rgb8_converted = cv2.cvtColor(semantic_rgb8, cv2.COLOR_BGR2RGB)
        rgb8_msg = self.bridge.cv2_to_imgmsg(semantic_rgb8_converted, encoding="rgb8")
        rgb8_msg.header = original_msg.header
        self.pub_semantic_rgb8.publish(rgb8_msg)
        
        if self.latest_camera_info is not None:
            cam_info = self.latest_camera_info
            cam_info.header.stamp = original_msg.header.stamp
            self.pub_semantic_info.publish(cam_info)

    def publish_empty_semantic(self, original_msg, original_image):
        """Publish semantic images when no objects detected."""
        H, W, _ = original_image.shape
        empty_mono = np.zeros((H, W), dtype=np.uint8)
        # WICHTIG: Original-Bild als RGB senden, damit nvblox Farben hat!
        self.publish_semantic_images(empty_mono, original_image, original_msg)

    def add_legend(self, image, phrases):
        """Add legend to image."""
        unique_phrases = list(set(phrases))
        
        legend_height = len(unique_phrases) * 25 + 20
        legend_width = 250
        
        overlay = image.copy()
        cv2.rectangle(overlay, (10, 10), (10 + legend_width, 10 + legend_height), (0, 0, 0), -1)
        image = cv2.addWeighted(overlay, 0.7, image, 0.3, 0)
        
        cv2.putText(image, "Detected Classes:", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        for i, phrase in enumerate(sorted(unique_phrases)):
            phrase_lower = phrase.lower()
            color = self.class_to_color.get(phrase_lower, [200, 200, 200])
            class_id = self.class_to_id.get(phrase_lower, 0)
            
            y_pos = 55 + i * 25
            cv2.rectangle(image, (20, y_pos - 10), (40, y_pos + 5), tuple(color), -1)
            cv2.putText(image, f"{phrase} (ID:  {class_id})", (50, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        return image

    def load_image_from_cv(self, cv_image, transform):
        from PIL import Image as PILImage
        image_pil = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        image_tensor = transform(image_pil, None)[0]
        return image_pil, image_tensor

    def box_convert(self, boxes):
        cx, cy, w, h = boxes.unbind(-1)
        b = [(cx - 0.5 * w), (cy - 0.5 * h), (cx + 0.5 * w), (cy + 0.5 * h)]
        return torch.stack(b, dim=-1)

def main(args=None):
    rclpy.init(args=args)
    node = SemanticDinoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()