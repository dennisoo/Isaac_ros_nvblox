import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import torch

# Imports for DINO & SAM
from groundingdino.util.inference import load_model, predict
import groundingdino.datasets.transforms as T
from segment_anything import sam_model_registry, SamPredictor
import supervision as sv

# PATHS (inside container)
DATA_DIR = "/workspaces/isaac_ros-dev/data/weights"
DINO_CONFIG = os.path.join(DATA_DIR, "GroundingDINO_SwinT_OGC.py")
DINO_CHECKPOINT = os.path.join(DATA_DIR, "groundingdino_swint_ogc.pth")
SAM_CHECKPOINT = os.path.join(DATA_DIR, "sam_vit_h_4b8939.pth")

class DinoNode(Node):
    def __init__(self):
        super().__init__('dino_node')
        
        # --- STANDARD VOCABULARY ---
        # GroundingDINO will search for ALL of these items simultaneously.
        self.common_objects = self.load_classes_from_file(CONFIG_FILE)
        
        # Combine list into a single prompt string separated by dots
        default_prompt = " . ".join(self.common_objects)

        # ROS Parameters
        self.declare_parameter('text_prompt', default_prompt) 
        
        # Increased thresholds to reduce false positives with large vocabulary
        self.declare_parameter('box_threshold', 0.50) 
        self.declare_parameter('text_threshold', 0.40)
        
        self.bridge = CvBridge()

        # --- 1. INTELLIGENT DEVICE DETECTION ---
        self.target_device = "cpu" # Default fallback
        if torch.cuda.is_available():
            try:
                self.get_logger().info("CUDA found. Testing compatibility...")
                # Simple dummy operation to check if GPU is actually accessible
                dummy = torch.zeros(1).to("cuda")
                _ = dummy + 1
                self.target_device = "cuda"
                self.get_logger().info("✅ GPU test successful! Using CUDA.")
            except Exception as e:
                self.get_logger().warn(f"⚠️ CUDA error: {e}. Falling back to CPU.")
                self.target_device = "cpu"
        else:
            self.get_logger().info("ℹ️ No CUDA found. Using CPU.")

        # --- 2. LOAD MODELS ---
        self.get_logger().info("Loading Grounding DINO...")
        if not os.path.exists(DINO_CHECKPOINT):
            self.get_logger().error(f"Weights missing at {DINO_CHECKPOINT}")
            return

        self.dino_model = load_model(DINO_CONFIG, DINO_CHECKPOINT)
        self.dino_model = self.dino_model.to(self.target_device)
        
        self.get_logger().info("Loading SAM...")
        self.sam = sam_model_registry["vit_h"](checkpoint=SAM_CHECKPOINT)
        self.sam.to(device=self.target_device)
        self.sam_predictor = SamPredictor(self.sam)
        
        # --- 3. INITIALIZE ANNOTATORS ---
        self.box_annotator = sv.BoxAnnotator()
        self.mask_annotator = sv.MaskAnnotator()
        self.label_annotator = sv.LabelAnnotator(text_position=sv.Position.TOP_LEFT)

        # --- ROS COMMUNICATION ---
        self.sub = self.create_subscription(Image, '/rgb', self.callback, 10)
        self.pub_overlay = self.create_publisher(Image, '/dino_sam/result', 10)
        
        self.get_logger().info(f"Ready on {self.target_device}!")
        self.get_logger().info(f"Vocabulary size: {len(self.common_objects)} objects")

    def callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            prompt = self.get_parameter('text_prompt').value
            
            # --- A. GROUNDING DINO PREDICTION ---
            transform = T.Compose([
                T.RandomResize([800], max_size=1333),
                T.ToTensor(),
                T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
            ])
            _, image_tensor = self.load_image_from_cv(cv_image, transform)
            
            # Detect boxes
            boxes, logits, phrases = predict(
                model=self.dino_model,
                image=image_tensor,
                caption=prompt,
                box_threshold=self.get_parameter('box_threshold').value,
                text_threshold=self.get_parameter('text_threshold').value,
                device=self.target_device
            )

            # If nothing found, publish original image
            if len(boxes) == 0:
                self.pub_overlay.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))
                return

            # --- B. SAM PREDICTION ---
            self.sam_predictor.set_image(cv_image)
            H, W, _ = cv_image.shape
            
            # Convert boxes to xyxy format for SAM
            boxes_xyxy = boxes * torch.Tensor([W, H, W, H])
            boxes_xyxy = self.box_convert(boxes=boxes_xyxy).numpy()

            transformed_boxes = self.sam_predictor.transform.apply_boxes_torch(
                torch.as_tensor(boxes_xyxy, device=self.sam.device), cv_image.shape[:2]
            )
            
            # Generate masks
            masks, _, _ = self.sam_predictor.predict_torch(
                point_coords=None,
                point_labels=None,
                boxes=transformed_boxes,
                multimask_output=False,
            )
            
            # --- C. INTELLIGENT CLASS MAPPING ---
            # DINO returns phrases like ['chair', 'chair', 'table'].
            # We map unique phrases to unique IDs so they get different colors.
            unique_phrases = list(set(phrases)) 
            class_ids = [unique_phrases.index(p) for p in phrases]
            
            detections = sv.Detections(
                xyxy=boxes_xyxy,
                mask=masks.cpu().numpy().squeeze(1),
                class_id=np.array(class_ids) # Assign IDs for coloring
            )

            # --- D. VISUALIZATION ---
            # 1. Draw Masks
            annotated_frame = self.mask_annotator.annotate(scene=cv_image.copy(), detections=detections)
            # 2. Draw Boxes
            annotated_frame = self.box_annotator.annotate(scene=annotated_frame, detections=detections)
            
            # 3. Draw Labels (Shows "chair 0.85")
            labels = [
                f"{phrase} {logit:.2f}"
                for phrase, logit in zip(phrases, logits)
            ]
            annotated_frame = self.label_annotator.annotate(
                scene=annotated_frame, detections=detections, labels=labels
            )
            
            self.pub_overlay.publish(self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8"))

        except Exception as e:
            self.get_logger().error(f"Error in callback: {e}")

    def load_image_from_cv(self, cv_image, transform):
        from PIL import Image as PILImage
        image_pil = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        image_tensor = transform(image_pil, None)[0]
        return image_pil, image_tensor

    def box_convert(self, boxes):
        # Convert cxcywh -> xyxy
        cx, cy, w, h = boxes.unbind(-1)
        b = [(cx - 0.5 * w), (cy - 0.5 * h), (cx + 0.5 * w), (cy + 0.5 * h)]
        return torch.stack(b, dim=-1)

def main(args=None):
    rclpy.init(args=args)
    node = DinoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()