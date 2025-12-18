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

# PATHS (inside container, mounted via data/weights)
DATA_DIR = "/workspaces/isaac_ros-dev/data/weights"
DINO_CONFIG = os.path.join(DATA_DIR, "GroundingDINO_SwinT_OGC.py")
DINO_CHECKPOINT = os.path.join(DATA_DIR, "groundingdino_swint_ogc.pth")
SAM_CHECKPOINT = os.path.join(DATA_DIR, "sam_vit_h_4b8939.pth")

class DinoNode(Node):
    def __init__(self):
        super().__init__('dino_node')
        
        # ROS Parameters
        self.declare_parameter('text_prompt', 'chair') # Default prompt
        self.declare_parameter('box_threshold', 0.35)
        self.declare_parameter('text_threshold', 0.25)
        
        self.bridge = CvBridge()

        # --- 1. INTELLIGENT DEVICE DETECTION ---
        self.target_device = "cpu" # Default fallback
        
        if torch.cuda.is_available():
            try:
                self.get_logger().info("CUDA found. Testing compatibility...")
                # Perform a small test to check if the GPU actually works
                # (Catches errors like 'no kernel image' on older GPUs like GTX 1080)
                dummy = torch.zeros(1).to("cuda")
                _ = dummy + 1
                self.target_device = "cuda"
                self.get_logger().info("✅ GPU test successful! Using CUDA.")
            except Exception as e:
                self.get_logger().warn(f"⚠️ CUDA available but access failed: {e}")
                self.get_logger().warn("➡️ Falling back to CPU (slow but stable).")
                self.target_device = "cpu"
        else:
            self.get_logger().info("ℹ️ No CUDA found. Using CPU.")

        # --- 2. LOAD MODELS ---
        self.get_logger().info("Loading Grounding DINO...")
        if not os.path.exists(DINO_CHECKPOINT):
            self.get_logger().error(f"Weights not found: {DINO_CHECKPOINT}")
            return

        self.dino_model = load_model(DINO_CONFIG, DINO_CHECKPOINT)
        # Move DINO model to the detected device
        self.dino_model = self.dino_model.to(self.target_device)
        
        self.get_logger().info("Loading SAM...")
        self.sam = sam_model_registry["vit_h"](checkpoint=SAM_CHECKPOINT)
        # Move SAM model to the detected device
        self.sam.to(device=self.target_device)
        self.sam_predictor = SamPredictor(self.sam)
        
        # --- 3. ROS SETUP ---
        self.image_topic = "/rgb" # Adjust to your bag topic if needed
        
        self.sub = self.create_subscription(Image, self.image_topic, self.callback, 10)
        self.pub_overlay = self.create_publisher(Image, '/dino_sam/result', 10)
        
        self.get_logger().info(f"Ready on {self.target_device}! Listening to {self.image_topic}")

    def callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # --- A. GROUNDING DINO ---
            prompt = self.get_parameter('text_prompt').value
            
            transform = T.Compose([
                T.RandomResize([800], max_size=1333),
                T.ToTensor(),
                T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
            ])
            image_source, image_tensor = self.load_image_from_cv(cv_image, transform)
            
            # IMPORTANT: Pass 'device=self.target_device' here!
            boxes, logits, phrases = predict(
                model=self.dino_model,
                image=image_tensor,
                caption=prompt,
                box_threshold=self.get_parameter('box_threshold').value,
                text_threshold=self.get_parameter('text_threshold').value,
                device=self.target_device  # <--- Use automatically selected device
            )

            if len(boxes) == 0:
                self.pub_overlay.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))
                return

            # --- B. SAM ---
            self.sam_predictor.set_image(cv_image)
            
            H, W, _ = cv_image.shape
            boxes_xyxy = boxes * torch.Tensor([W, H, W, H])
            boxes_xyxy = self.box_convert(boxes=boxes_xyxy).numpy()

            transformed_boxes = self.sam_predictor.transform.apply_boxes_torch(
                torch.as_tensor(boxes_xyxy, device=self.sam.device), cv_image.shape[:2]
            )
            
            masks, _, _ = self.sam_predictor.predict_torch(
                point_coords=None,
                point_labels=None,
                boxes=transformed_boxes,
                multimask_output=False,
            )
            
            # --- C. VISUALIZATION ---
            detections = sv.Detections(
                xyxy=boxes_xyxy,
                mask=masks.cpu().numpy().squeeze(1),
                class_id=np.array([0] * len(boxes))
            )
            
            box_annotator = sv.BoxAnnotator()
            mask_annotator = sv.MaskAnnotator()
            
            annotated_frame = mask_annotator.annotate(scene=cv_image.copy(), detections=detections)
            annotated_frame = box_annotator.annotate(scene=annotated_frame, detections=detections)
            
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