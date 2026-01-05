"""
Bridge Node: Konvertiert semantische Labels zu nvblox-kompatiblen Formaten
Erstellt separate Masken pro Klasse für nvblox multi-object reconstruction
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import yaml
import os

class SemanticToNvbloxBridge(Node):
    def __init__(self):
        super().__init__('semantic_to_nvblox_bridge')
        
        # Load semantic config
        config_path = "/workspaces/isaac_ros-dev/config/semantic_classes.yaml"
        self.semantic_config = self.load_config(config_path)
        
        # Build ID to color mapping
        self.id_to_color = {}
        self.id_to_name = {}
        for name, info in self.semantic_config['semantic_classes'].items():
            class_id = info['id']
            self.id_to_color[class_id] = info['color']
            self.id_to_name[class_id] = name
        
        self.bridge = CvBridge()
        self.latest_camera_info = None
        
        # Parameters
        self.declare_parameter('mode', 'single_mask')  # 'single_mask' or 'multi_class'
        self.declare_parameter('target_classes', 'chair,table,person')
        
        # Subscriptions
        self.sub_semantic = self.create_subscription(
            Image, '/semantic/image_mono8', self.semantic_callback, 10
        )
        self.sub_camera_info = self.create_subscription(
            CameraInfo, '/semantic/camera_info', self.camera_info_callback, 10
        )
        
        # Publishers
        self.pub_mask = self.create_publisher(Image, '/mask/image', 10)
        self.pub_mask_info = self.create_publisher(CameraInfo, '/mask/camera_info', 10)
        self.pub_colored_mask = self.create_publisher(Image, '/mask/colored', 10)
        
        self.get_logger().info("Semantic to nvblox bridge ready")

    def load_config(self, path):
        if not os.path.exists(path):
            self.get_logger().error(f"Config not found: {path}")
            return {'semantic_classes': {}, 'settings': {}}
        
        with open(path, 'r') as f:
            return yaml.safe_load(f)

    def camera_info_callback(self, msg):
        self.latest_camera_info = msg

    def semantic_callback(self, msg):
        try:
            # Convert semantic mono8 image
            semantic_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            
            mode = self.get_parameter('mode').value
            
            if mode == 'single_mask':
                # Create combined binary mask for all target classes
                mask = self.create_combined_mask(semantic_img)
            else:
                # Create colored mask with class-specific colors
                mask = self.create_colored_mask(semantic_img)
            
            # Publish mask
            self.publish_mask(mask, msg)
            
            # Publish colored visualization
            colored_vis = self.create_colored_visualization(semantic_img)
            colored_msg = self.bridge.cv2_to_imgmsg(colored_vis, encoding="bgr8")
            colored_msg.header = msg.header
            self.pub_colored_mask.publish(colored_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error in semantic callback: {e}")

    def create_combined_mask(self, semantic_img):
        """
        Create binary mask for target classes.
        All target classes become white (255), rest is black (0).
        """
        target_classes_str = self.get_parameter('target_classes').value
        target_classes = [c.strip().lower() for c in target_classes_str.split(',')]
        
        # Get IDs of target classes
        target_ids = []
        for name, info in self.semantic_config['semantic_classes'].items():
            if name in target_classes:
                target_ids.append(info['id'])
        
        # Create mask
        mask = np.zeros_like(semantic_img, dtype=np.uint8)
        for target_id in target_ids:
            mask[semantic_img == target_id] = 255
        
        # Count pixels per class
        unique_ids = np.unique(semantic_img[semantic_img > 0])
        for uid in unique_ids:
            if uid in target_ids:
                count = np.sum(semantic_img == uid)
                name = self.id_to_name.get(uid, f"ID_{uid}")
                self.get_logger().info(
                    f"Class '{name}' (ID {uid}): {count} pixels",
                    throttle_duration_sec=2.0
                )
        
        return mask

    def create_colored_mask(self, semantic_img):
        """
        Create RGB mask where each class has its defined color.
        This preserves class information in the mask.
        """
        H, W = semantic_img.shape
        colored_mask = np.zeros((H, W, 3), dtype=np.uint8)
        
        unique_ids = np.unique(semantic_img)
        for class_id in unique_ids:
            if class_id == 0:  # Skip background
                continue
            
            color = self.id_to_color.get(class_id, [200, 200, 200])
            mask_pixels = semantic_img == class_id
            colored_mask[mask_pixels] = color
        
        return colored_mask

    def create_colored_visualization(self, semantic_img):
        """
        Create visualization with class colors for debugging.
        """
        return self.create_colored_mask(semantic_img)

    def publish_mask(self, mask, original_msg):
        """Publish mask to nvblox."""
        if len(mask.shape) == 2:
            # Binary mask (mono8)
            encoding = "mono8"
        else:
            # Colored mask (bgr8)
            encoding = "bgr8"
        
        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding=encoding)
        mask_msg.header = original_msg.header
        mask_msg.header.frame_id = "camera_0_color_optical_frame"
        
        self.pub_mask.publish(mask_msg)
        
        # Republish camera info
        if self.latest_camera_info is not None:
            cam_info = self.latest_camera_info
            cam_info.header.stamp = original_msg.header.stamp
            self.pub_mask_info.publish(cam_info)

def main(args=None):
    rclpy.init(args=args)
    node = SemanticToNvbloxBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
