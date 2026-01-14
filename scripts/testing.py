import json
import numpy as np
from typing import Dict, Tuple

import message_filters
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from nvblox_msgs.msg import SemanticLabelsStamped
from sensor_msgs.msg import Image

class SemanticConversion(Node):
    
    def __init__(self) -> None:
        super().__init__('semantic_label_converter')
