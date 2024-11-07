import rclpy
import math
import time
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from geometry_msgs.msg import PoseStamped
from pyquaternion import Quaternion
import cv2
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class LinePublisher(Node):
    def __init__(self):
        super().__init__('line_publisher')
       
       self.suscriber_image = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            3
        )

    
def main(args=None):
    rclpy.init(args=args)
    line_publisher = LinePublisher()
    rclpy.spin(line_publisher)
    line_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()