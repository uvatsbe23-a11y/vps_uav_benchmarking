#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class IRPreprocessor(Node):
    def __init__(self):
        super().__init__("ir_preprocessor")
        self.bridge = CvBridge()
        self.sub_left = self.create_subscription(Image, "/camera/left_ir/image_raw", self.left_callback, 10)
        self.sub_right = self.create_subscription(Image, "/camera/right_ir/image_raw", self.right_callback, 10)
        self.pub_left = self.create_publisher(Image, "/camera/left_ir/image_filtered", 10)
        self.pub_right = self.create_publisher(Image, "/camera/right_ir/image_filtered", 10)
        self.get_logger().info("IR Preprocessor running")

    def process(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        filtered = cv2.medianBlur(img, 5)
        filtered = cv2.equalizeHist(filtered)
        return self.bridge.cv2_to_imgmsg(filtered, encoding="mono8", header=msg.header)

    def left_callback(self, msg):
        self.pub_left.publish(self.process(msg))

    def right_callback(self, msg):
        self.pub_right.publish(self.process(msg))

def main(args=None):
    rclpy.init(args=args)
    node = IRPreprocessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
