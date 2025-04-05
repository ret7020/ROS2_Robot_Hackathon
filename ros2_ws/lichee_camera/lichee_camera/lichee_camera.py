import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import time
import os
import glob

class CaptureCameraNode(Node):
    def __init__(self):
        super().__init__('capture_camera')
        
        self.bridge = CvBridge()

        self.publisher = self.create_publisher(Image, 'camera/image', 10)

        self.cap = cv2.VideoCapture("http://10.160.209.1:7777")
        print("Camera created", self.cap)

        self.timer = self.create_timer(0.1, self.capture_and_publish)

    def capture_and_publish(self):
        ret, frame = self.cap.read()
        self.get_logger().info(f"Cap trig: {frame}")
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    capture_camera = CaptureCameraNode()
    rclpy.spin(capture_camera)

if __name__ == '__main__':
    main()