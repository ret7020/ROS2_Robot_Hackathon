import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import time
import os
import glob

class MJPEGStreamProcessor(Node):
    def __init__(self):
        super().__init__('ai_module_camera_stream')


        self.declare_parameters(
            namespace='',
            parameters=[
                ('ai_module_ip', '10.160.209.1'),
                ('ai_module_mjpeg_port', 7777),
                ('receive_fps', 10)
            ]
        )

        (self.receive_fps_param, self.ai_module_ip, self.ai_module_mjpeg_port) = self.get_parameters(
            ['receive_fps', 'ai_module_ip', 'ai_module_mjpeg_port']
        )

        stream_uri = f"http://{self.ai_module_ip.value}:{self.ai_module_mjpeg_port.value}"

        self.get_logger().info(f"Usging AI module stream at: {stream_uri}")
        
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, 'ai_module/image', 10)

        self.cap = cv2.VideoCapture(stream_uri)
        self.timer = self.create_timer(1 / self.receive_fps_param.value, self.capture_and_publish)

    def capture_and_publish(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    ai_module_camera = MJPEGStreamProcessor()
    rclpy.spin(ai_module_camera)

if __name__ == '__main__':
    main()