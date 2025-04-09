#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from config_service_interface.srv import ConfigBase, ConfigArg
from .board_manager import API as BoardManageAPI

class ConfigServiceServer(Node):

    def __init__(self):
        super().__init__('config_service_server')
        self.camera_prepare_srv = self.create_service(
            ConfigBase, 
            'ai_module/camera_prepare',
            self.handle_camera_prepare
        )

        self.start_yolo_srv = self.create_service(
            ConfigBase, 
            'ai_module/yolo_start',
            self.handle_start_yolo
        )

        self.get_yolo_pid_srv = self.create_service(
            ConfigBase, 
            'ai_module/yolo_get_pid',
            self.handle_yolo_get_pid
        )

        self.stop_yolo_srv = self.create_service(
            ConfigArg, 
            'ai_module/yolo_stop',
            self.handle_stop_yolo
        )

        self.board_api = BoardManageAPI()

    def handle_camera_prepare(self, request, response):
        ip = request.ai_module_ip
        self.board_api.board_camera_prepare()

        response.success = True
        response.info = f'OK'

        return response

    def handle_start_yolo(self, request, response): 
        ip = request.ai_module_ip
        self.board_api.start_yolo()

        response.success = True
        response.info = f'OK'

        return response

    def handle_yolo_get_pid(self, request, response): 
        ip = request.ai_module_ip

        response.success = True
        response.info = str(self.board_api.get_yolo_pid())

    def handle_stop_yolo(self, request, response): 
        ip = request.ai_module_ip
        pid = request.process_id
        self.board_api.start_yolo()

        response.success = True
        response.info = f'OK'

def main(args=None):
    rclpy.init(args=args)
    node = ConfigServiceServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()