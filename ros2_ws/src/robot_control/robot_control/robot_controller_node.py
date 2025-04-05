#!/usr/bin/env python3
 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import serial

class RobotSerialController(Node):
    def __init__(self):
        super().__init__('robot_serial_controller')

        # Подписываемся на команду скорости
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)

        # Настройка Serial порта (замените на свой путь)
        self.serial_port = serial.Serial(
            "/dev/ttyUSB0",
            baudrate=115200,
            timeout=1
        )

        self.timer = self.create_timer(0.2, self.serial_listen_callback)
        self.odom_publisher = self.create_publisher(Float64MultiArray, 'odom', 10)
        
        ping_command = "PING\n"
        self.serial_port.write(ping_command.encode())
    
    def listener_callback(self, msg):
        linear_velocity = msg.linear.x * 1000  # м/с в мм/с
        angular_velocity = msg.angular.z  # рад/с

        command = f"SET_ROBOT_VELOCITY {linear_velocity:.2f} {angular_velocity:.2f}\n"
        self.serial_port.write(command.encode())
        self.get_logger().info(f"Sent command: {command.strip()}")
    
    def serial_listen_callback(self):
        line = self.serial_port.readline().decode("utf-8").split()
        try:
            if line[0] == "POS":
                x = float(line[1].replace("X=",""))
                y = float(line[2].replace("Y=",""))
                th = float(line[3].replace("Th=",""))
                self.get_logger().info(f"{x} {y} {th}")
                odom_msg = Float64MultiArray()
                odom_msg.data = [x, y, th]
                self.odom_publisher.publish(odom_msg)
        except Exception as e:
            self.get_logger().info(f"{e}  {line}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotSerialController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
