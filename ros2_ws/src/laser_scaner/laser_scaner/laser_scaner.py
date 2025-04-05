import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import serial
import re

class SectorPublisher(Node):
    def __init__(self):
        super().__init__('sector_publisher')
        self.publisher = self.create_publisher(Int16MultiArray, '/scan', 10)
        self.ser = None
        self.connect_serial('/dev/ttyUSB1', 115200)
        self.timer = self.create_timer(0.1, self.read_and_publish)

    def connect_serial(self, port, baud):
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f'Serial port {port} opened at {baud} baud.')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            rclpy.shutdown()

    def read_and_publish(self):
        if not self.ser or not self.ser.in_waiting:
            return

        try:
            line = self.ser.readline().decode('utf-8').strip()
            if not line.startswith("SECTORS:"):
                return

            # Extract numbers after SECTORS:
            numbers = re.findall(r'\d+', line)
            if len(numbers) != 12:
                self.get_logger().warn(f'Invalid sector count: {len(numbers)} in line "{line}"')
                return

            values = [int(n) for n in numbers]

            msg = Int16MultiArray()
            msg.data = values
            self.publisher.publish(msg)
            self.get_logger().info(f'Published: {values}')

        except Exception as e:
            self.get_logger().error(f'Error reading from serial: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SectorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
