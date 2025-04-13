# http_ros2_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray, MultiArrayDimension
from flask import Flask, request, jsonify
import threading
import json
import logging


log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)


class HttpToRosNode(Node):
    def __init__(self):
        super().__init__('http_to_ros_node')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'detector', 10)

# Flask app
app = Flask(__name__)
ros_node = None

@app.route('/data', methods=['POST'])
def post_data():
    data_raw = request.data.decode("utf-8").split(";")
    dt = Int32MultiArray()
    dd = []
    for i in data_raw:
        if len(i) > 0:
            dd.append(list(map(int, i.split(","))))

    dt.data = [item for sublist in dd for item in sublist]
    num_rows = len(dd)
    num_cols = len(dd[0]) if num_rows > 0 else 0
    dim1 = MultiArrayDimension()

    dim1.label = 'rows'
    dim1.size = num_rows
    dim1.stride = num_rows * num_cols

    dim2 = MultiArrayDimension()
    dim2.label = 'cols'
    dim2.size = num_cols
    dim2.stride = num_cols  

    dt.layout.dim = [dim1, dim2]

    ros_node.publisher_.publish(dt)

    return "OK"

def flask_thread():
    app.run(host='0.0.0.0', port=5000)

def main(args=None):
    global ros_node
    rclpy.init(args=args)
    ros_node = HttpToRosNode()

    # Start Flask server in separate thread
    thread = threading.Thread(target=flask_thread, daemon=True)
    thread.start()

    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
