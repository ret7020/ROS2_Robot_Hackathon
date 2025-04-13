from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    stream_node = Node(
        package='camera',
        executable='ai_module_camera',
    )

    detector_node = Node(
        package='http_api',
        executable='ai_module_detector'
    )

    config_service_server = Node(
        package='config_service_server',
        executable='ai_module_config'
    )

    return LaunchDescription([
        stream_node,
        detector_node,
        config_service_server
    ])