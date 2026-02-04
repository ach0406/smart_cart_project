from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='smart_cart_project', executable='motor_node', name='motor'),
        Node(package='smart_cart_project', executable='qr_node', name='qr'),
        Node(package='smart_cart_project', executable='sensor_node', name='sensor'),
        Node(package='smart_cart_project', executable='planner', name='planner'),
    ])