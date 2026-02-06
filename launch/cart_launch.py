from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='smart_cart_project', executable='motor_node'),
        Node(package='smart_cart_project', executable='sensor_node'),
        Node(package='smart_cart_project', executable='vision_node'),
        Node(package='smart_cart_project', executable='imu_node'),
        Node(package='smart_cart_project', executable='planner'),
    ])