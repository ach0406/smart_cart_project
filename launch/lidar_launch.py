from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='smart_cart_project',
            executable='lidar_node',
            name='lidar_node',
            output='screen',
            parameters=[{'port': '/dev/ttyUSB0', 'frame_id': 'laser_frame'}]
        ),
        Node(
            package='smart_cart_project',
            executable='imu_node',
            name='imu_node',
            output='screen',
        ),
    ])
