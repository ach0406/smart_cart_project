from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. RPLidar A1M8 드라이버 (빌드한 sllidar_ros2 패키지 사용)
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/ttyUSB0', # 만약 안 되면 /dev/ttyUSB1 확인
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'inverted': False,
                'angle_compensate': True,
            }],
            output='screen'
        ),

        # 2. IMU 노드
        Node(
            package='smart_cart_project',
            executable='imu_node',
            name='imu_node',
            output='screen'
        ),

        # 3. 좌표 변환 (TF) - 이게 있어야 Rviz에서 에러가 안 납니다
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['0.1', '0', '0.2', '0', '0', '0', 'base_link', 'laser_frame']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'imu_link']
        )
    ])
