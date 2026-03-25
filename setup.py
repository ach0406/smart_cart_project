from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'smart_cart_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch 파일들을 인식시키기 위한 설정
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='changhyeon',
    maintainer_email='changhyeon@todo.com',
    description='Smart Cart Project with RPLiDAR A1',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_node = smart_cart_project.motor_node:main',
            'sensor_node = smart_cart_project.sensor_node:main',
            'vision_node = smart_cart_project.vision_node:main',
            'imu_node = smart_cart_project.imu_node:main',
            'planner = smart_cart_project.path_planner:main',
            'app_node = smart_cart_project.app_node:main',
        ],
    },
)
