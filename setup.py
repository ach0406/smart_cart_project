from setuptools import setup
import os
from glob import glob

package_name = 'smart_cart_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='안창현',
    maintainer_email='ach0406@hanyang.ac.kr',
    description='Capstone: Autonomous Smart Cart Project',
    license='Apache License 2.0',
    tests_require=['pytest'],
entry_points={
        'console_scripts': [
            'motor_node = smart_cart_project.motor_node:main',
            'sensor_node = smart_cart_project.sensor_node:main',
            'vision_node = smart_cart_project.vision_node:main',
            'imu_node = smart_cart_project.imu_node:main',
            'planner = smart_cart_project.path_planner:main',
            'app_node = smart_cart_project.app_node:main',
            'final_cart = smart_cart_project.final_cart:main', 
        ],
    },
)
