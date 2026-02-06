entry_points={
    'console_scripts': [
        'motor_node = smart_cart_project.motor_node:main',
        'sensor_node = smart_cart_project.sensor_node:main',
        'vision_node = smart_cart_project.vision_node:main',
        'imu_node = smart_cart_project.imu_node:main',
        'planner = smart_cart_project.path_planner:main', # 패스 플래닝 추가
        'app_node = smart_cart_project.app_node:main',
    ],
},