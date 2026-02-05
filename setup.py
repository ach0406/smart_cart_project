entry_points={
    'console_scripts': [
        'motor_node = smart_cart_project.motor_node:main',
        'qr_node = smart_cart_project.qr_node:main',
        'planner = smart_cart_project.path_planner:main',
        'sensor_node = smart_cart_project.sensor_node:main',
        'vision_node = smart_cart_project.vision_node:main', 
    ],
},