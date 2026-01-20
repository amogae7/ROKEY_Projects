from setuptools import find_packages, setup

package_name = 'dsr_rokey2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mini_jog = dsr_rokey2.mini_jog:main',
            'smart_manager = dsr_rokey2.smart_manager:main',
            'robot_control = dsr_rokey2.robot_control:main',
            'yolo_node = dsr_rokey2.yolo_node:main',
            'check_matrix = dsr_rokey2.check_matrix:main',
            'realsense = dsr_rokey2.realsense:main',
            'yolo = dsr_rokey2.yolo:main',
            'detection_node = dsr_rokey2.detection_node:main',
            'aruco_detection_node = dsr_rokey2.aruco_detection_node:main',
            'yolo_node_3 = dsr_rokey2.yolo_node_3:main',
            'robot_control_3 = dsr_rokey2.robot_control_3:main',
            'yolo_node_4 = dsr_rokey2.yolo_node_4:main',
            'robot_control_4 = dsr_rokey2.robot_control_4:main',
            'smart_manager_4 = dsr_rokey2.smart_manager_4:main',
            'robot_control_1 = dsr_rokey2.robot_control_1:main',
            'smart_manager_5 = dsr_rokey2.smart_manager_5:main',
            'yolo_node_5 = dsr_rokey2.yolo_node_5:main',
            'yolo_node_7 = dsr_rokey2.yolo_node_7:main',
            'robot_control_6 = dsr_rokey2.robot_control_6:main',
            'smart_manager_6 = dsr_rokey2.smart_manager_6:main',
            'smart_manager_7 = dsr_rokey2.smart_manager_7:main',
            'robot_control_7 = dsr_rokey2.robot_control_7:main',
            'yolo_node_8 = dsr_rokey2.yolo_node_8:main',
            'robot_control_8 = dsr_rokey2.robot_control_8:main',
            'smart_manager_8 = dsr_rokey2.smart_manager_8:main',
            'yolo_node_fin = dsr_rokey2.yolo_node_fin:main',
            'yolo_node_9 = dsr_rokey2.yolo_node_9:main',
            'smart_manager_final = dsr_rokey2.smart_manager_final:main',
            'robot_control_final = dsr_rokey2.robot_control_final:main',
            'smart_manager_fin = dsr_rokey2.smart_manager_fin:main',
            'smart_manager_integrated = dsr_rokey2.smart_manager_integrated:main',









            
        ],
    },
)
