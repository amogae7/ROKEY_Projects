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
            'smart_manager_2 = dsr_rokey2.smart_manager_2:main',
            'smart_manager_J = dsr_rokey2.smart_manager_J:main',
            'smart_manager_fin = dsr_rokey2.smart_manager_fin:main',
            'smart_manager_integrated = dsr_rokey2.smart_manager_integrated:main',
            'smart_manager_integrated_2 = dsr_rokey2.smart_manager_integrated_2:main',
            'robot_control = dsr_rokey2.robot_control:main',
            'robot_control_2 = dsr_rokey2.robot_control_2:main',
            'robot_control_3 = dsr_rokey2.robot_control_3:main',
            'robot_control_4 = dsr_rokey2.robot_control_4:main',
            'robot_control_J = dsr_rokey2.robot_control_J:main',
            'robot_control_final = dsr_rokey2.robot_control_final:main',
            'yolo_node_2 = dsr_rokey2.yolo_node_2:main',
            'yolo_node_3 = dsr_rokey2.yolo_node_3:main',
            'yolo_node_J = dsr_rokey2.yolo_node_J:main',
            'yolo_node_fin = dsr_rokey2.yolo_node_fin:main',
            'check_matrix = dsr_rokey2.check_matrix:main',
            'yolo_cam_publisher = dsr_rokey2.yolo_cam_publisher:main',

            
        ],
    },
)
