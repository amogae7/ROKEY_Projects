from setuptools import find_packages, setup

package_name = 'cam_transform'

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
    description='Camera Transform and YOLO Detection Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 1. 좌표 변환 노드
            'transform_node = cam_transform.transform_node:main',
            
            # 2. 웹캠 퍼블리셔 노드
            'img_publisher = cam_transform.img_publisher:main',
            
            # 3. YOLO 감지 노드
            'yolo_node = cam_transform.yolo_webcam_node:main',
        ],
    },
)
