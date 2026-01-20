from setuptools import find_packages, setup

package_name = 'rokey_pjt'

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
    maintainer_email='namgoo1018@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'nav_to_pose_2 = rokey_pjt.nav_to_pose_2:main',
        'vehicle_type_2 = rokey_pjt.vehicle_type:main',
        'main_control_2 = rokey_pjt.main_control_2:main',
        ],
    },
)
