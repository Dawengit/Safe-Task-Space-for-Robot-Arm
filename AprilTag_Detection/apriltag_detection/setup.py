from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'apriltag_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Install rviz files
        (os.path.join('share', package_name, 'rviz'), 
         glob('rviz/*.rviz')),
        # Install config files
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stanley_chueh',
    maintainer_email='stanleychueh28@gmail.com',
    description='AprilTag detection with safety zone monitoring',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # AprilTag detection node
            'apriltag_detection= apriltag_detection.apriltag_detection:main',
            # safety controller node
            # 'arm_safety_controller = apriltag_detection.arm_safety_controller:main',
            # if you have more nodes, you can add them here
        ],
    },
)
