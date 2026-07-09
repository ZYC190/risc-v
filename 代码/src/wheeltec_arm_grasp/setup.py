from setuptools import setup
import os
from glob import glob

package_name = 'wheeltec_arm_grasp'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROS 2 grasping node for WHEELTEC 6-axis arm with eye-to-hand stereo camera',
    license='BSD',
    entry_points={
        'console_scripts': [
            'arm_grasp_node = wheeltec_arm_grasp.arm_grasp_node:main',
        ],
    },
)
