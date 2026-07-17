from setuptools import find_packages, setup

package_name = 'wheeltec_arm_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zyc',
    maintainer_email='zyc@todo.todo',
    description='Serial driver and vision-based grabber nodes for the WHEELTEC arm.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'arm_control = wheeltec_arm_control.arm_grabber:main',
            'arm_grabber = wheeltec_arm_control.arm_grabber:main',
            'arm_serial_driver = wheeltec_arm_control.arm_serial_driver:main',
        ],
    },
)
