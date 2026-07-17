from glob import glob

from setuptools import find_packages, setup

package_name = 'wheeltec_ui_dashboard'

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
        ('share/' + package_name + '/assets', glob('assets/*.png')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zyc',
    maintainer_email='zyc@todo.todo',
    description='PyQt5 ROS 2 dashboard for the WHEELTEC robot.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ui_dashboard = wheeltec_ui_dashboard.ui_dashboard:main',
        ],
    },
)
