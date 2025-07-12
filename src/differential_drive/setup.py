import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'differential_drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='james',
    maintainer_email='jamespetri28@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diff_vel_ctrl_node = differential_drive.velocity_control:main',
            'sony_gamepad_node = differential_drive.sony_gamepad:main',
            'logitech_gamepad_node = differential_drive.logitech_gamepad:main',
            'gps_monitor_node = differential_drive.gps_monitor_node:main',
            'camera_node = differential_drive.camera_node:main'
        ],
    },
)
