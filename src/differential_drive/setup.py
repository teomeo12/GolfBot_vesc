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
        (os.path.join('share', package_name, 'arduino_sketches'), glob('arduino_sketches/*.ino')),
        (os.path.join('share', package_name), glob('1600s_aug_100ep.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='james',
    maintainer_email='jamespetri28@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'], # This is deprecated and handled by package.xml
    entry_points={
        'console_scripts': [
            'diff_vel_ctrl_node = differential_drive.velocity_control:main',
            'sony_gamepad_node = differential_drive.sony_gamepad:main',
            'logitech_gamepad_node = differential_drive.logitech_gamepad:main',
            'gps_monitor_node = differential_drive.gps_monitor_node:main',
            'camera_node = differential_drive.camera_node:main',
            'stepper_controller_node = old_stepper_nodes.stepper_controller_node:main',
            'stepper_imu_node = differential_drive.stepper_imu_node:main',
            'divot_detector = differential_drive.divot_detection_intel:main',
            'align_and_repair_node = differential_drive.align_and_repair_node:main',
            'odometry_node = differential_drive.odometry_node:main',
            'path_follower_node = differential_drive.path_follower_node:main',
            'golfbot_gui_node = differential_drive.golfbot_gui_node:main',
        ],
    },
)
