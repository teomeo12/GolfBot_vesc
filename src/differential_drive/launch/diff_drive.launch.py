import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    left_vesc_config = os.path.join(
        get_package_share_directory('differential_drive'),
        'config',
        'left_vesc_config.yaml'
    )
    right_vesc_config = os.path.join(
        get_package_share_directory('differential_drive'),
        'config',
        'right_vesc_config.yaml'
    )
    
    # Declare launch argument for gamepad type
    gamepad_type_arg = DeclareLaunchArgument(
        'gamepad_type',
        default_value='logitech',
        description='Type of gamepad to use: sony or logitech'
    )

    return LaunchDescription([
        gamepad_type_arg,
        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='left_vesc_driver_node',
            parameters=[left_vesc_config],
            remappings=[
                ('sensors/servo_position_command', 'left_vesc/sensors/servo_position_command'),
                ('commands/motor/duty_cycle', 'left_vesc/commands/motor/duty_cycle'),
                ('commands/motor/current', 'left_vesc/commands/motor/current'),
                ('commands/motor/brake', 'left_vesc/commands/motor/brake'),
                ('commands/motor/speed', 'left_vesc/commands/motor/speed'),
                ('commands/motor/position', 'left_vesc/commands/motor/position'),
                ('commands/servo/position', 'left_vesc/commands/servo/position'),
            ]
        ),
        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='right_vesc_driver_node',
            parameters=[right_vesc_config],
            remappings=[
                ('sensors/servo_position_command', 'right_vesc/sensors/servo_position_command'),
                ('commands/motor/duty_cycle', 'right_vesc/commands/motor/duty_cycle'),
                ('commands/motor/current', 'right_vesc/commands/motor/current'),
                ('commands/motor/brake', 'right_vesc/commands/motor/brake'),
                ('commands/motor/speed', 'right_vesc/commands/motor/speed'),
                ('commands/motor/position', 'right_vesc/commands/motor/position'),
                ('commands/servo/position', 'right_vesc/commands/servo/position'),
            ]
        ),
        Node(
            package='differential_drive',
            executable='diff_vel_ctrl_node',
            name='differential_drive_node',
            parameters=[
                {'left_vel_topic': 'left_vesc/commands/motor/speed'},
                {'right_vel_topic': 'right_vesc/commands/motor/speed'}
            ]
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
        ),
        Node(
            package='differential_drive',
            executable='sony_gamepad_node',
            name='sony_gamepad_node',
            condition=IfCondition(
                PythonExpression(["'", LaunchConfiguration('gamepad_type'), "' == 'sony'"])
            )
        ),
        Node(
            package='differential_drive',
            executable='logitech_gamepad_node',
            name='logitech_gamepad_node',
            condition=IfCondition(
                PythonExpression(["'", LaunchConfiguration('gamepad_type'), "' == 'logitech'"])
            )
        ),
    ])
