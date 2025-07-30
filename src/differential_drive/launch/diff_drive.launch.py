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
    # --- Define the path to the u-blox config file ---
    # This gets the zed_f9p_simple.yaml file from the ublox_gps package
    ublox_config = os.path.join(
        get_package_share_directory('ublox_gps'),
        'config',
        'zed_f9p_simple.yaml'
    )
    
    # Declare launch argument for gamepad type
    gamepad_type_arg = DeclareLaunchArgument(
        'gamepad_type',
        default_value='logitech',
        description='Type of gamepad to use: sony or logitech'
    )

    # --- RViz Configuration ---
    # rviz_config_file = os.path.join(get_package_share_directory('differential_drive'), 'config', 'path_following.rviz')
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config_file],
    #     output='screen'
    # )

    return LaunchDescription([
        gamepad_type_arg,
        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='left_vesc_driver_node',
            parameters=[left_vesc_config],
            remappings=[
                ('sensors/core', 'left_vesc/sensors/core'),
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
                ('sensors/core', 'right_vesc/sensors/core'),
                ('sensors/servo_position_command', 'right_vesc/sensors/servo_position_command'),
                ('commands/motor/duty_cycle', 'right_vesc/commands/motor/duty_cycle'),
                ('commands/motor/current', 'right_vesc/commands/motor/current'),
                ('commands/motor/brake', 'right_vesc/commands/motor/brake'),
                ('commands/motor/speed', 'right_vesc/commands/motor/speed'),
                ('commands/motor/position', 'right_vesc/commands/motor/position'),
                ('commands/servo/position', 'right_vesc/commands/servo/position'),
            ]
        ),
        # Velocity Control Node
        Node(
            package='differential_drive',
            executable='velocity_control_node',
            name='differential_drive_node', # This name is often used by other tools like teleop
            output='screen',
            parameters=[
                {"wheel_base": 0.57},
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
        # Add the u-blox GPS node to the launch list
        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            name='ublox_gps_node',
            # Load the YAML configuration file
            parameters=[ublox_config]
        ),
        # GPS accuracy monitor node
        Node(
            package='differential_drive',
            executable='gps_monitor_node',
            name='gps_monitor_node',
            output='screen'
        ),
        
        # # --- NEW: Launch the Odometry Test Node ---
        # Node(
        #     package='differential_drive',
        #     executable='odometry_test_node_enhanced',
        #     name='odometry_test_node_enhanced',
        #     output='screen'
        # ),
        # #--- OLD: Launch the Odometry Test Node ---
        Node(
            package='differential_drive',
            executable='odometry_test_node',
            name='odometry_test_node',
            output='screen'
        ),
        # Original Camera Node - This provides the image stream
        Node(
            package='differential_drive',
            executable='camera_node', # Make sure this executable is in your setup.py
            name='camera_node'
    ),

        
        # -----------------------------------------------------------------
        # --- STEPPER NODE SELECTION ---
        # --- Make sure only ONE of the following is uncommented! ---
        # -----------------------------------------------------------------

        # Option 1: Original enhanced Stepper Controller (Legacy - Do not use with IMU sketch)
        # Node(
        #     package='differential_drive',
        #     executable='stepper_controller_node',
        #     name='stepper_controller_node',
        #     parameters=[os.path.join(get_package_share_directory('differential_drive'), 'config', 'stepper_config.yaml')],
        #     output='screen'
        # ),
        
        # Option 2: New unified Stepper + IMU node (CORRECT ONE for stepper_controller_with_imu.ino)
        Node(
            package='differential_drive',
            executable='stepper_imu_node',
            name='stepper_imu_node',
            parameters=[os.path.join(get_package_share_directory('differential_drive'), 'config', 'stepper_imu_config.yaml')],
            output='screen'
        ),

        # YOLO Divot Detector Node
        Node(
            package='differential_drive',
            executable='divot_detector',
            name='divot_detector',
            output='screen'
        ),

        # Autonomous Alignment and Repair Node
        Node(
            package='differential_drive',
            executable='align_and_repair_node',
            name='align_and_repair_node',
            output='screen'
        ),

        # Odometry Publisher Node
        Node(
            package='differential_drive',
            executable='odometry_node',
            name='odometry_node',
            output='screen'
        ),

        # Path Follower Node
        Node(
            package='differential_drive',
            executable='path_follower_node',
            name='path_follower_node',
            output='screen'
        ),

        # GolfBot GUI Node
        Node(
            package='differential_drive',
            executable='golfbot_gui_node',
            name='golfbot_gui_node',
            output='screen'
        ),

        # Add RViz to the launch description
        # rviz_node
    ])
