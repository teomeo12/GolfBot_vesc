from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            remappings=[('/gps/fix', '/fix')],  # Remap the input topic
            parameters=[{
                'yaw_offset': 1.5707963,  # 90 degrees in radians (adjust if needed)
                'magnetic_declination_radians': 0.0,  # Adjust for your location if needed
                'broadcast_utm_transform': True,
                'publish_filtered_gps': True,
                'use_odometry_yaw': False,
                'wait_for_datum': False
            }]
        )
    ]) 