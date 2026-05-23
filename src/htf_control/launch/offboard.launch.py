from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    controller = Node(
        package='htf_control',
        executable='offboard_controller',
        name='offboard_controller',
        output='screen',
        parameters=[{
            'follow_distance': 2.0,
            'max_xy_speed': 1.5,
            'max_z_speed': 0.8,
            'max_yaw_rate': 0.8,
            'kp_dist': 0.5,
            'kp_lat': 0.0,
            'kp_alt': 0.4,
            'kp_yaw': 0.6,
        }],
    )

    return LaunchDescription([controller])
