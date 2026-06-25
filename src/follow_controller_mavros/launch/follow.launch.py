from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('follow_distance', default_value='2.5'),
        DeclareLaunchArgument('max_xy_speed',    default_value='0.8'),
        DeclareLaunchArgument('max_yaw_rate',    default_value='0.5'),
        DeclareLaunchArgument('kp_dist',         default_value='0.5'),
        DeclareLaunchArgument('kd_dist',         default_value='0.3'),
        DeclareLaunchArgument('kp_yaw',          default_value='1.5'),

        Node(
            package='follow_controller_mavros',
            executable='pd_follow_controller_mavros',
            name='pd_follow_controller_mavros',
            parameters=[{
                'follow_distance': LaunchConfiguration('follow_distance'),
                'max_xy_speed':    LaunchConfiguration('max_xy_speed'),
                'max_yaw_rate':    LaunchConfiguration('max_yaw_rate'),
                'kp_dist':         LaunchConfiguration('kp_dist'),
                'kd_dist':         LaunchConfiguration('kd_dist'),
                'kp_yaw':          LaunchConfiguration('kp_yaw'),
            }],
            output='screen',
        ),
    ])
