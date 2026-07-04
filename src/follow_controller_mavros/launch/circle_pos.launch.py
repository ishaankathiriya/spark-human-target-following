from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('radius',    default_value='3.0'),
        DeclareLaunchArgument('speed',     default_value='0.5'),
        DeclareLaunchArgument('clockwise', default_value='false'),

        Node(
            package='follow_controller_mavros',
            executable='circle_pos_node',
            name='circle_pos_node',
            parameters=[{
                'radius':    LaunchConfiguration('radius'),
                'speed':     LaunchConfiguration('speed'),
                'clockwise': LaunchConfiguration('clockwise'),
            }],
            output='screen',
        ),
    ])
