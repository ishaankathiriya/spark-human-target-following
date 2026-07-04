from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('width',     default_value='4.0'),
        DeclareLaunchArgument('height',    default_value='4.0'),
        DeclareLaunchArgument('speed',     default_value='0.5'),
        DeclareLaunchArgument('clockwise', default_value='false'),

        Node(
            package='follow_controller_mavros',
            executable='rect_pos_node',
            name='rect_pos_node',
            parameters=[{
                'width':     LaunchConfiguration('width'),
                'height':    LaunchConfiguration('height'),
                'speed':     LaunchConfiguration('speed'),
                'clockwise': LaunchConfiguration('clockwise'),
            }],
            output='screen',
        ),
    ])
