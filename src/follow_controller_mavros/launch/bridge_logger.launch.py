import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mavros_share = get_package_share_directory('mavros')
    px4_config = os.path.join(mavros_share, 'launch', 'px4_config.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', default_value='serial:///dev/ttyTHS1:921600'),
        DeclareLaunchArgument('gcs_url', default_value='udp://@10.61.52.76:14550'),

        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            parameters=[
                px4_config,
                {
                    'fcu_url': LaunchConfiguration('fcu_url'),
                    'gcs_url': LaunchConfiguration('gcs_url'),
                    'target_system_id': 1,
                    'target_component_id': 1,
                    'plugin_allowlist': [
                        'command',
                        'sys_status',
                        'sys_time',
                        'local_position',
                        'imu',
                    ],
                },
            ],
            output='screen',
        ),

        Node(
            package='follow_controller_mavros',
            executable='bridge_logger',
            name='bridge_logger',
            output='screen',
        ),
    ])
