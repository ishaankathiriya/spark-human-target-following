import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mavros_share  = get_package_share_directory('mavros')
    pkg_share     = get_package_share_directory('follow_controller_mavros')
    px4_config    = os.path.join(mavros_share, 'launch', 'px4_config.yaml')
    custom_config = os.path.join(pkg_share, 'config', 'px4_custom.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', default_value='serial:///dev/ttyTHS1:921600'),
        DeclareLaunchArgument('gcs_url', default_value='udp://@10.61.52.76:14550'),

        Node(
            package='mavros',
            executable='mavros_node',
            # name='mavros',
            parameters=[
                px4_config,     # MAVROS defaults
                custom_config,  # our overrides (plugin params)
                {
                    'fcu_url': LaunchConfiguration('fcu_url'),
                    # 'gcs_url': LaunchConfiguration('gcs_url'),
                    'gcs_url' : 'udp://@10.205.72.219:14550',
                    'target_system_id': 1,
                    'target_component_id': 1,
                    'plugin_allowlist': [
                        'command',
                        'sys_status',
                        'sys_time',
                        'setpoint_velocity',
                        'setpoint_position',
                        'setpoint_attitude',
                        'local_position',
                        'setpoint_raw',
                    ],
                },
            ],
            output='screen',
        ),

        Node(
            package='follow_controller_mavros',
            executable='offboard_mode_test',
            name='offboard_mode_test',
            output='screen',
        ),
    ])
