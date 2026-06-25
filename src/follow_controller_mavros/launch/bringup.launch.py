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
        DeclareLaunchArgument('fcu_url',     default_value='serial:///dev/ttyTHS1:921600'),
        DeclareLaunchArgument('gcs_url',     default_value='udp://@10.61.52.76:14550'),
        DeclareLaunchArgument('takeoff_alt', default_value='2.0'),
        DeclareLaunchArgument('active_s',    default_value='20.0'),

        Node(
            package='mavros',
            executable='mavros_node',
            parameters=[
                px4_config,
                custom_config,
                {
                    'fcu_url': LaunchConfiguration('fcu_url'),
                    'gcs_url': LaunchConfiguration('gcs_url'),
                    'target_system_id':    1,
                    'target_component_id': 1,
                    'plugin_allowlist': [
                        'command',           # /mavros/cmd/arming
                        'sys_status',        # /mavros/state + /mavros/set_mode
                        'sys_time',
                        'setpoint_velocity', # /mavros/setpoint_velocity/cmd_vel_unstamped
                        'setpoint_position', # /mavros/setpoint_position/local
                        'local_position',    # /mavros/local_position/pose
                    ],
                },
            ],
            output='screen',
        ),

        Node(
            package='follow_controller_mavros',
            executable='flight_manager',
            name='flight_manager',
            parameters=[{
                'takeoff_alt': LaunchConfiguration('takeoff_alt'),
                'active_s':    LaunchConfiguration('active_s'),
            }],
            output='screen',
        ),
    ])
