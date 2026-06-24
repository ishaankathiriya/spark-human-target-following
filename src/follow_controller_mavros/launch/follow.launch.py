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
        DeclareLaunchArgument('fcu_url',         default_value='serial:///dev/ttyACM0:57600'),
        DeclareLaunchArgument('follow_distance',  default_value='2.5'),
        DeclareLaunchArgument('max_xy_speed',     default_value='0.8'),
        DeclareLaunchArgument('max_yaw_rate',     default_value='0.5'),
        DeclareLaunchArgument('kp_dist',          default_value='0.5'),
        DeclareLaunchArgument('kd_dist',          default_value='0.3'),
        DeclareLaunchArgument('kp_yaw',           default_value='1.5'),

        # MAVROS — connects to the FC over serial
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            parameters=[
                px4_config,
                {
                    'fcu_url': LaunchConfiguration('fcu_url'),
                    'gcs_url': '',
                    'target_system_id': 1,
                    'target_component_id': 1,
                    # Allowlist only what we need — MAVROS 2.14.0 has a type-conflict
                    # bug in several plugins (companion_process_status, debug_value, ...)
                    # that causes a crash. Loading only required plugins avoids all of them.
                    'plugin_allowlist': [
                        'command',           # arming service at /mavros/mavros/arming
                        'sys_status',        # /mavros/state + /mavros/set_mode
                        'sys_time',          # time sync with FC
                        'local_position',    # pose at /mavros/mavros/pose
                        'setpoint_velocity', # cmd_vel at /mavros/mavros/cmd_vel_unstamped
                        'setpoint_position', # position setpoint at /mavros/mavros/local
                    ],
                },
            ],
            output='screen',
        ),

        # PD follow controller
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
