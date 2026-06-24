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
                        'setpoint_velocity',
                        'vision_pose',
                    ],
                },
            ],
            output='screen',
        ),

        # Publishes static (0,0,0) pose at 30 Hz so EKF2 has a valid position source
        Node(
            package='follow_controller_mavros',
            executable='fake_pose_pub',
            name='fake_pose_pub',
            output='screen',
        ),

        # Streams zeros and switches to OFFBOARD
        Node(
            package='follow_controller_mavros',
            executable='offboard_mode_test',
            name='offboard_mode_test',
            output='screen',
        ),

        # Waits for OFFBOARD confirmation, then arms
        Node(
            package='follow_controller_mavros',
            executable='arm_node',
            name='arm_node',
            output='screen',
        ),
    ])
