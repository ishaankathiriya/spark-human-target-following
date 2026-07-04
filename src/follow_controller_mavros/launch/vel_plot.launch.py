import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tracker_launch = os.path.join(
        get_package_share_directory('human_tracker'), 'launch', 'tracker.launch.py')

    return LaunchDescription([
        # Tracker args
        DeclareLaunchArgument('confidence', default_value='0.5'),
        DeclareLaunchArgument('target_id',  default_value='-1'),

        # RealSense + YOLO tracker → /tracker/centroid_3d
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tracker_launch),
            launch_arguments={
                'confidence': LaunchConfiguration('confidence'),
                'target_id':  LaunchConfiguration('target_id'),
            }.items(),
        ),

        # PD controller — computes and publishes velocity setpoints
        # Node(
        #     package='follow_controller_mavros',
        #     executable='pd_follow_controller_mavros',
        #     name='pd_follow_controller_mavros',
        #     output='screen',
        # ),

        Node(
            package='follow_controller_mavros',
            executable='pd_follow_xy',
            name='pd_follow_xy',
            output='screen',
        ),

        # Plotter — publishes /flight_ready to activate controller, then plots cmd_vel
        Node(
            package='follow_controller_mavros',
            executable='vel_plot_node',
            name='vel_plot_node',
            output='screen',
        ),
    ])
