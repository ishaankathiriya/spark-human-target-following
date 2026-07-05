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

        # Controller args
        DeclareLaunchArgument('follow_distance', default_value='2.5'),
        DeclareLaunchArgument('max_xy_speed',    default_value='0.5'),
        DeclareLaunchArgument('kp_dist',         default_value='0.5'),
        DeclareLaunchArgument('kd_dist',         default_value='0.3'),

        # RealSense + YOLO tracker → /tracker/centroid_3d
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tracker_launch),
            launch_arguments={
                'confidence': LaunchConfiguration('confidence'),
                'target_id':  LaunchConfiguration('target_id'),
            }.items(),
        ),

        # XY-only follow controller (vz=0, yaw_rate=0)
        Node(
            package='follow_controller_mavros',
            executable='pd_follow_xy',
            name='pd_follow_xy',
            parameters=[{
                'follow_distance': LaunchConfiguration('follow_distance'),
                'max_xy_speed':    LaunchConfiguration('max_xy_speed'),
                'kp_dist':         LaunchConfiguration('kp_dist'),
                'kd_dist':         LaunchConfiguration('kd_dist'),
            }],
            output='screen',
        ),
    ])
