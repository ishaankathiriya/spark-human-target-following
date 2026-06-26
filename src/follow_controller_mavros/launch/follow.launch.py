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

        # Follow controller args
        DeclareLaunchArgument('follow_distance', default_value='2.5'),
        DeclareLaunchArgument('max_xy_speed',    default_value='0.8'),
        DeclareLaunchArgument('max_yaw_rate',    default_value='0.5'),
        DeclareLaunchArgument('kp_dist',         default_value='0.5'),
        DeclareLaunchArgument('kd_dist',         default_value='0.3'),
        DeclareLaunchArgument('kp_yaw',          default_value='1.5'),
        DeclareLaunchArgument('alt_offset',      default_value='0.1'),
        DeclareLaunchArgument('kp_z',            default_value='0.4'),
        DeclareLaunchArgument('kd_z',            default_value='0.2'),
        DeclareLaunchArgument('max_z_speed',     default_value='0.5'),

        # RealSense camera + YOLO tracker → publishes /tracker/centroid_3d
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tracker_launch),
            launch_arguments={
                'confidence': LaunchConfiguration('confidence'),
                'target_id':  LaunchConfiguration('target_id'),
            }.items(),
        ),

        # PD follow controller — waits for /flight_ready, then takes over setpoints
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
                'alt_offset':      LaunchConfiguration('alt_offset'),
                'kp_z':            LaunchConfiguration('kp_z'),
                'kd_z':            LaunchConfiguration('kd_z'),
                'max_z_speed':     LaunchConfiguration('max_z_speed'),
            }],
            output='screen',
        ),
    ])
