from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('confidence', default_value='0.5'),
        DeclareLaunchArgument('target_id', default_value='-1'),

        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'align_depth.enable': True,
                'color_width': 640,
                'color_height': 480,
                'color_fps': 30.0,
                'depth_width': 640,
                'depth_height': 480,
                'depth_fps': 30.0,
            }],
        ),

        Node(
            package='human_tracker',
            executable='tracker_3d_node',
            name='human_tracker_3d',
            parameters=[{
                'model': '/home/btp/spark-human-target-following/yolov8n_480.engine',
                'confidence': LaunchConfiguration('confidence'),
                'target_id': LaunchConfiguration('target_id'),
            }],
            output='screen',
        ),
    ])
