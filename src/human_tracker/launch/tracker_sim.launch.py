"""
Launch the 3D human tracker against the Gazebo simulation camera.

The tracker was written for a real RealSense; this file remaps its
hardcoded topics to the names produced by the ros_gz_bridge in simulation.

Real camera topics         → Gazebo bridge topics
/camera/camera/color/camera_info            → /camera/camera_info
/camera/camera/color/image_raw              → /camera/image
/camera/camera/aligned_depth_to_color/image_raw → /camera/depth_image
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('confidence', default_value='0.4'),
        DeclareLaunchArgument('target_id', default_value='-1'),

        Node(
            package='human_tracker',
            executable='tracker_3d_node',
            name='human_tracker_3d',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'confidence': LaunchConfiguration('confidence'),
                'target_id': LaunchConfiguration('target_id'),
            }],
            remappings=[
                ('/camera/camera/color/camera_info',
                 '/camera/camera_info'),
                ('/camera/camera/color/image_raw',
                 '/camera/image'),
                ('/camera/camera/aligned_depth_to_color/image_raw',
                 '/camera/depth_image'),
            ],
        ),
    ])
