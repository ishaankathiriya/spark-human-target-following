from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='htf_control',
            executable='hover_test',
            name='hover_test',
            output='screen',
            parameters=[{
                'takeoff_altitude':    1.5,   # metres AGL
                'ab_alpha':            0.5,   # position smoothing
                'ab_beta':             0.08,  # velocity gain (noise ≈ 0.073 m/s)
                'max_plausible_speed': 8.0,   # m/s — gate rejects sensor outliers
            }],
        )
    ])
