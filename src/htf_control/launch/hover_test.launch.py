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
                'takeoff_altitude': 1.5,    # metres AGL — how high to hover
                'ab_alpha':         0.8,    # alpha-beta position gain
                'ab_beta':          0.6,    # alpha-beta velocity gain (must be < 2*alpha)
            }],
        )
    ])
