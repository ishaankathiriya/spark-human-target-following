from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    controller = Node(
        package='htf_control',
        executable='offboard_controller',
        name='offboard_controller',
        output='screen',
        parameters=[{
            'takeoff_altitude': 2.0,    # metres AGL
            'follow_distance':  0.5,    # metres behind centroid
            'height_offset':    0.5,    # metres above centroid
            'max_xy_speed':     1.5,    # m/s
            'max_z_speed':      0.8,    # m/s
            'max_yaw_rate':     0.8,    # rad/s
            # PD on forward distance
            'kp_dist':          0.6,
            'kd_dist':          0.4,    # brakes when gap is closing
            # P on lateral body (0 = yaw-only, no sideways translation)
            'kp_lat':           0.0,
            # P on altitude
            'kp_alt':           0.5,
            # P on yaw
            'kp_yaw':           0.8,
            # EMA smoothing (0=max smooth, 1=raw)
            'ema_alpha':        0.4,
            # Lateral deadband to suppress yaw-hunting (metres)
            'lat_deadband':     0.08,
        }],
    )

    return LaunchDescription([controller])
