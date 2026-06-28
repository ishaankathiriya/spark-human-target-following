from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    controller = Node(
        package='htf_control',
        executable='pd_follow_simple',
        name='pd_follow_simple',
        output='screen',
        parameters=[{
            # ── Flight geometry ───────────────────────────────────────────────
            'takeoff_altitude': 1.5,    # m AGL
            'follow_distance':  3.0,    # m depth to maintain

            # ── Speed limits ──────────────────────────────────────────────────
            'max_xy_speed':  1.5,       # m/s
            'max_z_speed':   1.5,       # m/s
            'max_yaw_rate':  0.8,       # rad/s

            # ── Alpha-beta filter ─────────────────────────────────────────────
            'ab_alpha':            0.5,
            'ab_beta':             0.08,
            'max_plausible_speed': 8.0,

            # ── Depth axis (camera z → forward/backward) ──────────────────────
            'ff_dist':  1.0,
            'kp_dist':  0.5,
            'kd_dist':  0.3,

            # ── Yaw axis (bearing = atan2(x_cam, z_cam)) ──────────────────────
            # kp_yaw: rad/s per radian of bearing error
            # yaw_deadband: suppress yaw P-term within this angle (rad)
            'kp_yaw':       1.5,
            'yaw_deadband': 0.05,   # ~3 deg

            # ── Lateral strafe (off by default — yaw handles centering) ───────
            'ff_lat':      0.0,
            'kp_lat':      0.0,
            'kd_lat':      0.0,
            'lat_deadband': 0.08,   # m

            # ── Altitude — fixed height ───────────────────────────────────────
            'fixed_altitude': 0.8,  # m AGL, held throughout FOLLOW
            'kp_alt':         0.5,  # m/s per m of altitude error

            # ── Lookahead ─────────────────────────────────────────────────────
            'lookahead_t': 0.4,     # seconds
        }],
    )

    return LaunchDescription([controller])
