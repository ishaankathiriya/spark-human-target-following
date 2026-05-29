from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    controller = Node(
        package='htf_control',
        executable='follow_controller',
        name='follow_controller',
        output='screen',
        parameters=[{
            # ── Flight geometry ───────────────────────────────────────────────
            'takeoff_altitude': 2.0,    # metres AGL
            'follow_distance':  1.5,    # metres behind centroid
            'height_offset':    0.5,    # metres: desired camera-y at equilibrium

            # ── Speed limits ──────────────────────────────────────────────────
            'max_xy_speed':     1.5,    # m/s
            'max_z_speed':      0.8,    # m/s
            'max_yaw_rate':     0.8,    # rad/s

            # ── Alpha-beta filter ─────────────────────────────────────────────
            # alpha: position gain — higher = more responsive, less smooth (0–1)
            # beta:  velocity gain — must be < 2 * alpha for stability
            #        critically damped: beta = alpha² / (2 − alpha)
            #        with alpha=0.6: critically damped beta ≈ 0.257
            'ab_alpha':         0.6,
            'ab_beta':          0.2,    # conservative: stable, slightly underdamped

            # ── Feedforward weight ────────────────────────────────────────────
            # 0.0 = pure proportional (reactive only)
            # 1.0 = full feedforward + P correction (ideal but sensitive to
            #        velocity estimate noise — increase ab_alpha/ab_beta first)
            'ff_gain':          0.7,

            # ── Proportional gains ────────────────────────────────────────────
            # Lower than the old kp_dist=0.6 because feedforward now handles
            # dynamic tracking; P only corrects residual steady-state error.
            'kp_dist':          0.4,    # m/s per m of depth error
            'kp_lat':           0.0,    # lateral strafe (0 = yaw-only)
            'kp_alt':           0.5,    # altitude
            'kp_yaw':           0.6,    # rad/s per m of lateral centroid offset

            # ── Deadband ──────────────────────────────────────────────────────
            'lat_deadband':     0.08,   # metres — suppresses yaw-hunting on jitter
        }],
    )

    return LaunchDescription([controller])
