from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    controller = Node(
        package='htf_control',
        executable='pd_follow_controller',
        name='pd_follow_controller',
        output='screen',
        parameters=[{
            # ── Flight geometry ───────────────────────────────────────────────
            'takeoff_altitude': 2.0,    # metres AGL
            'follow_distance':  1.5,    # metres behind centroid
            'height_offset':    0.5,    # camera-y equilibrium (human slightly below centre)

            # ── Speed limits ──────────────────────────────────────────────────
            'max_xy_speed':  1.5,       # m/s
            'max_z_speed':   0.8,       # m/s
            'max_yaw_rate':  0.8,       # rad/s

            # ── Alpha-beta filter ─────────────────────────────────────────────
            # alpha: position responsiveness (0–1). Higher → faster, noisier.
            # beta:  velocity responsiveness. Must satisfy: beta < 2 * alpha.
            'ab_alpha': 0.8,
            'ab_beta':  0.6,

            # ── Depth axis (camera z → body forward/backward) ─────────────────
            # ff: pre-match human's recession/approach velocity
            # kp: close the distance error (follow_distance - centroid_z)
            # kd: damp overshoot on approach/recession  (D = kd * vel[2])
            'ff_dist': 1.0,
            'kp_dist': 0.5,
            'kd_dist': 0.3,

            # ── Yaw axis (camera x → yaw rate) ───────────────────────────────
            # ff: pre-rotate when human moves laterally
            # kp: centre human in frame
            # kd: damp yaw oscillation
            'ff_yaw': 0.8,
            'kp_yaw': 0.6,
            'kd_yaw': 0.2,

            # ── Lateral strafe axis (camera x → body right) ───────────────────
            # Off by default — yaw handles lateral. Enable if strafing is needed.
            'ff_lat': 0.0,
            'kp_lat': 0.0,
            'kd_lat': 0.0,

            # ── Altitude axis (camera y → body up/down) ───────────────────────
            # ff: match vertical motion of human relative to camera
            # kp: correct height error (height_offset - centroid_y)
            # kd: damp vertical oscillation
            'ff_alt': 0.8,
            'kp_alt': 0.5,
            'kd_alt': 0.2,

            # ── Deadband ──────────────────────────────────────────────────────
            # Suppress yaw P-term and strafe when human is near lateral centre.
            # FF and D remain active inside the deadband.
            'lat_deadband': 0.08,       # metres
        }],
    )

    return LaunchDescription([controller])
