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
            'follow_distance':  2.5,    # metres behind centroid
            'height_offset':    0.6,    # camera-y equilibrium (human slightly below centre)

            # ── Speed limits ──────────────────────────────────────────────────
            'max_xy_speed':  1.5,       # m/s
            'max_z_speed':   0.8,       # m/s
            'max_yaw_rate':  0.8,       # rad/s

            # ── Alpha-beta filter ─────────────────────────────────────────────
            # alpha: position responsiveness (0–1). Higher → faster, noisier.
            # beta:  velocity responsiveness. Must satisfy: beta < 2 * alpha.
            'ab_alpha':            0.5,   # position smoothing
            'ab_beta':             0.08,  # velocity gain (noise ≈ 0.073 m/s)
            'max_plausible_speed': 8.0,   # m/s — gate rejects sensor outliers

            # ── Depth axis (camera z → body forward/backward) ─────────────────
            # ff: pre-match human's recession/approach velocity
            # kp: close the distance error (follow_distance - centroid_z)
            # kd: damp overshoot on approach/recession  (D = kd * vel[2])
            'ff_dist': 0.6,
            'kp_dist': 0.6,
            'kd_dist': 0.9,

            # ── Yaw axis — bearing-angle P controller ────────────────────────
            # yaw_rate = kp_yaw * atan2(lateral_offset, depth)
            # kp_yaw units: rad/s per radian of bearing error.
            # yaw_deadband: angles below this treated as zero (prevents hunting).
            'kp_yaw':       1.5,    # rad/s per radian
            'yaw_deadband': 0.05,   # radians (~3°)

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

            # ── Lookahead ─────────────────────────────────────────────────────
            # P term acts on the predicted position T seconds from now.
            # Drone starts backing up while human is still at follow_distance
            # and approaching, giving PX4 time to decelerate before overshoot.
            # Tune with: lookahead_t ≈ max_xy_speed / MPC_ACC_HOR
            # With max_xy=1.5 m/s and MPC_ACC_HOR=6 m/s²: 1.5/6 = 0.25 s
            # Start at 0.4 s (a bit conservative) and reduce if drone backs up too eagerly.
            'lookahead_t': 0.4,         # seconds
        }],
    )

    return LaunchDescription([controller])
