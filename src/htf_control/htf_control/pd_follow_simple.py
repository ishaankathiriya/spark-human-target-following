#!/usr/bin/env python3
"""
PDFollowSimple — adapted from pd_follow_controller_mavros.py (Jetson branch).

Key differences vs pd_follow_controller.py:
  - Altitude reference is locked from the first `alt_calib_n` centroid readings
    (drone sees human at a certain camera-Y at the moment it reaches altitude, and
    holds that as the desired reference + offset). This prevents the P-term from
    fighting the drone's own pitch via camera projection changes.
  - Yaw control uses atan2(x_cam, z_cam) bearing rather than raw pixel-x error.
  - Lateral strafe is available but off by default (yaw handles lateral centering).

Transport: uXRCE-DDS / PX4 topics — NOT MAVROS.
State machine: PRIMING -> ARMING -> TAKEOFF -> HOLD <-> FOLLOW
               (identical to pd_follow_controller.py)
"""

import math
import signal
import time
from enum import IntEnum

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy)

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from geometry_msgs.msg import PointStamped, Vector3Stamped


# ── Alpha-beta filter ─────────────────────────────────────────────────────────

class _AxisEstimator:
    _MIN_DT = 1e-3

    def __init__(self, alpha, beta, max_plausible_speed=8.0):
        self._alpha = alpha
        self._beta  = beta
        self._max_v = max_plausible_speed
        self._pos   = 0.0
        self._vel   = 0.0
        self._t     = 0.0
        self._ready = False

    def update(self, meas, t):
        if not self._ready:
            self._pos, self._vel, self._t, self._ready = meas, 0.0, t, True
            return self._pos, 0.0
        dt         = max(t - self._t, self._MIN_DT)
        pred       = self._pos + self._vel * dt
        innovation = meas - pred
        if abs(innovation) > self._max_v * dt:
            self._t = t
            return self._pos, self._vel
        self._t   = t
        self._pos = pred + self._alpha * innovation
        self._vel = self._vel + (self._beta / dt) * innovation
        return self._pos, self._vel

    def predict(self, t):
        if not self._ready:
            return 0.0, 0.0
        dt = max(t - self._t, 0.0)
        return self._pos + self._vel * dt, self._vel

    @property
    def ready(self):
        return self._ready


# ── State machine ─────────────────────────────────────────────────────────────

class _State(IntEnum):
    PRIMING = 0
    ARMING  = 1
    TAKEOFF = 2
    HOLD    = 3
    FOLLOW  = 4


# ── QoS ──────────────────────────────────────────────────────────────────────

_PX4_PUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST, depth=1)

_PX4_SUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=1)

_ARMING_STATE_NAMES = {
    VehicleStatus.ARMING_STATE_DISARMED: 'DISARMED',
    VehicleStatus.ARMING_STATE_ARMED:    'ARMED',
}


# ── Node ──────────────────────────────────────────────────────────────────────

class PDFollowSimple(Node):

    _PRIMING_CYCLES = 20
    _TAKEOFF_TOL    = 0.3   # metres — transition to HOLD when within this of target z
    _TARGET_TIMEOUT = 1.0   # seconds without detection before FOLLOW → HOLD
    _LOOP_DT        = 0.02  # 50 Hz

    def __init__(self):
        super().__init__('pd_follow_simple')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('takeoff_altitude',    1.5)   # m AGL
        self.declare_parameter('follow_distance',     2.5)   # m depth to maintain
        self.declare_parameter('max_xy_speed',        0.8)   # m/s
        self.declare_parameter('max_yaw_rate',        0.8)   # rad/s
        self.declare_parameter('max_z_speed',         0.5)   # m/s

        self.declare_parameter('ab_alpha',            0.5)
        self.declare_parameter('ab_beta',             0.08)
        self.declare_parameter('max_plausible_speed', 8.0)

        self.declare_parameter('ff_dist',             1.0)   # feedforward on depth velocity
        self.declare_parameter('kp_dist',             0.5)   # P on depth error
        self.declare_parameter('kd_dist',             0.3)   # D on depth velocity

        self.declare_parameter('kp_yaw',              1.5)   # rad/s per radian of bearing
        self.declare_parameter('yaw_deadband',        0.05)  # rad (~3 deg)

        self.declare_parameter('ff_lat',              0.0)
        self.declare_parameter('kp_lat',              0.0)
        self.declare_parameter('kd_lat',              0.0)
        self.declare_parameter('lat_deadband',        0.08)  # m

        self.declare_parameter('fixed_altitude',      0.8)   # m AGL, held throughout FOLLOW
        self.declare_parameter('kp_alt',              0.5)   # vel P-gain on altitude error (m/s per m)

        self.declare_parameter('lookahead_t',         0.4)   # seconds

        self._takeoff_alt = self.get_parameter('takeoff_altitude').value
        self._follow_dist = self.get_parameter('follow_distance').value
        self._max_xy      = self.get_parameter('max_xy_speed').value
        self._max_yaw     = self.get_parameter('max_yaw_rate').value
        self._max_z       = self.get_parameter('max_z_speed').value
        ab_alpha          = self.get_parameter('ab_alpha').value
        ab_beta           = self.get_parameter('ab_beta').value
        max_spd           = self.get_parameter('max_plausible_speed').value
        self._ff_dist     = self.get_parameter('ff_dist').value
        self._kp_dist     = self.get_parameter('kp_dist').value
        self._kd_dist     = self.get_parameter('kd_dist').value
        self._kp_yaw      = self.get_parameter('kp_yaw').value
        self._yaw_db      = self.get_parameter('yaw_deadband').value
        self._ff_lat      = self.get_parameter('ff_lat').value
        self._kp_lat      = self.get_parameter('kp_lat').value
        self._kd_lat      = self.get_parameter('kd_lat').value
        self._lat_db      = self.get_parameter('lat_deadband').value
        self._fixed_alt   = self.get_parameter('fixed_altitude').value
        self._kp_alt      = self.get_parameter('kp_alt').value
        self._lookahead   = self.get_parameter('lookahead_t').value

        if ab_beta >= 2.0 * ab_alpha:
            ab_beta = 0.99 * 2.0 * ab_alpha
            self.get_logger().warn(f'ab_beta clamped to {ab_beta:.3f}')

        # [0] = lateral (x_cam), [1] = altitude (y_cam, down+), [2] = depth (z_cam)
        self._ab = [_AxisEstimator(ab_alpha, ab_beta, max_spd) for _ in range(3)]

        # ── PX4 publishers ────────────────────────────────────────────────────
        self._ocm_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', _PX4_PUB_QOS)
        self._sp_pub  = self.create_publisher(
            TrajectorySetpoint,  '/fmu/in/trajectory_setpoint',   _PX4_PUB_QOS)
        self._cmd_pub = self.create_publisher(
            VehicleCommand,      '/fmu/in/vehicle_command',        _PX4_PUB_QOS)

        # ── Debug publishers (same format as pd_follow_controller → works with debug_plotter) ──
        self._dbg_vel_pub = self.create_publisher(Vector3Stamped, '/htf/human_vel', 10)
        self._dbg_pos_pub = self.create_publisher(PointStamped,   '/htf/human_pos', 10)

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._local_pos_cb, _PX4_SUB_QOS)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4',
            self._vehicle_status_cb, _PX4_SUB_QOS)
        self.create_subscription(
            PointStamped, '/tracker/centroid_3d',
            self._centroid_cb, 10)

        # ── Internal state ────────────────────────────────────────────────────
        self._local_pos      = VehicleLocalPosition()
        self._vehicle_status = VehicleStatus()
        self._centroid_seen  = False
        self._last_detection = 0.0
        self._state          = _State.PRIMING
        self._cycle          = 0
        self._last_arm_t     = 0.0
        self._prev_arm_state = -1
        self._ground_z       = None
        self._takeoff_z_ned  = None
        self._takeoff_x      = None
        self._takeoff_y      = None
        self._hold_x         = None
        self._hold_y         = None
        self._hold_z         = None
        self._hold_yaw       = None

        self.create_timer(self._LOOP_DT, self._control_loop)
        self.get_logger().info(
            f'PDFollowSimple ready | '
            f'α={ab_alpha} β={ab_beta} gate={max_spd} m/s | '
            f'dist(ff={self._ff_dist} kp={self._kp_dist} kd={self._kd_dist}) | '
            f'yaw(kp={self._kp_yaw} db={math.degrees(self._yaw_db):.1f}°) | '
            f'alt(fixed={self._fixed_alt} m AGL, kp={self._kp_alt})'
        )

    # ── Subscriptions ─────────────────────────────────────────────────────────

    def _local_pos_cb(self, msg: VehicleLocalPosition):
        self._local_pos = msg

    def _vehicle_status_cb(self, msg: VehicleStatus):
        self._vehicle_status = msg

    def _centroid_cb(self, msg: PointStamped):
        t = time.monotonic()
        self._ab[0].update(msg.point.x, t)
        self._ab[1].update(msg.point.y, t)
        self._ab[2].update(msg.point.z, t)
        self._centroid_seen  = True
        self._last_detection = t

    # ── Control loop (50 Hz) ──────────────────────────────────────────────────

    def _control_loop(self):
        now      = time.monotonic()
        lp       = self._local_pos
        vs       = self._vehicle_status
        is_armed = vs.arming_state == VehicleStatus.ARMING_STATE_ARMED
        in_ob    = vs.nav_state    == VehicleStatus.NAVIGATION_STATE_OFFBOARD

        arm_state = vs.arming_state
        if arm_state != self._prev_arm_state:
            name = _ARMING_STATE_NAMES.get(arm_state, str(arm_state))
            self.get_logger().info(f'PX4 arming_state → {name} ({arm_state})')
            self._prev_arm_state = arm_state

        # ── PRIMING ───────────────────────────────────────────────────────────
        if self._state == _State.PRIMING:
            if self._ground_z is None:
                self._ground_z = lp.z
            self._publish_ocm(velocity=True)
            self._publish_velocity(0.0, 0.0, 0.0)
            self._cycle += 1
            if self._cycle >= self._PRIMING_CYCLES:
                self._state = _State.ARMING
                self.get_logger().info('Priming done — starting arm attempts.')
            return

        # ── ARMING ────────────────────────────────────────────────────────────
        if self._state == _State.ARMING:
            self._publish_ocm(velocity=True)
            self._publish_velocity(0.0, 0.0, 0.0)
            if is_armed and in_ob:
                self._takeoff_z_ned = self._ground_z - self._takeoff_alt
                self._takeoff_x     = lp.x
                self._takeoff_y     = lp.y
                self._state         = _State.TAKEOFF
                self.get_logger().info(
                    f'Armed + OFFBOARD — taking off to {self._takeoff_alt:.1f} m AGL '
                    f'(NED z = {self._takeoff_z_ned:.2f})')
            if now - self._last_arm_t > 1.0:
                self._last_arm_t = now
                self.get_logger().info(
                    f'Arm attempt — armed={is_armed} offboard={in_ob}')
                self._send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                self._send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            return

        # ── Lost arm or OFFBOARD mid-flight ───────────────────────────────────
        if not (is_armed and in_ob):
            self.get_logger().warn(
                f'Lost arm/OFFBOARD (armed={is_armed} offboard={in_ob}) — returning to ARMING.')
            self._state      = _State.ARMING
            self._last_arm_t = 0.0
            self._publish_ocm(velocity=True)
            self._publish_velocity(0.0, 0.0, 0.0)
            return

        # ── TAKEOFF ───────────────────────────────────────────────────────────
        if self._state == _State.TAKEOFF:
            self._publish_ocm(position=True)
            self._publish_position(self._takeoff_x, self._takeoff_y, self._takeoff_z_ned)
            if abs(lp.z - self._takeoff_z_ned) < self._TAKEOFF_TOL:
                self._hold_x   = self._takeoff_x
                self._hold_y   = self._takeoff_y
                self._hold_z   = self._takeoff_z_ned   # latch commanded z, not measured
                self._hold_yaw = lp.heading
                self._state    = _State.HOLD
                self.get_logger().info(
                    f'Reached altitude — HOLD at NED ({self._hold_x:.2f}, '
                    f'{self._hold_y:.2f}, {self._hold_z:.2f})')
            return

        has_target = (self._centroid_seen and
                      now - self._last_detection < self._TARGET_TIMEOUT)

        # ── HOLD ──────────────────────────────────────────────────────────────
        if not has_target:
            if self._state == _State.FOLLOW:
                self._hold_x   = lp.x
                self._hold_y   = lp.y
                self._hold_z   = lp.z
                self._hold_yaw = lp.heading
                self._state    = _State.HOLD
                self.get_logger().info('Target lost — HOLD')
            self._publish_ocm(position=True)
            self._publish_position(
                self._hold_x, self._hold_y, self._hold_z, yaw=self._hold_yaw)
            return

        # ── FOLLOW ────────────────────────────────────────────────────────────
        if self._state != _State.FOLLOW:
            self._state = _State.FOLLOW
            self.get_logger().info('Target acquired — FOLLOW')

        self._follow_target(now, lp.heading, lp.z)

    # ── PD follow ─────────────────────────────────────────────────────────────

    def _follow_target(self, now: float, heading: float, ned_z: float):
        pos = [self._ab[i].predict(now)[0] for i in range(3)]
        vel = [self._ab[i].predict(now)[1] for i in range(3)]

        # ── Depth axis (z_cam) → forward/backward body velocity ───────────────
        pred_dist_err = (pos[2] + vel[2] * self._lookahead) - self._follow_dist
        kp = self._kp_dist * 2.0 if pred_dist_err < 0 else self._kp_dist
        kd = self._kd_dist * 2.0 if pred_dist_err < 0 else self._kd_dist
        body_fwd = _clamp(
            self._ff_dist * vel[2] + kp * pred_dist_err + kd * vel[2],
            -self._max_xy, self._max_xy)

        # ── Yaw axis — bearing to human ───────────────────────────────────────
        # bearing > 0: human to the right → yaw right → CW in NED → positive yawspeed
        bearing = math.atan2(pos[0], pos[2])
        if abs(bearing) < self._yaw_db:
            bearing = 0.0
        yaw_rate = _clamp(self._kp_yaw * bearing, -self._max_yaw, self._max_yaw)

        # ── Lateral strafe axis (off by default, kp_lat=0) ────────────────────
        lat_p      = pos[0] if abs(pos[0]) >= self._lat_db else 0.0
        body_right = _clamp(
            self._ff_lat * vel[0] + self._kp_lat * lat_p + self._kd_lat * vel[0],
            -self._max_xy, self._max_xy)

        # ── Altitude — fixed height P controller (NED local position) ────────
        # target_ned_z is negative (up) since NED z is positive-down.
        # error > 0 means drone is below target → need negative vz (up).
        target_ned_z = self._ground_z - self._fixed_alt
        ned_vz = _clamp(
            self._kp_alt * (ned_z - target_ned_z),
            -self._max_z, self._max_z)

        # ── Body FRD → NED ────────────────────────────────────────────────────
        ch, sh = math.cos(heading), math.sin(heading)
        ned_vx = body_fwd * ch - body_right * sh
        ned_vy = body_fwd * sh + body_right * ch

        self._publish_ocm(velocity=True)
        self._publish_velocity(ned_vx, ned_vy, ned_vz, yaw_rate)

        # ── Debug publishers (for debug_plotter) ──────────────────────────────
        stamp = self.get_clock().now().to_msg()

        vel_msg = Vector3Stamped()
        vel_msg.header.stamp    = stamp
        vel_msg.header.frame_id = 'camera_link'
        vel_msg.vector.x = vel[0]   # lateral   (m/s, + = moving right)
        vel_msg.vector.y = vel[1]   # vertical  (m/s, + = moving down)
        vel_msg.vector.z = vel[2]   # depth     (m/s, + = moving away)
        self._dbg_vel_pub.publish(vel_msg)

        pos_msg = PointStamped()
        pos_msg.header.stamp    = stamp
        pos_msg.header.frame_id = 'camera_link'
        pos_msg.point.x = pos[0]   # lateral offset (m)
        pos_msg.point.y = pos[1]   # vertical offset (m)
        pos_msg.point.z = pos[2]   # depth (m)
        self._dbg_pos_pub.publish(pos_msg)

    # ── Publish helpers ───────────────────────────────────────────────────────

    def _publish_ocm(self, position=False, velocity=False):
        msg = OffboardControlMode()
        msg.position     = position
        msg.velocity     = velocity
        msg.acceleration = msg.attitude = msg.body_rate = False
        msg.timestamp    = self._now_us()
        self._ocm_pub.publish(msg)

    def _publish_velocity(self, vx, vy, vz, yaw_rate=0.0):
        nan = float('nan')
        msg = TrajectorySetpoint()
        msg.position     = [nan, nan, nan]
        msg.acceleration = [nan, nan, nan]
        msg.jerk         = [nan, nan, nan]
        msg.velocity     = [float(vx), float(vy), float(vz)]
        msg.yaw          = nan
        msg.yawspeed     = float(yaw_rate)
        msg.timestamp    = self._now_us()
        self._sp_pub.publish(msg)

    def _publish_position(self, x, y, z, yaw=float('nan')):
        nan = float('nan')
        msg = TrajectorySetpoint()
        msg.position     = [float(x), float(y), float(z)]
        msg.velocity     = [nan, nan, nan]
        msg.acceleration = [nan, nan, nan]
        msg.jerk         = [nan, nan, nan]
        msg.yaw          = float(yaw) if not math.isnan(yaw) else nan
        msg.yawspeed     = nan
        msg.timestamp    = self._now_us()
        self._sp_pub.publish(msg)

    def _send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command          = command
        msg.param1           = float(param1)
        msg.param2           = float(param2)
        msg.target_system    = msg.source_system    = 1
        msg.target_component = msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = self._now_us()
        self._cmd_pub.publish(msg)

    def _now_us(self):
        return int(time.time() * 1e6)

    def send_land_command(self):
        self.get_logger().warn('Sending LAND command.')
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)


# ── helpers ───────────────────────────────────────────────────────────────────

def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def main(args=None):
    rclpy.init(args=args)
    node = PDFollowSimple()
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(0)))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.send_land_command()
        deadline = time.monotonic() + 0.8
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
