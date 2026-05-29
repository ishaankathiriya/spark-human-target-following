#!/usr/bin/env python3
"""
HTF follow controller — alpha-beta filter + feedforward velocity controller.

Key differences from offboard_controller.py
────────────────────────────────────────────
• Alpha-beta filter (one per centroid axis) replaces the scalar EMA.
  It produces BOTH a smoothed position estimate AND a smoothed velocity
  estimate from the raw per-frame detections.

• Feedforward control law (in the follow state):
      v_drone = ff_gain * v_human_estimated + kp * position_error
  The drone pre-matches the human's velocity so it doesn't have to wait
  for a large position error before accelerating.  The P term corrects
  whatever residual error the feedforward doesn't cancel.

• No separate D gain (kd_dist) — the alpha-beta velocity estimate is
  already a clean, noise-suppressed derivative.  Adding an extra D term
  would double-count the derivative and amplify noise.

• Between centroid frames the filter extrapolates via constant-velocity
  prediction, so the control output is smooth even when the camera drops
  a frame.

State machine  (identical to offboard_controller.py)
─────────────────────────────────────────────────────
  PRIMING → ARMING → TAKEOFF → HOLD ↔ FOLLOW

Alpha-beta filter quick reference
───────────────────────────────────
  Predict:   pos_pred = pos + vel * dt
             vel_pred = vel                     (constant-velocity model)
  Correct:   residual = measurement - pos_pred
             pos ← pos_pred + alpha * residual
             vel ← vel_pred + (beta / dt) * residual

  alpha ∈ (0, 1): position responsiveness. Higher → tracks faster, noisier.
  beta  ∈ (0, 2α): velocity responsiveness. Higher → velocity adapts faster.
  Stability: beta < 2 * alpha.  Critically damped: beta = alpha² / (2 - alpha).

Topics consumed
───────────────
  /fmu/out/vehicle_local_position_v1   px4_msgs/VehicleLocalPosition
  /fmu/out/vehicle_status_v4           px4_msgs/VehicleStatus
  /tracker/centroid_3d                 geometry_msgs/PointStamped
      Camera optical frame: x = right, y = down, z = forward (depth)

Topics produced
───────────────
  /fmu/in/offboard_control_mode        px4_msgs/OffboardControlMode
  /fmu/in/trajectory_setpoint          px4_msgs/TrajectorySetpoint
  /fmu/in/vehicle_command              px4_msgs/VehicleCommand
"""

import math
import signal
import time
from enum import IntEnum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from geometry_msgs.msg import PointStamped


# ──────────────────────────────────────────────────────────────────────────────
# Alpha-beta filter  (single axis)
# ──────────────────────────────────────────────────────────────────────────────

class _AlphaBeta:
    """Single-axis alpha-beta tracker: position + velocity from noisy position."""

    _MIN_DT = 1e-3   # guard against zero/negative dt

    def __init__(self, alpha: float, beta: float):
        self._alpha = alpha
        self._beta  = beta
        self._pos   = 0.0
        self._vel   = 0.0
        self._t     = 0.0
        self._ready = False

    # ── Public interface ──────────────────────────────────────────────────────

    def update(self, meas: float, t: float) -> tuple[float, float]:
        """Incorporate a new measurement at time t.  Returns (pos, vel)."""
        if not self._ready:
            self._pos   = meas
            self._vel   = 0.0
            self._t     = t
            self._ready = True
            return self._pos, self._vel

        dt = max(t - self._t, self._MIN_DT)
        self._t = t

        # Predict
        pred_pos = self._pos + self._vel * dt

        # Correct
        residual   = meas - pred_pos
        self._pos  = pred_pos + self._alpha * residual
        self._vel  = self._vel + (self._beta / dt) * residual

        return self._pos, self._vel

    def predict(self, t: float) -> tuple[float, float]:
        """Extrapolate to time t without a new measurement.  Returns (pos, vel)."""
        if not self._ready:
            return 0.0, 0.0
        dt = max(t - self._t, 0.0)
        return self._pos + self._vel * dt, self._vel

    @property
    def ready(self) -> bool:
        return self._ready

    def reset(self):
        self._ready = False


# ──────────────────────────────────────────────────────────────────────────────
# State machine + QoS constants
# ──────────────────────────────────────────────────────────────────────────────

class _State(IntEnum):
    PRIMING  = 0
    ARMING   = 1
    TAKEOFF  = 2
    HOLD     = 3
    FOLLOW   = 4


_PX4_PUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
_PX4_SUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

_ARMING_STATE_NAMES = {
    0: 'INIT', 1: 'STANDBY', 2: 'ARMED',
    3: 'STANDBY_ERROR', 6: 'SHUTDOWN',
}


# ──────────────────────────────────────────────────────────────────────────────
# Node
# ──────────────────────────────────────────────────────────────────────────────

class FollowController(Node):

    _PREARM_CYCLES  = 20     # cycles before first arm attempt  (~0.4 s @ 50 Hz)
    _TARGET_TIMEOUT = 1.0    # seconds without detection → FOLLOW → HOLD
    _TAKEOFF_TOL    = 0.3    # metres: altitude error to exit TAKEOFF
    _LOOP_DT        = 0.02   # seconds: control loop period (50 Hz)

    def __init__(self):
        super().__init__('follow_controller')

        # ── Parameters ────────────────────────────────────────────────────────
        # Flight geometry
        self.declare_parameter('takeoff_altitude', 2.0)   # m AGL
        self.declare_parameter('follow_distance',  1.5)   # m behind centroid
        self.declare_parameter('height_offset',    0.5)   # m: desired camera-y at equilibrium
        # Speed limits
        self.declare_parameter('max_xy_speed',     1.5)   # m/s
        self.declare_parameter('max_z_speed',      0.8)   # m/s
        self.declare_parameter('max_yaw_rate',     0.8)   # rad/s
        # Alpha-beta filter
        self.declare_parameter('ab_alpha',         0.6)   # position gain (0–1); higher → more responsive
        self.declare_parameter('ab_beta',          0.2)   # velocity gain; must be < 2*alpha
        # Feedforward weight (0 = pure-P, 1 = full velocity feedforward + P correction)
        self.declare_parameter('ff_gain',          0.)
        # Proportional gains
        self.declare_parameter('kp_dist',          0.4)   # forward  (m/s per m of depth error)
        self.declare_parameter('kp_lat',           0.0)   # lateral body  (0 = yaw-only, no strafe)
        self.declare_parameter('kp_alt',           0.5)   # altitude
        self.declare_parameter('kp_yaw',           0.6)   # yaw  (rad/s per m of lateral error)
        # Lateral deadband: suppress yaw-hunting on small centroid jitter
        self.declare_parameter('lat_deadband',     0.08)  # metres

        self._takeoff_alt = self.get_parameter('takeoff_altitude').value
        self._follow_dist = self.get_parameter('follow_distance').value
        self._height_off  = self.get_parameter('height_offset').value
        self._max_xy      = self.get_parameter('max_xy_speed').value
        self._max_z       = self.get_parameter('max_z_speed').value
        self._max_yaw     = self.get_parameter('max_yaw_rate').value
        ab_alpha          = self.get_parameter('ab_alpha').value
        ab_beta           = self.get_parameter('ab_beta').value
        self._ff_gain     = self.get_parameter('ff_gain').value
        self._kp_d        = self.get_parameter('kp_dist').value
        self._kp_l        = self.get_parameter('kp_lat').value
        self._kp_a        = self.get_parameter('kp_alt').value
        self._kp_y        = self.get_parameter('kp_yaw').value
        self._lat_db      = self.get_parameter('lat_deadband').value

        if ab_beta >= 2.0 * ab_alpha:
            self.get_logger().warn(
                f'ab_beta={ab_beta} >= 2*ab_alpha={2*ab_alpha}: filter is unstable. '
                f'Clamping beta to {0.99 * 2 * ab_alpha:.3f}.')
            ab_beta = 0.99 * 2.0 * ab_alpha

        # One filter per camera axis: [0]=x(right), [1]=y(down), [2]=z(depth)
        self._ab = [_AlphaBeta(ab_alpha, ab_beta) for _ in range(3)]

        # ── Publishers ────────────────────────────────────────────────────────
        self._ocm_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', _PX4_PUB_QOS)
        self._sp_pub  = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint',   _PX4_PUB_QOS)
        self._cmd_pub = self.create_publisher(
            VehicleCommand,     '/fmu/in/vehicle_command',        _PX4_PUB_QOS)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self._local_pos_cb, _PX4_SUB_QOS)
        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v4',
            self._vehicle_status_cb, _PX4_SUB_QOS)
        self.create_subscription(
            PointStamped,
            '/tracker/centroid_3d',
            self._centroid_cb, 10)

        # ── Internal state ────────────────────────────────────────────────────
        self._local_pos      = VehicleLocalPosition()
        self._vehicle_status = VehicleStatus()
        self._centroid_seen  = False        # True once first centroid arrived
        self._last_detection = 0.0
        self._cycle          = 0
        self._last_arm_t     = 0.0
        self._prev_arm_state = -1

        self._ground_z      = None
        self._takeoff_z_ned = None
        self._takeoff_x     = None
        self._takeoff_y     = None
        self._hold_x        = None
        self._hold_y        = None
        self._hold_z        = None
        self._hold_yaw      = None
        self._state         = _State.PRIMING

        self.create_timer(self._LOOP_DT, self._control_loop)
        self.get_logger().info(
            f'FollowController ready.  '
            f'α={ab_alpha:.2f}  β={ab_beta:.2f}  ff={self._ff_gain:.2f}')

    # ──────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────────────

    def _local_pos_cb(self, msg: VehicleLocalPosition):
        self._local_pos = msg

    def _vehicle_status_cb(self, msg: VehicleStatus):
        self._vehicle_status = msg

    def _centroid_cb(self, msg: PointStamped):
        t  = time.monotonic()
        pt = msg.point
        self._ab[0].update(pt.x, t)   # lateral (right)
        self._ab[1].update(pt.y, t)   # vertical (down)
        self._ab[2].update(pt.z, t)   # depth (forward)
        self._centroid_seen  = True
        self._last_detection = t

    # ──────────────────────────────────────────────────────────────────────────
    # Control loop  (50 Hz)
    # ──────────────────────────────────────────────────────────────────────────

    def _control_loop(self):
        now      = time.monotonic()
        is_armed = (self._vehicle_status.arming_state ==
                    VehicleStatus.ARMING_STATE_ARMED)
        in_ob    = (self._vehicle_status.nav_state ==
                    VehicleStatus.NAVIGATION_STATE_OFFBOARD)

        arm_state = self._vehicle_status.arming_state
        if arm_state != self._prev_arm_state:
            name = _ARMING_STATE_NAMES.get(arm_state, str(arm_state))
            self.get_logger().info(
                f'PX4 arming_state → {name} ({arm_state})')
            self._prev_arm_state = arm_state

        # ── PRIMING ───────────────────────────────────────────────────────────
        if self._state == _State.PRIMING:
            if self._ground_z is None:
                self._ground_z = self._local_pos.z
            self._publish_ocm(velocity=True)
            self._publish_velocity(0.0, 0.0, 0.0, 0.0)
            self._cycle += 1
            if self._cycle >= self._PREARM_CYCLES:
                self._state = _State.ARMING
                self.get_logger().info('Priming done — starting arm attempts.')
            return

        # ── ARMING ────────────────────────────────────────────────────────────
        if self._state == _State.ARMING:
            self._publish_ocm(velocity=True)
            self._publish_velocity(0.0, 0.0, 0.0, 0.0)

            if is_armed and in_ob:
                self._takeoff_z_ned = self._ground_z - self._takeoff_alt
                current_agl = self._ground_z - self._local_pos.z
                if current_agl >= self._takeoff_alt - self._TAKEOFF_TOL:
                    self._enter_hold('already at target altitude')
                else:
                    self._takeoff_x = self._local_pos.x
                    self._takeoff_y = self._local_pos.y
                    self._state = _State.TAKEOFF
                    self.get_logger().info(
                        f'Armed + OFFBOARD.  Taking off to {self._takeoff_alt:.1f} m AGL '
                        f'(NED z = {self._takeoff_z_ned:.2f} m).')
                return

            if now - self._last_arm_t > 1.0:
                self._last_arm_t = now
                arm_name = _ARMING_STATE_NAMES.get(arm_state, str(arm_state))
                self.get_logger().info(
                    f'Arm attempt — PX4: {arm_name}, nav: {self._vehicle_status.nav_state}, '
                    f'armed={is_armed}, offboard={in_ob}')
                self._send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                self._send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            return

        # ── Lost arm or OFFBOARD mid-flight ───────────────────────────────────
        if not (is_armed and in_ob):
            self.get_logger().warn(
                f'Lost arm or OFFBOARD (armed={is_armed}, offboard={in_ob}) '
                f'— returning to ARMING.')
            self._state      = _State.ARMING
            self._last_arm_t = 0.0
            self._publish_ocm(velocity=True)
            self._publish_velocity(0.0, 0.0, 0.0, 0.0)
            return

        # ── TAKEOFF ───────────────────────────────────────────────────────────
        if self._state == _State.TAKEOFF:
            self._publish_ocm(position=True)
            self._publish_position(
                self._takeoff_x, self._takeoff_y, self._takeoff_z_ned,
                yaw=self._local_pos.heading)
            if abs(self._local_pos.z - self._takeoff_z_ned) < self._TAKEOFF_TOL:
                self._enter_hold('takeoff complete')
                self.get_logger().info(
                    f'Takeoff complete (NED z = {self._local_pos.z:.2f} m).')
            return

        # ── HOLD / FOLLOW ─────────────────────────────────────────────────────
        has_target = (self._centroid_seen and
                      (now - self._last_detection) < self._TARGET_TIMEOUT)

        if has_target:
            if self._state != _State.FOLLOW:
                self._state = _State.FOLLOW
                self.get_logger().info('Target acquired — entering FOLLOW.')
            self._publish_ocm(velocity=True)
            self._follow_target(now)
        else:
            if self._state != _State.HOLD:
                self._enter_hold('target lost')
            self._publish_ocm(position=True)
            self._publish_position(
                self._hold_x, self._hold_y, self._hold_z, yaw=self._hold_yaw)

    # ──────────────────────────────────────────────────────────────────────────
    # HOLD entry
    # ──────────────────────────────────────────────────────────────────────────

    def _enter_hold(self, reason: str = ''):
        self._hold_x   = self._local_pos.x
        self._hold_y   = self._local_pos.y
        self._hold_z   = self._local_pos.z
        self._hold_yaw = self._local_pos.heading
        self._state    = _State.HOLD
        tag = f' ({reason})' if reason else ''
        self.get_logger().info(
            f'Entering HOLD{tag} — NED '
            f'({self._hold_x:.2f}, {self._hold_y:.2f}, {self._hold_z:.2f}), '
            f'yaw={math.degrees(self._hold_yaw):.1f}°')

    # ──────────────────────────────────────────────────────────────────────────
    # Feedforward + P velocity controller
    # ──────────────────────────────────────────────────────────────────────────

    def _follow_target(self, now: float):
        heading = self._local_pos.heading

        # Query alpha-beta filters at current time (extrapolates between frames)
        pos = [self._ab[i].predict(now)[0] for i in range(3)]
        vel = [self._ab[i].predict(now)[1] for i in range(3)]
        # pos: [x_right, y_down, z_depth] in camera frame  (metres)
        # vel: rates of change of those positions           (m/s relative to drone)

        # ── Depth axis  (camera z = forward)  ────────────────────────────────
        # dist_err > 0 → human is farther than desired → move forward
        # vel[2] > 0   → human is moving away          → pre-accelerate forward
        dist_err  = pos[2] - self._follow_dist
        body_fwd  = _clamp(
            self._ff_gain * vel[2] + self._kp_d * dist_err,
            -self._max_xy, self._max_xy)

        # ── Lateral axis  (camera x = right)  ────────────────────────────────
        # P-only lateral strafe (kp_lat=0 by default → yaw-only control).
        # Feedforward and P yaw both use camera x:
        #   vel[0] > 0 → human moving right  → pre-yaw right
        #   lat_err > 0 → human off-centre right → yaw right
        lat_err    = pos[0]
        lat_err_p  = lat_err if abs(lat_err) >= self._lat_db else 0.0
        body_right = _clamp(self._kp_l * lat_err_p, -self._max_xy, self._max_xy)
        yaw_rate   = _clamp(
            self._ff_gain * vel[0] + self._kp_y * lat_err_p,
            -self._max_yaw, self._max_yaw)

        # ── Altitude axis  (camera y = down)  ────────────────────────────────
        # alt_err > 0 → human appears more below than desired → drone too high → descend
        # vel[1] > 0 → human moving down relative to camera  → descend to track
        # Both terms reduce body_up (negative = descend in body frame).
        alt_err  = pos[1] - self._height_off
        body_up  = _clamp(
            -self._ff_gain * vel[1] - self._kp_a * alt_err,
            -self._max_z, self._max_z)

        # ── Body → NED  ───────────────────────────────────────────────────────
        ch, sh = math.cos(heading), math.sin(heading)
        self._publish_velocity(
            body_fwd * ch - body_right * sh,   # NED vx
            body_fwd * sh + body_right * ch,   # NED vy
            -body_up,                           # NED vz (positive = down)
            yaw_rate,
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Publish helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _publish_ocm(self, position=False, velocity=False):
        msg = OffboardControlMode()
        msg.position     = position
        msg.velocity     = velocity
        msg.acceleration = False
        msg.attitude     = False
        msg.body_rate    = False
        msg.timestamp    = self._now_us()
        self._ocm_pub.publish(msg)

    def _publish_velocity(self, vx, vy, vz, yaw_rate=0.0):
        nan = float('nan')
        msg = TrajectorySetpoint()
        msg.position     = [nan, nan, nan]
        msg.velocity     = [float(vx), float(vy), float(vz)]
        msg.acceleration = [nan, nan, nan]
        msg.jerk         = [nan, nan, nan]
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

    def _send_vehicle_command(self, command: int, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command          = command
        msg.param1           = float(param1)
        msg.param2           = float(param2)
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = self._now_us()
        self._cmd_pub.publish(msg)

    def send_land_command(self):
        self.get_logger().warn('Sending LAND command to PX4.')
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def _now_us(self) -> int:
        return int(time.time() * 1e6)


# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────

def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = FollowController()

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
