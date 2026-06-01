#!/usr/bin/env python3
"""
HTF PD follow controller — explicit feedforward + PD per axis.

Control law (applied independently on each axis):
    v_cmd = ff * v_human + kp * pos_error + kd * v_human

Where:
    v_human   = alpha-beta velocity estimate (relative velocity of human w.r.t. drone)
    pos_error = centroid_<axis> - desired_<axis>
    kd*v_human = D term, because d(pos_error)/dt = d(centroid)/dt = v_human
                 (desired position is constant, so its derivative is zero)

Three controlled axes (separate ff/kp/kd gains for each):
    Depth    : centroid z → body forward/backward velocity
    Yaw      : centroid x → yaw rate  (+ optional lateral strafe)
    Altitude : centroid y → body up/down velocity

State machine (identical to other controllers):
    PRIMING → ARMING → TAKEOFF → HOLD ↔ FOLLOW

Topics consumed:
    /fmu/out/vehicle_local_position_v1  (px4_msgs/VehicleLocalPosition)
    /fmu/out/vehicle_status_v4          (px4_msgs/VehicleStatus)
    /tracker/centroid_3d                (geometry_msgs/PointStamped)
        camera optical frame: x=right, y=down, z=forward (depth)

Topics produced:
    /fmu/in/offboard_control_mode       (px4_msgs/OffboardControlMode)
    /fmu/in/trajectory_setpoint         (px4_msgs/TrajectorySetpoint)
    /fmu/in/vehicle_command             (px4_msgs/VehicleCommand)
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
# Alpha-beta filter (single axis)
# ──────────────────────────────────────────────────────────────────────────────

class _AlphaBeta:
    """Single-axis position + velocity estimator from noisy position measurements."""

    _MIN_DT = 1e-3

    def __init__(self, alpha: float, beta: float):
        self._alpha = alpha
        self._beta  = beta
        self._pos   = 0.0
        self._vel   = 0.0
        self._t     = 0.0
        self._ready = False

    def update(self, meas: float, t: float) -> tuple[float, float]:
        if not self._ready:
            self._pos, self._vel, self._t, self._ready = meas, 0.0, t, True
            return self._pos, self._vel
        dt         = max(t - self._t, self._MIN_DT)
        self._t    = t
        pred_pos   = self._pos + self._vel * dt
        residual   = meas - pred_pos
        self._pos  = pred_pos + self._alpha * residual
        self._vel  = self._vel + (self._beta / dt) * residual
        return self._pos, self._vel

    def predict(self, t: float) -> tuple[float, float]:
        if not self._ready:
            return 0.0, 0.0
        dt = max(t - self._t, 0.0)
        return self._pos + self._vel * dt, self._vel

    @property
    def ready(self) -> bool:
        return self._ready


# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

class _State(IntEnum):
    PRIMING = 0
    ARMING  = 1
    TAKEOFF = 2
    HOLD    = 3
    FOLLOW  = 4


_PX4_PUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST, depth=1)

_PX4_SUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=1)

_ARMING_STATE_NAMES = {
    0: 'INIT', 1: 'STANDBY', 2: 'ARMED', 3: 'STANDBY_ERROR', 6: 'SHUTDOWN'}


# ──────────────────────────────────────────────────────────────────────────────
# Node
# ──────────────────────────────────────────────────────────────────────────────

class PDFollowController(Node):

    _PREARM_CYCLES  = 20
    _TARGET_TIMEOUT = 1.0
    _TAKEOFF_TOL    = 0.3
    _LOOP_DT        = 0.02

    def __init__(self):
        super().__init__('pd_follow_controller')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('takeoff_altitude', 2.0)
        self.declare_parameter('follow_distance',  1.5)
        self.declare_parameter('height_offset',    0.5)

        self.declare_parameter('max_xy_speed',  1.5)
        self.declare_parameter('max_z_speed',   0.8)
        self.declare_parameter('max_yaw_rate',  0.8)

        self.declare_parameter('ab_alpha', 0.8)
        self.declare_parameter('ab_beta',  0.6)

        # Depth axis (camera z → body forward/backward)
        self.declare_parameter('ff_dist', 1.0)   # feedforward: match human depth-velocity
        self.declare_parameter('kp_dist', 0.5)   # P: close distance gap
        self.declare_parameter('kd_dist', 0.3)   # D: damp approach/recession

        # Yaw axis (camera x → yaw rate)
        self.declare_parameter('ff_yaw',  0.8)   # feedforward: pre-rotate with lateral motion
        self.declare_parameter('kp_yaw',  0.6)   # P: correct lateral offset
        self.declare_parameter('kd_yaw',  0.2)   # D: damp yaw oscillation

        # Lateral strafe axis (camera x → body right) — off by default
        self.declare_parameter('ff_lat',  0.0)
        self.declare_parameter('kp_lat',  0.0)
        self.declare_parameter('kd_lat',  0.0)

        # Altitude axis (camera y → body up/down)
        self.declare_parameter('ff_alt',  0.8)   # feedforward: match vertical motion
        self.declare_parameter('kp_alt',  0.5)   # P: correct height offset
        self.declare_parameter('kd_alt',  0.2)   # D: damp altitude oscillation

        self.declare_parameter('lat_deadband', 0.08)   # metres

        # Read all parameters
        self._takeoff_alt = self.get_parameter('takeoff_altitude').value
        self._follow_dist = self.get_parameter('follow_distance').value
        self._height_off  = self.get_parameter('height_offset').value
        self._max_xy      = self.get_parameter('max_xy_speed').value
        self._max_z       = self.get_parameter('max_z_speed').value
        self._max_yaw     = self.get_parameter('max_yaw_rate').value
        ab_alpha          = self.get_parameter('ab_alpha').value
        ab_beta           = self.get_parameter('ab_beta').value

        self._ff_dist = self.get_parameter('ff_dist').value
        self._kp_dist = self.get_parameter('kp_dist').value
        self._kd_dist = self.get_parameter('kd_dist').value

        self._ff_yaw  = self.get_parameter('ff_yaw').value
        self._kp_yaw  = self.get_parameter('kp_yaw').value
        self._kd_yaw  = self.get_parameter('kd_yaw').value

        self._ff_lat  = self.get_parameter('ff_lat').value
        self._kp_lat  = self.get_parameter('kp_lat').value
        self._kd_lat  = self.get_parameter('kd_lat').value

        self._ff_alt  = self.get_parameter('ff_alt').value
        self._kp_alt  = self.get_parameter('kp_alt').value
        self._kd_alt  = self.get_parameter('kd_alt').value

        self._lat_db  = self.get_parameter('lat_deadband').value

        if ab_beta >= 2.0 * ab_alpha:
            ab_beta = 0.99 * 2.0 * ab_alpha
            self.get_logger().warn(
                f'ab_beta clamped to {ab_beta:.3f} (stability: beta < 2*alpha)')

        # [0]=x/lateral, [1]=y/altitude, [2]=z/depth
        self._ab = [_AlphaBeta(ab_alpha, ab_beta) for _ in range(3)]

        # ── Publishers ────────────────────────────────────────────────────────
        self._ocm_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', _PX4_PUB_QOS)
        self._sp_pub  = self.create_publisher(
            TrajectorySetpoint,  '/fmu/in/trajectory_setpoint',   _PX4_PUB_QOS)
        self._cmd_pub = self.create_publisher(
            VehicleCommand,      '/fmu/in/vehicle_command',        _PX4_PUB_QOS)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._local_pos_cb, _PX4_SUB_QOS)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4',
            self._vehicle_status_cb, _PX4_SUB_QOS)
        self.create_subscription(
            PointStamped, '/tracker/centroid_3d',
            self._centroid_cb, 10)

        # ── State ─────────────────────────────────────────────────────────────
        self._local_pos      = VehicleLocalPosition()
        self._vehicle_status = VehicleStatus()
        self._centroid_seen  = False
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
            f'PDFollowController ready.  '
            f'α={ab_alpha}  β={ab_beta}  '
            f'dist(ff={self._ff_dist} kp={self._kp_dist} kd={self._kd_dist})  '
            f'yaw(ff={self._ff_yaw} kp={self._kp_yaw} kd={self._kd_yaw})  '
            f'alt(ff={self._ff_alt} kp={self._kp_alt} kd={self._kd_alt})')

    # ──────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────────────

    def _local_pos_cb(self, msg):
        self._local_pos = msg

    def _vehicle_status_cb(self, msg):
        self._vehicle_status = msg

    def _centroid_cb(self, msg: PointStamped):
        t  = time.monotonic()
        pt = msg.point
        self._ab[0].update(pt.x, t)
        self._ab[1].update(pt.y, t)
        self._ab[2].update(pt.z, t)
        self._centroid_seen  = True
        self._last_detection = t

    # ──────────────────────────────────────────────────────────────────────────
    # Control loop (50 Hz)
    # ──────────────────────────────────────────────────────────────────────────

    def _control_loop(self):
        now      = time.monotonic()
        is_armed = self._vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        in_ob    = self._vehicle_status.nav_state    == VehicleStatus.NAVIGATION_STATE_OFFBOARD

        arm_state = self._vehicle_status.arming_state
        if arm_state != self._prev_arm_state:
            name = _ARMING_STATE_NAMES.get(arm_state, str(arm_state))
            self.get_logger().info(f'PX4 arming_state → {name} ({arm_state})')
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
                        f'Armed + OFFBOARD. Taking off to {self._takeoff_alt:.1f} m AGL '
                        f'(NED z = {self._takeoff_z_ned:.2f} m).')
                return
            if now - self._last_arm_t > 1.0:
                self._last_arm_t = now
                self.get_logger().info(
                    f'Arm attempt — PX4: {_ARMING_STATE_NAMES.get(arm_state, arm_state)}, '
                    f'armed={is_armed}, offboard={in_ob}')
                self._send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                self._send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            return

        # ── Lost arm or OFFBOARD mid-flight ───────────────────────────────────
        if not (is_armed and in_ob):
            self.get_logger().warn(
                f'Lost arm/OFFBOARD (armed={is_armed}, offboard={in_ob}) — back to ARMING.')
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
                      now - self._last_detection < self._TARGET_TIMEOUT)

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

    def _enter_hold(self, reason=''):
        self._hold_x   = self._local_pos.x
        self._hold_y   = self._local_pos.y
        self._hold_z   = self._local_pos.z
        self._hold_yaw = self._local_pos.heading
        self._state    = _State.HOLD
        self.get_logger().info(
            f'HOLD ({reason}) — NED '
            f'({self._hold_x:.2f}, {self._hold_y:.2f}, {self._hold_z:.2f}), '
            f'yaw={math.degrees(self._hold_yaw):.1f}°')

    # ──────────────────────────────────────────────────────────────────────────
    # PD follow controller
    # ──────────────────────────────────────────────────────────────────────────

    def _follow_target(self, now: float):
        heading = self._local_pos.heading

        # Get filtered position + velocity estimates for all 3 axes.
        # pos[i]: smoothed centroid position in camera frame (metres)
        # vel[i]: estimated relative velocity of human w.r.t. drone (m/s)
        pos = [self._ab[i].predict(now)[0] for i in range(3)]
        vel = [self._ab[i].predict(now)[1] for i in range(3)]

        # ── Depth axis ────────────────────────────────────────────────────────
        # pos[2] = centroid z = depth (how far in front the human is)
        # vel[2] = d(depth)/dt (positive = human moving away, negative = approaching)
        # pos_error > 0 → human farther than desired → move forward
        # D term uses vel[2] because d(pos_error)/dt = d(centroid_z)/dt = vel[2]
        dist_err  = pos[2] - self._follow_dist
        # Use stronger gains when human is too close (dist_err < 0)
        kp = self._kp_dist * 2.0 if dist_err < 0 else self._kp_dist
        kd = self._kd_dist * 2.0 if dist_err < 0 else self._kd_dist
        body_fwd  = _clamp(
            self._ff_dist * vel[2]          # feedforward: match human's recession/approach speed
            + kp * dist_err      # P:           close the distance gap
            + kd * vel[2],       # D:           damp overshoot
            -self._max_xy, self._max_xy)

        # ── Lateral / yaw axis ────────────────────────────────────────────────
        # pos[0] = centroid x = lateral offset (positive = human is to the right)
        # vel[0] = d(lateral)/dt (positive = human moving right relative to drone)
        # Deadband applied to P and strafe only — FF and D remain active
        # to preserve responsiveness during actual lateral motion.
        lat_p      = pos[0] if abs(pos[0]) >= self._lat_db else 0.0

        body_right = _clamp(
            self._ff_lat * vel[0]
            + self._kp_lat * lat_p
            + self._kd_lat * vel[0],
            -self._max_xy, self._max_xy)

        yaw_rate   = _clamp(
            self._ff_yaw * vel[0]           # feedforward: pre-rotate with lateral motion
            + self._kp_yaw * lat_p          # P:           centre human in frame
            + self._kd_yaw * vel[0],        # D:           damp yaw oscillation
            -self._max_yaw, self._max_yaw)

        # ── Altitude axis ─────────────────────────────────────────────────────
        # pos[1] = centroid y = camera-down offset (positive = human below centre)
        # vel[1] = d(y)/dt (positive = human moving downward in camera frame)
        # alt_err > 0 → human appears lower than desired → drone is too high → descend
        # All terms are negated: increasing camera-y means we should descend (body_up < 0)
        alt_err  = pos[1] - self._height_off
        body_up  = _clamp(
            -(self._ff_alt * vel[1]         # feedforward: match vertical motion
              + self._kp_alt * alt_err      # P:           correct height error
              + self._kd_alt * vel[1]),     # D:           damp vertical oscillation
            -self._max_z, self._max_z)

        # ── Body → NED ────────────────────────────────────────────────────────
        ch, sh = math.cos(heading), math.sin(heading)
        self._publish_velocity(
            vx       = body_fwd * ch - body_right * sh,
            vy       = body_fwd * sh + body_right * ch,
            vz       = -body_up,    # NED z is positive-down
            yaw_rate = yaw_rate,
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Publish helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _publish_ocm(self, position=False, velocity=False):
        msg = OffboardControlMode()
        msg.position = position;  msg.velocity = velocity
        msg.acceleration = msg.attitude = msg.body_rate = False
        msg.timestamp = self._now_us()
        self._ocm_pub.publish(msg)

    def _publish_velocity(self, vx, vy, vz, yaw_rate=0.0):
        nan = float('nan')
        msg = TrajectorySetpoint()
        msg.position = msg.acceleration = msg.jerk = [nan, nan, nan]
        msg.velocity  = [float(vx), float(vy), float(vz)]
        msg.yaw       = nan
        msg.yawspeed  = float(yaw_rate)
        msg.timestamp = self._now_us()
        self._sp_pub.publish(msg)

    def _publish_position(self, x, y, z, yaw=float('nan')):
        nan = float('nan')
        msg = TrajectorySetpoint()
        msg.position     = [float(x), float(y), float(z)]
        msg.velocity = msg.acceleration = msg.jerk = [nan, nan, nan]
        msg.yaw      = float(yaw) if not math.isnan(yaw) else nan
        msg.yawspeed = nan
        msg.timestamp = self._now_us()
        self._sp_pub.publish(msg)

    def _send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1  = float(param1);  msg.param2 = float(param2)
        msg.target_system = msg.source_system = 1
        msg.target_component = msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._now_us()
        self._cmd_pub.publish(msg)

    def send_land_command(self):
        self.get_logger().warn('Sending LAND command.')
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def _now_us(self):
        return int(time.time() * 1e6)


# ──────────────────────────────────────────────────────────────────────────────

def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def main(args=None):
    rclpy.init(args=args)
    node = PDFollowController()
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
