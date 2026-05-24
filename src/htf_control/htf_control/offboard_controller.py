#!/usr/bin/env python3
"""
Offboard PD velocity controller for the HTF quadrotor.

State machine:
  PRIMING → ARMING → TAKEOFF → HOLD ↔ FOLLOW

  PRIMING : stream zero-velocity setpoints for ~0.4 s so PX4 accepts OFFBOARD
  ARMING  : retry arm + OFFBOARD every 1 s until both confirmed
  TAKEOFF : position-mode climb to target altitude; auto-transitions when close
  HOLD    : velocity-mode hover (zero vel) when no target visible
  FOLLOW  : PD velocity-mode tracking when /tracker/centroid_3d is fresh

Control architecture (FOLLOW state):
  - EMA filter on raw centroid (alpha=ema_alpha) removes single-frame jitter
  - PD on forward distance: P reacts to gap error, D brakes when gap is closing
    so the drone starts backing up before the human reaches follow_distance
  - P-only on lateral and altitude (yaw handles lateral, so kp_lat=0 by default)
  - Lateral deadband: ignore small horizontal offsets to prevent yaw-hunting

Topics consumed:
  /fmu/out/vehicle_local_position_v1  (px4_msgs/VehicleLocalPosition)
  /fmu/out/vehicle_status_v4          (px4_msgs/VehicleStatus)
  /tracker/centroid_3d                (geometry_msgs/PointStamped)
      point in camera optical frame: x=right, y=down, z=forward (depth)

Topics produced:
  /fmu/in/offboard_control_mode       (px4_msgs/OffboardControlMode)
  /fmu/in/trajectory_setpoint         (px4_msgs/TrajectorySetpoint)
  /fmu/in/vehicle_command             (px4_msgs/VehicleCommand)
"""

import math
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


class OffboardController(Node):
    _PREARM_CYCLES  = 20     # 50 Hz cycles before first arm attempt (~0.4 s)
    _TARGET_TIMEOUT = 1.0    # seconds without detection → FOLLOW → HOLD
    _TAKEOFF_TOL    = 0.3    # metres: altitude error threshold to exit TAKEOFF
    _LOOP_DT        = 0.02   # seconds: control loop period (50 Hz)

    def __init__(self):
        super().__init__('offboard_controller')

        # --- parameters -------------------------------------------------------
        self.declare_parameter('takeoff_altitude', 2.0)   # metres above ground
        self.declare_parameter('follow_distance',  1.5)   # metres behind centroid
        self.declare_parameter('height_offset',    0.5)   # metres above centroid
        self.declare_parameter('max_xy_speed',     1.5)   # m/s
        self.declare_parameter('max_z_speed',      0.8)   # m/s
        self.declare_parameter('max_yaw_rate',     0.8)   # rad/s
        # PD gains for forward distance
        self.declare_parameter('kp_dist',          0.6)
        self.declare_parameter('kd_dist',          0.4)   # derivative: brakes when gap closes
        # P gains for lateral (body-right) and altitude
        self.declare_parameter('kp_lat',           0.0)   # 0 = yaw-only lateral
        self.declare_parameter('kp_alt',           0.5)
        # Yaw P gain
        self.declare_parameter('kp_yaw',           0.8)
        # EMA smoothing factor for centroid (0=max smooth/lag, 1=raw/no filter)
        self.declare_parameter('ema_alpha',        0.4)
        # Lateral deadband: ignore |lat_err| below this to suppress yaw-hunting
        self.declare_parameter('lat_deadband',     0.08)  # metres

        self._takeoff_alt  = self.get_parameter('takeoff_altitude').value
        self._follow_dist  = self.get_parameter('follow_distance').value
        self._height_off   = self.get_parameter('height_offset').value
        self._max_xy       = self.get_parameter('max_xy_speed').value
        self._max_z        = self.get_parameter('max_z_speed').value
        self._max_yaw      = self.get_parameter('max_yaw_rate').value
        self._kp_d         = self.get_parameter('kp_dist').value
        self._kd_d         = self.get_parameter('kd_dist').value
        self._kp_l         = self.get_parameter('kp_lat').value
        self._kp_a         = self.get_parameter('kp_alt').value
        self._kp_y         = self.get_parameter('kp_yaw').value
        self._ema_alpha    = self.get_parameter('ema_alpha').value
        self._lat_db       = self.get_parameter('lat_deadband').value

        # --- publishers -------------------------------------------------------
        self._ocm_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', _PX4_PUB_QOS)
        self._sp_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', _PX4_PUB_QOS)
        self._cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', _PX4_PUB_QOS)

        # --- subscribers ------------------------------------------------------
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

        # --- state ------------------------------------------------------------
        self._local_pos      = VehicleLocalPosition()
        self._vehicle_status = VehicleStatus()
        self._centroid       = None
        self._last_detection = 0.0
        self._cycle          = 0
        self._last_arm_t     = 0.0
        self._ground_z       = None   # NED z latched on first PRIMING cycle
        self._takeoff_z_ned  = None
        self._state          = _State.PRIMING

        # EMA-filtered centroid [x, y, z] in camera optical frame (metres)
        self._filt           = None
        # Previous distance error for PD derivative term
        self._prev_dist_err  = 0.0

        self.create_timer(self._LOOP_DT, self._control_loop)
        self.get_logger().info('OffboardController ready.')

    # --------------------------------------------------------------------------
    # Callbacks
    # --------------------------------------------------------------------------

    def _local_pos_cb(self, msg: VehicleLocalPosition):
        self._local_pos = msg

    def _vehicle_status_cb(self, msg: VehicleStatus):
        self._vehicle_status = msg

    def _centroid_cb(self, msg: PointStamped):
        pt = msg.point
        if self._filt is None:
            self._filt = [pt.x, pt.y, pt.z]
            self._prev_dist_err = pt.z - self._follow_dist
        else:
            a = self._ema_alpha
            self._filt[0] = a * pt.x + (1.0 - a) * self._filt[0]
            self._filt[1] = a * pt.y + (1.0 - a) * self._filt[1]
            self._filt[2] = a * pt.z + (1.0 - a) * self._filt[2]
        self._centroid       = msg
        self._last_detection = time.monotonic()

    # --------------------------------------------------------------------------
    # Control loop (50 Hz)
    # --------------------------------------------------------------------------

    def _control_loop(self):
        now      = time.monotonic()
        is_armed = self._vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        in_ob    = self._vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

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
                    self._state = _State.HOLD
                    self.get_logger().info(
                        f'Armed + OFFBOARD confirmed. Already at {current_agl:.1f} m AGL '
                        f'(target {self._takeoff_alt:.1f} m) — entering HOLD.')
                else:
                    self._state = _State.TAKEOFF
                    self.get_logger().info(
                        f'Armed + OFFBOARD confirmed. '
                        f'Taking off to {self._takeoff_alt:.1f} m AGL '
                        f'(NED z target = {self._takeoff_z_ned:.2f} m).')
                return

            if now - self._last_arm_t > 1.0:
                self._last_arm_t = now
                self._send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                self._send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
                self.get_logger().info(
                    f'Sending arm + OFFBOARD (armed={is_armed}, offboard={in_ob})')
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
                self._local_pos.x,
                self._local_pos.y,
                self._takeoff_z_ned,
                yaw=self._local_pos.heading,
            )
            if abs(self._local_pos.z - self._takeoff_z_ned) < self._TAKEOFF_TOL:
                self._state = _State.HOLD
                self.get_logger().info(
                    f'Takeoff complete (z = {self._local_pos.z:.2f} m NED). '
                    f'Entering HOLD.')
            return

        # ── HOLD / FOLLOW ─────────────────────────────────────────────────────
        has_target = (self._centroid is not None and
                      (now - self._last_detection) < self._TARGET_TIMEOUT)

        if has_target:
            if self._state != _State.FOLLOW:
                self._state = _State.FOLLOW
                # Reset derivative accumulator on fresh FOLLOW entry to avoid spike.
                self._prev_dist_err = self._filt[2] - self._follow_dist
                self.get_logger().info('Target acquired — entering FOLLOW.')
            self._publish_ocm(velocity=True)
            self._follow_target()
        else:
            if self._state != _State.HOLD:
                self._state = _State.HOLD
                self.get_logger().info('Target lost — entering HOLD.')
            self._publish_ocm(velocity=True)
            self._publish_velocity(0.0, 0.0, 0.0, 0.0)

    # --------------------------------------------------------------------------
    # Following  (PD on distance, P on altitude, P+deadband on yaw)
    # --------------------------------------------------------------------------

    def _follow_target(self):
        fc      = self._filt           # EMA-filtered [x, y, z] in camera frame
        heading = self._local_pos.heading

        # --- distance: PD -------------------------------------------------------
        dist_err      = fc[2] - self._follow_dist
        dist_err_rate = (dist_err - self._prev_dist_err) / self._LOOP_DT
        self._prev_dist_err = dist_err
        body_fwd = _clamp(
            self._kp_d * dist_err + self._kd_d * dist_err_rate,
            -self._max_xy, self._max_xy)

        # --- lateral: deadband then P (body-right) + P (yaw) -------------------
        lat_err = fc[0]
        if abs(lat_err) < self._lat_db:
            lat_err = 0.0
        body_right = _clamp(self._kp_l * lat_err, -self._max_xy, self._max_xy)
        yaw_rate   = _clamp(self._kp_y * lat_err, -self._max_yaw, self._max_yaw)

        # --- altitude: P --------------------------------------------------------
        # Equilibrium: centroid appears height_off metres below camera centre (fc[1] == height_off).
        # fc[1] > 0 means centroid is below camera; fc[1] - height_off > 0 means drone is too high.
        alt_err  = fc[1] - self._height_off
        body_up  = _clamp(-self._kp_a * alt_err, -self._max_z, self._max_z)

        # --- body → NED rotation ------------------------------------------------
        ch, sh = math.cos(heading), math.sin(heading)
        self._publish_velocity(
            body_fwd * ch - body_right * sh,
            body_fwd * sh + body_right * ch,
            -body_up,   # NED z is positive-down
            yaw_rate,
        )

    # --------------------------------------------------------------------------
    # Publish helpers
    # --------------------------------------------------------------------------

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
        msg.position     = [float(x) if not math.isnan(x) else nan,
                            float(y) if not math.isnan(y) else nan,
                            float(z)]
        msg.velocity     = [nan, nan, nan]
        msg.acceleration = [nan, nan, nan]
        msg.jerk         = [nan, nan, nan]
        msg.yaw          = float(yaw)
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

    def _now_us(self) -> int:
        return int(time.time() * 1e6)


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def main(args=None):
    rclpy.init(args=args)
    node = OffboardController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
