#!/usr/bin/env python3
"""
Offboard velocity controller for the HTF quadrotor.

State machine:
  PRIMING → ARMING → TAKEOFF → HOLD ↔ FOLLOW

  PRIMING : stream zero-velocity setpoints for ~0.4 s so PX4 accepts OFFBOARD
  ARMING  : retry arm + OFFBOARD every 1 s until both confirmed
  TAKEOFF : position-mode climb to target altitude; auto-transitions when close
  HOLD    : velocity-mode hover (zero vel) when no target visible
  FOLLOW  : velocity-mode tracking when /tracker/centroid_3d is fresh

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
    PRIMING  = 0   # streaming setpoints to prime PX4 before arming
    ARMING   = 1   # sending arm + OFFBOARD commands, retrying until confirmed
    TAKEOFF  = 2   # position-mode climb to target altitude
    HOLD     = 3   # velocity-mode hover (zero vel)
    FOLLOW   = 4   # velocity-mode tracking


# Publishing TO PX4 (/fmu/in/): TRANSIENT_LOCAL satisfies PX4's VOLATILE subscriber.
_PX4_PUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Subscribing FROM PX4 (/fmu/out/): VOLATILE matches the XRCE agent's publisher.
# Using TRANSIENT_LOCAL here would cause a QoS incompatibility and no messages.
_PX4_SUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class OffboardController(Node):
    _PREARM_CYCLES  = 20    # 50 Hz cycles before first arm attempt (~0.4 s)
    _TARGET_TIMEOUT = 1.0   # seconds without detection → FOLLOW → HOLD
    _TAKEOFF_TOL    = 0.3   # metres: altitude error threshold to exit TAKEOFF

    def __init__(self):
        super().__init__('offboard_controller')

        # --- parameters -------------------------------------------------------
        self.declare_parameter('takeoff_altitude', 2.0)  # metres above ground
        self.declare_parameter('follow_distance',  2.0)  # metres
        self.declare_parameter('max_xy_speed',     1.5)  # m/s
        self.declare_parameter('max_z_speed',      0.8)  # m/s
        self.declare_parameter('max_yaw_rate',     0.8)  # rad/s
        self.declare_parameter('kp_dist',          0.5)
        self.declare_parameter('kp_lat',           0.0)  # 0 = yaw-only lateral
        self.declare_parameter('kp_alt',           0.4)
        self.declare_parameter('kp_yaw',           0.6)

        self._takeoff_alt = self.get_parameter('takeoff_altitude').value
        self._follow_dist = self.get_parameter('follow_distance').value
        self._max_xy      = self.get_parameter('max_xy_speed').value
        self._max_z       = self.get_parameter('max_z_speed').value
        self._max_yaw     = self.get_parameter('max_yaw_rate').value
        self._kp_d        = self.get_parameter('kp_dist').value
        self._kp_l        = self.get_parameter('kp_lat').value
        self._kp_a        = self.get_parameter('kp_alt').value
        self._kp_y        = self.get_parameter('kp_yaw').value

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
        self._takeoff_z_ned  = None   # target NED z, set when ARMING completes
        self._state          = _State.PRIMING

        self.create_timer(0.02, self._control_loop)   # 50 Hz
        self.get_logger().info('OffboardController ready.')

    # --------------------------------------------------------------------------
    # Callbacks
    # --------------------------------------------------------------------------

    def _local_pos_cb(self, msg: VehicleLocalPosition):
        self._local_pos = msg

    def _vehicle_status_cb(self, msg: VehicleStatus):
        self._vehicle_status = msg

    def _centroid_cb(self, msg: PointStamped):
        self._centroid = msg
        self._last_detection = time.monotonic()

    # --------------------------------------------------------------------------
    # Control loop
    # --------------------------------------------------------------------------

    def _control_loop(self):
        now      = time.monotonic()
        is_armed = self._vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        in_ob    = self._vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

        # ── PRIMING ───────────────────────────────────────────────────────────
        if self._state == _State.PRIMING:
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
                # Record ground z so takeoff target is relative to spawn point.
                # NED z is positive-down, so target altitude is negative.
                self._takeoff_z_ned = self._local_pos.z - self._takeoff_alt
                self._state = _State.TAKEOFF
                self.get_logger().info(
                    f'Armed + OFFBOARD confirmed. '
                    f'Taking off to {self._takeoff_alt:.1f} m '
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
            self._state = _State.ARMING
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

            alt_err = abs(self._local_pos.z - self._takeoff_z_ned)
            if alt_err < self._TAKEOFF_TOL:
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
    # Following
    # --------------------------------------------------------------------------

    def _follow_target(self):
        pt      = self._centroid.point
        heading = self._local_pos.heading   # radians from North, clockwise

        # Errors in camera optical frame (x=right, y=down, z=depth/forward)
        dist_err = pt.z - self._follow_dist   # + = too far  → move forward
        lat_err  = pt.x                        # + = right    → yaw right
        alt_err  = pt.y                        # + = below    → descend

        body_fwd   = _clamp(self._kp_d * dist_err,  -self._max_xy, self._max_xy)
        body_right = _clamp(self._kp_l * lat_err,   -self._max_xy, self._max_xy)
        body_up    = _clamp(-self._kp_a * alt_err,  -self._max_z,  self._max_z)
        yaw_rate   = _clamp(self._kp_y * lat_err,   -self._max_yaw, self._max_yaw)

        # Body → NED rotation using current heading h:
        #   NED_x = fwd·cos(h) - right·sin(h)
        #   NED_y = fwd·sin(h) + right·cos(h)
        #   NED_z = -up  (NED z is positive-down)
        ch, sh = math.cos(heading), math.sin(heading)
        self._publish_velocity(
            body_fwd * ch - body_right * sh,
            body_fwd * sh + body_right * ch,
            -body_up,
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
        # PX4 expects Unix wall-clock time in microseconds, not simulation time.
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
