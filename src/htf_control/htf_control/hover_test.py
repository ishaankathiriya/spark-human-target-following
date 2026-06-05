#!/usr/bin/env python3
"""
HTF hover test node.

Arms, takes off to `takeoff_altitude`, and holds position permanently.
Does NOT follow the human — the drone stays stationary.

Purpose: verify the alpha-beta filter while the drone is not moving.
Since the drone is stationary, relative velocity = absolute human velocity,
making it easy to confirm filter accuracy with the debug_plotter.

The alpha-beta filter runs on every incoming centroid and publishes
/htf/human_vel and /htf/human_pos every control cycle (50 Hz), even
while hovering. Run the debug_plotter in a separate terminal alongside this.

State machine:
  PRIMING → ARMING → TAKEOFF → HOLD (permanent)

Topics consumed:
  /fmu/out/vehicle_local_position_v1   px4_msgs/VehicleLocalPosition
  /fmu/out/vehicle_status_v4           px4_msgs/VehicleStatus
  /tracker/centroid_3d                 geometry_msgs/PointStamped

Topics produced:
  /fmu/in/offboard_control_mode        px4_msgs/OffboardControlMode
  /fmu/in/trajectory_setpoint          px4_msgs/TrajectorySetpoint
  /fmu/in/vehicle_command              px4_msgs/VehicleCommand
  /htf/human_vel                       geometry_msgs/Vector3Stamped  (alpha-beta vel)
  /htf/human_pos                       geometry_msgs/PointStamped    (alpha-beta pos)
"""

import collections
import math
import signal
import time
from enum import IntEnum

import numpy as np

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
from geometry_msgs.msg import PointStamped, Vector3Stamped


# ──────────────────────────────────────────────────────────────────────────────
# Axis estimator: alpha-beta for position, linear regression for velocity
# ──────────────────────────────────────────────────────────────────────────────

class _AxisEstimator:
    """Alpha-beta position + sliding-window regression velocity (same interface as _AlphaBeta)."""

    _MIN_DT = 1e-3

    def __init__(self, alpha: float, beta: float, max_plausible_speed: float = 8.0):
        self._alpha = alpha
        self._beta  = beta
        self._max_v = max_plausible_speed
        self._pos   = 0.0
        self._vel   = 0.0
        self._t     = 0.0
        self._ready = False

    def update(self, meas: float, t: float) -> tuple[float, float]:
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

    def predict(self, t: float) -> tuple[float, float]:
        if not self._ready:
            return 0.0, 0.0
        dt = max(t - self._t, 0.0)
        return self._pos + self._vel * dt, self._vel

    @property
    def ready(self) -> bool:
        return self._ready


# ──────────────────────────────────────────────────────────────────────────────

class _State(IntEnum):
    PRIMING = 0
    ARMING  = 1
    TAKEOFF = 2
    HOLD    = 3


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

class HoverTestNode(Node):

    _PREARM_CYCLES = 20
    _TAKEOFF_TOL   = 0.3
    _LOOP_DT       = 0.02

    def __init__(self):
        super().__init__('hover_test')

        self.declare_parameter('takeoff_altitude', 1.5)  # metres AGL
        self.declare_parameter('ab_alpha',            0.5)
        self.declare_parameter('ab_beta',             0.08)
        self.declare_parameter('max_plausible_speed', 8.0)

        self._takeoff_alt = self.get_parameter('takeoff_altitude').value
        ab_alpha          = self.get_parameter('ab_alpha').value
        ab_beta           = self.get_parameter('ab_beta').value
        max_plausible_spd = self.get_parameter('max_plausible_speed').value

        if ab_beta >= 2.0 * ab_alpha:
            ab_beta = 0.99 * 2.0 * ab_alpha

        # [0]=lateral, [1]=vertical, [2]=depth
        self._ab = [_AxisEstimator(ab_alpha, ab_beta, max_plausible_spd) for _ in range(3)]

        # Runtime parameter tuning — allows `ros2 param set /hover_test ab_beta 0.25`
        # without restarting the node. Updates filter objects in place.
        self.add_on_set_parameters_callback(self._on_param_change)

        # ── Publishers ────────────────────────────────────────────────────────
        self._ocm_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', _PX4_PUB_QOS)
        self._sp_pub  = self.create_publisher(
            TrajectorySetpoint,  '/fmu/in/trajectory_setpoint',   _PX4_PUB_QOS)
        self._cmd_pub = self.create_publisher(
            VehicleCommand,      '/fmu/in/vehicle_command',        _PX4_PUB_QOS)
        self._vel_pub = self.create_publisher(
            Vector3Stamped, '/htf/human_vel', 10)
        self._pos_pub = self.create_publisher(
            PointStamped,   '/htf/human_pos', 10)

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
        self._ground_z       = None
        self._takeoff_z_ned  = None
        self._takeoff_x      = None
        self._takeoff_y      = None
        self._hold_x         = None
        self._hold_y         = None
        self._hold_z         = None
        self._hold_yaw       = None
        self._cycle          = 0
        self._last_arm_t     = 0.0
        self._prev_arm_state = -1
        self._state          = _State.PRIMING

        self.create_timer(self._LOOP_DT, self._control_loop)
        self.get_logger().info(
            f'HoverTest ready — will hold at {self._takeoff_alt:.1f} m AGL.  '
            f'Publishing /htf/human_vel and /htf/human_pos continuously.')

    # ── Runtime parameter callback ────────────────────────────────────────────

    def _on_param_change(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'ab_alpha':
                for f in self._ab:
                    f._alpha = p.value
                self.get_logger().info(f'ab_alpha → {p.value}')
            elif p.name == 'ab_beta':
                for f in self._ab:
                    f._beta = p.value
                self.get_logger().info(f'ab_beta → {p.value}')
            elif p.name == 'max_plausible_speed':
                for f in self._ab:
                    f._max_v = p.value
                self.get_logger().info(f'max_plausible_speed → {p.value}')
        return SetParametersResult(successful=True)

    # ── Callbacks ─────────────────────────────────────────────────────────────

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

    # ── Control loop ──────────────────────────────────────────────────────────

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
            self._publish_debug(now)
            return

        # ── ARMING ────────────────────────────────────────────────────────────
        if self._state == _State.ARMING:
            self._publish_ocm(velocity=True)
            self._publish_velocity(0.0, 0.0, 0.0, 0.0)
            if is_armed and in_ob:
                self._takeoff_z_ned = self._ground_z - self._takeoff_alt
                self._takeoff_x     = self._local_pos.x
                self._takeoff_y     = self._local_pos.y
                self._state         = _State.TAKEOFF
                self.get_logger().info(
                    f'Armed + OFFBOARD. Taking off to {self._takeoff_alt:.1f} m AGL '
                    f'(NED z = {self._takeoff_z_ned:.2f} m).')
            elif now - self._last_arm_t > 1.0:
                self._last_arm_t = now
                self.get_logger().info(
                    f'Arm attempt — armed={is_armed}, offboard={in_ob}')
                self._send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                self._send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self._publish_debug(now)
            return

        # ── Lost arm or OFFBOARD mid-flight ───────────────────────────────────
        if not (is_armed and in_ob):
            self.get_logger().warn('Lost arm/OFFBOARD — returning to ARMING.')
            self._state      = _State.ARMING
            self._last_arm_t = 0.0
            self._publish_ocm(velocity=True)
            self._publish_velocity(0.0, 0.0, 0.0, 0.0)
            self._publish_debug(now)
            return

        # ── TAKEOFF ───────────────────────────────────────────────────────────
        if self._state == _State.TAKEOFF:
            self._publish_ocm(position=True)
            self._publish_position(
                self._takeoff_x, self._takeoff_y, self._takeoff_z_ned,
                yaw=self._local_pos.heading)
            if abs(self._local_pos.z - self._takeoff_z_ned) < self._TAKEOFF_TOL:
                self._hold_x   = self._local_pos.x
                self._hold_y   = self._local_pos.y
                self._hold_z   = self._local_pos.z
                self._hold_yaw = self._local_pos.heading
                self._state    = _State.HOLD
                self.get_logger().info(
                    f'Reached {self._takeoff_alt:.1f} m. Holding. '
                    f'Walk in front of the camera to test the filter.')
            self._publish_debug(now)
            return

        # ── HOLD (permanent) ──────────────────────────────────────────────────
        self._publish_ocm(position=True)
        self._publish_position(
            self._hold_x, self._hold_y, self._hold_z, yaw=self._hold_yaw)
        self._publish_debug(now)

    # ── Debug publisher ───────────────────────────────────────────────────────

    def _publish_debug(self, now: float):
        """Publish alpha-beta estimates every control cycle regardless of state."""
        pos = [self._ab[i].predict(now)[0] for i in range(3)]
        vel = [self._ab[i].predict(now)[1] for i in range(3)]
        stamp = self.get_clock().now().to_msg()

        v = Vector3Stamped()
        v.header.stamp    = stamp
        v.header.frame_id = 'camera_link'
        v.vector.x = vel[0]; v.vector.y = vel[1]; v.vector.z = vel[2]
        self._vel_pub.publish(v)

        p = PointStamped()
        p.header.stamp    = stamp
        p.header.frame_id = 'camera_link'
        p.point.x = pos[0]; p.point.y = pos[1]; p.point.z = pos[2]
        self._pos_pub.publish(p)

    # ── Publish helpers ───────────────────────────────────────────────────────

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
        msg.velocity = [float(vx), float(vy), float(vz)]
        msg.yaw = nan;  msg.yawspeed = float(yaw_rate)
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

def main(args=None):
    rclpy.init(args=args)
    node = HoverTestNode()
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
