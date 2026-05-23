#!/usr/bin/env python3
"""
Offboard velocity controller for the HTF quadrotor.

State machine:
  IDLE  ─▶  ARM_OFFBOARD  ─▶  HOLD  ─▶  FOLLOW
  (publishes setpoints for N cycles before arming so PX4 accepts OFFBOARD)

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


# QoS matching PX4 uXRCE-DDS defaults
_PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class OffboardController(Node):
    # Number of setpoint cycles before issuing arm + mode-switch command.
    # At 50 Hz this is 0.4 s — enough for PX4 to accept OFFBOARD.
    _PREARM_CYCLES = 20

    # How long (seconds) without a detection before switching back to HOLD.
    _TARGET_TIMEOUT = 1.0

    def __init__(self):
        super().__init__('offboard_controller')

        # --- parameters -------------------------------------------------------
        self.declare_parameter('follow_distance', 2.0)   # metres
        self.declare_parameter('max_xy_speed', 1.5)      # m/s horizontal
        self.declare_parameter('max_z_speed', 0.8)       # m/s vertical
        self.declare_parameter('max_yaw_rate', 0.8)      # rad/s
        self.declare_parameter('kp_dist', 0.5)           # forward proportional gain
        self.declare_parameter('kp_lat', 0.0)            # lateral strafe gain (0 = yaw-only)
        self.declare_parameter('kp_alt', 0.4)            # altitude proportional gain
        self.declare_parameter('kp_yaw', 0.6)            # yaw proportional gain

        self._follow_dist = self.get_parameter('follow_distance').value
        self._max_xy = self.get_parameter('max_xy_speed').value
        self._max_z = self.get_parameter('max_z_speed').value
        self._max_yaw = self.get_parameter('max_yaw_rate').value
        self._kp_d = self.get_parameter('kp_dist').value
        self._kp_l = self.get_parameter('kp_lat').value
        self._kp_a = self.get_parameter('kp_alt').value
        self._kp_y = self.get_parameter('kp_yaw').value

        # --- publishers -------------------------------------------------------
        self._ocm_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', _PX4_QOS)
        self._sp_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', _PX4_QOS)
        self._cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', _PX4_QOS)

        # --- subscribers ------------------------------------------------------
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self._local_pos_cb,
            _PX4_QOS,
        )
        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v4',
            self._vehicle_status_cb,
            _PX4_QOS,
        )
        self.create_subscription(
            PointStamped,
            '/tracker/centroid_3d',
            self._centroid_cb,
            10,
        )

        # --- state ------------------------------------------------------------
        self._local_pos = VehicleLocalPosition()
        self._vehicle_status = VehicleStatus()
        self._centroid: PointStamped | None = None
        self._last_detection_time = 0.0
        self._cycle = 0

        # 50 Hz control loop
        self.create_timer(0.02, self._control_loop)
        self.get_logger().info('OffboardController ready — will arm and switch to OFFBOARD automatically.')

    # --------------------------------------------------------------------------
    # Callbacks
    # --------------------------------------------------------------------------

    def _local_pos_cb(self, msg: VehicleLocalPosition):
        self._local_pos = msg

    def _vehicle_status_cb(self, msg: VehicleStatus):
        self._vehicle_status = msg

    def _centroid_cb(self, msg: PointStamped):
        self._centroid = msg
        self._last_detection_time = self.get_clock().now().nanoseconds * 1e-9

    # --------------------------------------------------------------------------
    # Control loop
    # --------------------------------------------------------------------------

    def _control_loop(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        # 1. Always publish OffboardControlMode at 50 Hz (velocity control)
        self._publish_ocm(velocity=True)

        # 2. After PREARM_CYCLES, send arm + OFFBOARD once
        if self._cycle == self._PREARM_CYCLES:
            self._send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                param1=1.0, param2=6.0)   # 6 = OFFBOARD
            self._send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=1.0)
            self.get_logger().info('Arm + OFFBOARD commands sent.')
        if self._cycle <= self._PREARM_CYCLES:
            self._cycle += 1

        # 3. Check states
        is_armed = (
            self._vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        in_offboard = (
            self._vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

        if not (is_armed and in_offboard):
            # Publish zero velocity so PX4 keeps seeing valid setpoints
            self._publish_velocity(0.0, 0.0, 0.0, 0.0)
            return

        # 4. Determine if target is fresh
        target_age = now - self._last_detection_time
        has_target = (self._centroid is not None and
                      target_age < self._TARGET_TIMEOUT)

        if has_target:
            self._follow_target()
        else:
            # Hold: zero velocity; drone EKF and position controller keep it stable
            self._publish_velocity(0.0, 0.0, 0.0, 0.0)

    def _follow_target(self):
        pt = self._centroid.point
        heading = self._local_pos.heading   # radians from North, clockwise

        # --- errors in camera optical frame (x=right, y=down, z=fwd) ----------
        dist_err = pt.z - self._follow_dist   # positive → too far → move fwd
        lat_err = pt.x                         # positive → target right → yaw right
        alt_err = pt.y                         # positive → target below → descend

        # --- body-frame velocity commands -------------------------------------
        # Forward/backward to maintain follow distance
        body_fwd = self._kp_d * dist_err

        # Lateral strafe (usually 0; controlled via yaw)
        body_right = self._kp_l * lat_err

        # Altitude: target below → descend → negative up velocity → positive NED_z
        body_up = -self._kp_a * alt_err

        # Yaw rate: target right → yaw clockwise (positive)
        yaw_rate = self._kp_y * lat_err

        # --- clamp body velocities --------------------------------------------
        body_fwd = _clamp(body_fwd, -self._max_xy, self._max_xy)
        body_right = _clamp(body_right, -self._max_xy, self._max_xy)
        body_up = _clamp(body_up, -self._max_z, self._max_z)
        yaw_rate = _clamp(yaw_rate, -self._max_yaw, self._max_yaw)

        # --- body → NED frame -------------------------------------------------
        # body_fwd = +x_body, body_right = +y_body (PX4 FRD)
        # NED_x = fwd*cos(h) - right*sin(h)
        # NED_y = fwd*sin(h) + right*cos(h)
        # NED_z = -body_up  (NED z is down)
        ch, sh = math.cos(heading), math.sin(heading)
        ned_vx = body_fwd * ch - body_right * sh
        ned_vy = body_fwd * sh + body_right * ch
        ned_vz = -body_up

        self._publish_velocity(ned_vx, ned_vy, ned_vz, yaw_rate)

    # --------------------------------------------------------------------------
    # Publish helpers
    # --------------------------------------------------------------------------

    def _publish_ocm(self, velocity=True):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = velocity
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self._now_us()
        self._ocm_pub.publish(msg)

    def _publish_velocity(self, vx, vy, vz, yaw_rate):
        msg = TrajectorySetpoint()
        nan = float('nan')
        msg.position = [nan, nan, nan]
        msg.velocity = [float(vx), float(vy), float(vz)]
        msg.acceleration = [nan, nan, nan]
        msg.jerk = [nan, nan, nan]
        msg.yaw = nan
        msg.yawspeed = float(yaw_rate)
        msg.timestamp = self._now_us()
        self._sp_pub.publish(msg)

    def _send_vehicle_command(self, command: int, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._now_us()
        self._cmd_pub.publish(msg)

    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)


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
