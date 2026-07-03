#!/usr/bin/env python3
# Base flight manager node.
# Ground → OFFBOARD → ARM → TAKEOFF → HOVER → monitor velocity → AUTO.LAND
#
# Phases:
#   1. Wait for first pose  → record ground (gx, gy, gz), start prime timer
#   2. Prime 2 s            → publish position setpoints at ground level
#   3. OFFBOARD switch      → requested once via one-shot timer (no retry — RC can take over)
#   4. Arm                  → requested once in _state_cb when OFFBOARD first confirmed
#   5. Takeoff              → position setpoint at gz + takeoff_alt
#   6. Hover                → publish /flight_ready, start active window timer
#   7. Active window        → if velocity setpoints arrive, stop publishing position setpoints
#                             if velocity stops   → AUTO.LAND
#                             if timer expires    → AUTO.LAND

import math
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State

_PRIME_S     = 2.0   # seconds of position setpoints before requesting OFFBOARD
_ALT_THRESH  = 0.15  # metres tolerance to consider "at altitude"
_VEL_TIMEOUT = 3.0   # seconds without velocity msg = velocity considered stopped


class FlightManager(Node):
    _LOOP_DT = 0.02  # 50 Hz

    def __init__(self):
        super().__init__('flight_manager')

        self.declare_parameter('takeoff_alt',   1.5)
        self.declare_parameter('active_s',      120.0)
        self.declare_parameter('hover_yaw_deg', 90.0)   # ENU: 0=East, 90=North, 180=West
        self._takeoff_alt = self.get_parameter('takeoff_alt').value
        self._active_s    = self.get_parameter('active_s').value
        yaw               = math.radians(self.get_parameter('hover_yaw_deg').value)
        self._yaw_w       = math.cos(yaw / 2.0)
        self._yaw_z       = math.sin(yaw / 2.0)

        latch_qos  = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._ready_pub = self.create_publisher(Bool,        '/flight_ready',                   latch_qos)
        self._pos_pub   = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        self.create_subscription(State,       '/mavros/state',               self._state_cb, 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._pose_cb,  sensor_qos)
        self.create_subscription(Twist,       '/mavros/setpoint_velocity/cmd_vel_unstamped',
                                  self._vel_cb, sensor_qos)

        self._set_mode_cl = self.create_client(SetMode,     '/mavros/set_mode')
        self._arming_cl   = self.create_client(CommandBool, '/mavros/cmd/arming')

        self._gx = self._gy = self._gz = 0.0
        self._ground_set = False
        self._current_z  = 0.0

        self._fc_state   = State()
        self._arm_sent   = False
        self._hover_t    = None  # set when hover altitude reached
        self._last_vel_t = None  # set when first velocity setpoint received
        self._landed     = False

        self.create_timer(self._LOOP_DT, self._loop)
        self.get_logger().info(
            f'FlightManager ready — takeoff {self._takeoff_alt} m, '
            f'hover_yaw {math.degrees(yaw):.0f}° ENU, active window {self._active_s} s')

    def _state_cb(self, msg: State):
        prev = self._fc_state
        self._fc_state = msg
        # Arm once on first OFFBOARD confirmation — no retry, RC stays free
        if msg.mode == 'OFFBOARD' and prev.mode != 'OFFBOARD':
            self.get_logger().info('OFFBOARD confirmed')
            if not self._arm_sent and self._arming_cl.service_is_ready():
                req = CommandBool.Request()
                req.value = True
                self._arming_cl.call_async(req)
                self._arm_sent = True
                self.get_logger().info('Arm requested')
        if prev.mode == 'OFFBOARD' and msg.mode != 'OFFBOARD' and not self._landed:
            self.get_logger().warn(f'Lost OFFBOARD → {msg.mode}')

    def _pose_cb(self, msg: PoseStamped):
        p = msg.pose.position
        self._current_z = p.z
        if not self._ground_set:
            self._gx, self._gy, self._gz = p.x, p.y, p.z
            self._ground_set = True
            self.get_logger().info(f'Ground recorded: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})')
            # One-shot timer: request OFFBOARD once after prime period
            self._prime_timer = self.create_timer(_PRIME_S, self._request_offboard)

    def _vel_cb(self, _msg):
        self._last_vel_t = time.monotonic()

    def _request_offboard(self):
        self._prime_timer.cancel()  # fire once only
        if self._set_mode_cl.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = 'OFFBOARD'
            self._set_mode_cl.call_async(req)
            self.get_logger().info('OFFBOARD requested')
        else:
            self.get_logger().warn('set_mode service not ready — switch to OFFBOARD manually via RC')

    def _pub_pos(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = self._yaw_w
        msg.pose.orientation.z = self._yaw_z
        self._pos_pub.publish(msg)

    def _land(self):
        self._landed = True
        req = SetMode.Request()
        req.custom_mode = 'AUTO.LAND'
        self._set_mode_cl.call_async(req)
        self.get_logger().info('AUTO.LAND requested')

    def _loop(self):
        if self._landed or not self._ground_set:
            return

        now   = time.monotonic()
        armed = self._fc_state.armed

        # ── Pre-hover: publish setpoints, wait for arm + altitude ─────────────

        if self._hover_t is None:
            target_z = self._gz + self._takeoff_alt if armed else self._gz
            self._pub_pos(self._gx, self._gy, target_z)

            if not armed:
                return
            if self._current_z < self._gz + self._takeoff_alt - _ALT_THRESH:
                return

            # Hover reached — signal application node and start timer
            self._hover_t = now
            self._ready_pub.publish(Bool(data=True))
            self.get_logger().info(
                f'Hover at {self._current_z:.2f} m — /flight_ready published, '
                f'{self._active_s} s active window started')
            return

        # ── Hover / Active window ─────────────────────────────────────────────

        if now - self._hover_t > self._active_s:
            self.get_logger().info('Active window expired — landing')
            self._land()
            return

        vel_active = (self._last_vel_t is not None and
                      now - self._last_vel_t < _VEL_TIMEOUT)

        if vel_active:
            pass  # application node is publishing velocity — don't interfere
        elif self._last_vel_t is not None:
            self.get_logger().info('Velocity setpoints stopped — landing')
            self._land()
        else:
            self._pub_pos(self._gx, self._gy, self._gz + self._takeoff_alt)


def main(args=None):
    rclpy.init(args=args)
    node = FlightManager()
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(0)))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.get_logger().warn('FlightManager shutting down')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
