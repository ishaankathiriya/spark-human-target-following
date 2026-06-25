#!/usr/bin/env python3
# Base flight manager node.
# Ground → OFFBOARD → ARM → TAKEOFF → HOVER → monitor velocity → AUTO.LAND
#
# Phases (no explicit enum, just sequential flags):
#   1. Wait for first pose  → record ground (gx, gy, gz)
#   2. Prime 2 s            → publish position setpoints at ground level
#   3. OFFBOARD switch      → retry every 5 s if not confirmed
#   4. Arm                  → retry on next loop if service busy
#   5. Takeoff              → position setpoint at gz + takeoff_alt
#   6. Hover                → publish /flight_ready, start active window timer
#   7. Active window        → if velocity setpoints arrive, stop publishing position setpoints
#                             if velocity stops   → AUTO.LAND
#                             if timer expires    → AUTO.LAND

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
_RETRY_S     = 5.0   # OFFBOARD retry interval if not confirmed
_VEL_TIMEOUT = 0.5   # seconds without velocity msg = velocity considered stopped


class FlightManager(Node):
    _LOOP_DT = 0.02  # 50 Hz

    def __init__(self):
        super().__init__('flight_manager')

        self.declare_parameter('takeoff_alt', 2.0)
        self.declare_parameter('active_s',    20.0)
        self._takeoff_alt = self.get_parameter('takeoff_alt').value
        self._active_s    = self.get_parameter('active_s').value

        latch_qos  = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._ready_pub = self.create_publisher(Bool,        '/flight_ready',                   latch_qos)
        self._pos_pub   = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        self.create_subscription(State,       '/mavros/state',               self._state_cb, 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._pose_cb,  sensor_qos)
        # Monitor velocity topic — fires when application node is actively publishing
        self.create_subscription(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped',
                                  self._vel_cb, sensor_qos)

        self._set_mode_cl = self.create_client(SetMode,     '/mavros/set_mode')
        self._arming_cl   = self.create_client(CommandBool, '/mavros/cmd/arming')

        # Ground reference — set on first pose message
        self._gx = self._gy = self._gz = 0.0
        self._ground_set  = False
        self._ground_t    = 0.0   # monotonic time when ground was recorded
        self._current_z   = 0.0   # updated every pose message

        self._fc_state    = State()
        self._mode_sent   = False
        self._mode_sent_t = 0.0
        self._arm_sent    = False
        self._hover_t     = None  # monotonic time when hover was reached (None until then)
        self._last_vel_t  = None  # monotonic time of last velocity setpoint received
        self._landed      = False

        self.create_timer(self._LOOP_DT, self._loop)
        self.get_logger().info(
            f'FlightManager ready — takeoff {self._takeoff_alt} m, active window {self._active_s} s')

    def _state_cb(self, msg: State):
        prev = self._fc_state
        self._fc_state = msg
        if msg.mode == 'OFFBOARD' and prev.mode != 'OFFBOARD':
            self.get_logger().info('OFFBOARD confirmed')
        if prev.mode == 'OFFBOARD' and msg.mode != 'OFFBOARD' and not self._landed:
            self._mode_sent = False
            self.get_logger().warn(f'Lost OFFBOARD → {msg.mode}')
        # Reset arm flag if disarmed before hover so we retry
        if prev.armed and not msg.armed and self._hover_t is None and not self._landed:
            self._arm_sent = False
            self.get_logger().warn('Disarmed — will retry')

    def _pose_cb(self, msg: PoseStamped):
        p = msg.pose.position
        self._current_z = p.z
        if not self._ground_set:
            self._gx, self._gy, self._gz = p.x, p.y, p.z
            self._ground_set = True
            self._ground_t   = time.monotonic()
            self.get_logger().info(f'Ground recorded: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})')

    def _vel_cb(self, _msg):
        # Just timestamp — tells us the application node is actively publishing
        self._last_vel_t = time.monotonic()

    def _pub_pos(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
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
        in_ob = self._fc_state.mode == 'OFFBOARD'
        armed = self._fc_state.armed

        # ── Pre-hover: OFFBOARD → ARM → TAKEOFF ──────────────────────────────

        if self._hover_t is None:

            # Publish ground-level setpoint until armed, then switch to takeoff altitude
            target_z = self._gz + self._takeoff_alt if armed else self._gz
            self._pub_pos(self._gx, self._gy, target_z)

            # Prime: wait before sending OFFBOARD request
            if now - self._ground_t < _PRIME_S:
                return

            # Request OFFBOARD (retry every _RETRY_S if not confirmed)
            if not self._mode_sent:
                if self._set_mode_cl.service_is_ready():
                    req = SetMode.Request()
                    req.custom_mode = 'OFFBOARD'
                    self._set_mode_cl.call_async(req)
                    self._mode_sent   = True
                    self._mode_sent_t = now
                    self.get_logger().info('OFFBOARD requested')
                return

            if not in_ob:
                if now - self._mode_sent_t > _RETRY_S:
                    self._mode_sent = False
                return

            # Arm once OFFBOARD confirmed
            if not armed:
                if not self._arm_sent and self._arming_cl.service_is_ready():
                    req = CommandBool.Request()
                    req.value = True
                    self._arming_cl.call_async(req)
                    self._arm_sent = True
                    self.get_logger().info('Arm requested')
                return

            # Wait until at takeoff altitude
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

        # Active window expired → land
        if now - self._hover_t > self._active_s:
            self.get_logger().info('Active window expired — landing')
            self._land()
            return

        vel_active = (self._last_vel_t is not None and
                      now - self._last_vel_t < _VEL_TIMEOUT)

        if vel_active:
            pass  # application node is publishing velocity — don't interfere

        elif self._last_vel_t is not None:
            # Velocity was active but has stopped
            self.get_logger().info('Velocity setpoints stopped — landing')
            self._land()

        else:
            # No velocity received yet — keep hovering with position setpoints
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
