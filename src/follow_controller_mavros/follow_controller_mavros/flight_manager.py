#!/usr/bin/env python3
"""
Brings the drone from disarmed to OFFBOARD + armed + hovering at target_alt,
then signals the follow controller to take over via /flight_ready (latched Bool).

Sequence:
  prime (2 s zero setpoints)
  → request OFFBOARD
  → arm
  → climb at _CLIMB_VZ until altitude >= target_alt
  → publish /flight_ready True
  → stop publishing (follow controller takes over)
"""
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State

_PRIME_S  = 2.0   # seconds of zero setpoints before requesting OFFBOARD
_CLIMB_VZ = 0.3   # m/s upward during takeoff phase
_RETRY_S  = 5.0   # OFFBOARD retry interval if not confirmed


class FlightManager(Node):
    _LOOP_DT = 0.02  # 50 Hz

    def __init__(self):
        super().__init__('flight_manager')

        self.declare_parameter('target_alt', 0.5)
        self._target_alt = self.get_parameter('target_alt').value

        latch_qos  = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._ready_pub = self.create_publisher(Bool,  '/flight_ready',                             latch_qos)
        self._vel_pub   = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        self.create_subscription(State,       '/mavros/state',              self._state_cb, 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._pose_cb, sensor_qos)

        self._set_mode_cl = self.create_client(SetMode,     '/mavros/set_mode')
        self._arming_cl   = self.create_client(CommandBool, '/mavros/cmd/arming')

        self._fc_state    = State()
        self._alt         = 0.0
        self._start_t     = time.monotonic()
        self._mode_sent   = False
        self._mode_sent_t = 0.0
        self._arm_sent    = False
        self._done        = False

        self.create_timer(self._LOOP_DT, self._loop)
        self.get_logger().info(f'FlightManager ready — target altitude {self._target_alt} m')

    def _state_cb(self, msg: State):
        prev = self._fc_state
        self._fc_state = msg
        if msg.mode == 'OFFBOARD' and prev.mode != 'OFFBOARD':
            self.get_logger().info('OFFBOARD confirmed')
        if prev.mode == 'OFFBOARD' and msg.mode != 'OFFBOARD' and not self._done:
            self._mode_sent = False
            self.get_logger().warn(f'Lost OFFBOARD → {msg.mode} — retrying')
        if prev.armed and not msg.armed and not self._done:
            self._arm_sent = False
            self.get_logger().warn('Disarmed unexpectedly — will retry')

    def _pose_cb(self, msg: PoseStamped):
        self._alt = msg.pose.position.z  # ENU z = altitude above takeoff point

    def _loop(self):
        if self._done:
            return

        now   = time.monotonic()
        in_ob = self._fc_state.mode == 'OFFBOARD'
        armed = self._fc_state.armed

        # Phase 1: prime — stream zeros so PX4 will accept the OFFBOARD switch
        if (now - self._start_t) < _PRIME_S:
            self._pub_vel(0.0, 0.0, 0.0)
            return

        # Phase 2: request OFFBOARD once (retry every _RETRY_S if not confirmed)
        if not self._mode_sent:
            self._pub_vel(0.0, 0.0, 0.0)
            if self._set_mode_cl.service_is_ready():
                req = SetMode.Request()
                req.custom_mode = 'OFFBOARD'
                self._set_mode_cl.call_async(req)
                self._mode_sent   = True
                self._mode_sent_t = now
                self.get_logger().info('OFFBOARD requested')
            return

        if not in_ob:
            self._pub_vel(0.0, 0.0, 0.0)
            if (now - self._mode_sent_t) > _RETRY_S:
                self._mode_sent = False
            return

        # Phase 3: arm once OFFBOARD is confirmed
        if not armed:
            self._pub_vel(0.0, 0.0, 0.0)
            if not self._arm_sent and self._arming_cl.service_is_ready():
                req = CommandBool.Request()
                req.value = True
                self._arming_cl.call_async(req)
                self._arm_sent = True
                self.get_logger().info('Arm requested')
            return

        # Phase 4: climb to target altitude
        if self._alt < self._target_alt:
            self._pub_vel(0.0, 0.0, _CLIMB_VZ)
            return

        # Phase 5: done — hand off to follow controller
        self.get_logger().info(
            f'Altitude {self._alt:.2f} m >= {self._target_alt} m '
            'reached — publishing /flight_ready'
        )
        msg = Bool()
        msg.data = True
        self._ready_pub.publish(msg)
        self._done = True

    def _pub_vel(self, vx: float, vy: float, vz: float):
        t = Twist()
        t.linear.x = vx
        t.linear.y = vy
        t.linear.z = vz
        self._vel_pub.publish(t)


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
