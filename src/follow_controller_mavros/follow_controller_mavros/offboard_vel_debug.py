#!/usr/bin/env python3
"""
Stream zero setpoints → OFFBOARD → arm.
Keeps streaming zeros throughout (no velocity setpoints after arm).
"""
import time
import signal
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State

_PRIME_S = 2.0
_RETRY_S = 5.0


class OffboardVelDebug(Node):
    _LOOP_DT = 0.02

    def __init__(self):
        super().__init__('offboard_vel_debug')

        self.create_subscription(State, '/mavros/state', self._state_cb, 10)
        self._vel_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self._set_mode_cl = self.create_client(SetMode, '/mavros/set_mode')
        self._arming_cl = self.create_client(CommandBool, '/mavros/cmd/arming')

        self._fc_state = State()
        self._start_t = time.monotonic()
        self._mode_sent = False
        self._mode_sent_t = 0.0
        self._arm_sent = False

        self.create_timer(self._LOOP_DT, self._loop)
        self.get_logger().info('OffboardVelDebug ready — priming → OFFBOARD → arm')

    def _state_cb(self, msg: State):
        prev = self._fc_state
        self._fc_state = msg
        if msg.mode == 'OFFBOARD' and prev.mode != 'OFFBOARD':
            self.get_logger().info('OFFBOARD confirmed')
        if prev.mode == 'OFFBOARD' and msg.mode != 'OFFBOARD':
            self._mode_sent = False
            self.get_logger().warn(f'Lost OFFBOARD → {msg.mode} — will retry')
        if prev.armed and not msg.armed:
            self._arm_sent = False
            self.get_logger().warn('Disarmed — will retry arm')

    def _loop(self):
        now = time.monotonic()
        in_ob = self._fc_state.mode == 'OFFBOARD'
        armed = self._fc_state.armed

        self._vel_pub.publish(Twist())

        if (now - self._start_t) < _PRIME_S:
            return

        if not self._mode_sent:
            if self._set_mode_cl.service_is_ready():
                req = SetMode.Request()
                req.custom_mode = 'OFFBOARD'
                self._set_mode_cl.call_async(req)
                self._mode_sent = True
                self._mode_sent_t = now
                self.get_logger().info('OFFBOARD requested')
            return

        if not in_ob:
            if (now - self._mode_sent_t) > _RETRY_S:
                self._mode_sent = False
                self.get_logger().warn('OFFBOARD not confirmed — retrying')
            return

        if not armed and not self._arm_sent:
            if self._arming_cl.service_is_ready():
                req = CommandBool.Request()
                req.value = True
                self._arming_cl.call_async(req)
                self._arm_sent = True
                self.get_logger().info('Arm requested')

    def send_zero(self):
        self.get_logger().warn('Shutting down — zeroing velocity.')
        self._vel_pub.publish(Twist())
        if self._fc_state.armed and self._arming_cl.service_is_ready():
            req = CommandBool.Request()
            req.value = False
            self._arming_cl.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardVelDebug()
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(0)))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.send_zero()
        deadline = time.monotonic() + 0.8
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
