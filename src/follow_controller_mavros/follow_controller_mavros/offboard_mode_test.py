#!/usr/bin/env python3
"""
Minimal node: stream zero-vel setpoints for PRIME_S seconds, then switch to
OFFBOARD once. No arming. Use this to verify the OFFBOARD mode switch works
independently of arm/disarm state.
"""
import time
import signal
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State

_PRIME_S = 2.0        # seconds of zero setpoints before requesting OFFBOARD
_RETRY_S = 5.0        # retry if OFFBOARD not confirmed within this many seconds


class OffboardModeTest(Node):
    _LOOP_DT = 0.02   # 50 Hz

    def __init__(self):
        super().__init__('offboard_mode_test')
        self.create_subscription(State, '/mavros/state', self._state_cb, 10)
        self._vel_pub = self.create_publisher(Twist, '/mavros/mavros/cmd_vel_unstamped', 10)
        self._set_mode_cl = self.create_client(SetMode, '/mavros/set_mode')

        self._fc_state = State()
        self._start_t = time.monotonic()
        self._mode_sent = False
        self._mode_sent_t = 0.0

        self.create_timer(self._LOOP_DT, self._loop)
        self.get_logger().info(f'OffboardModeTest ready — priming {_PRIME_S}s then requesting OFFBOARD once')

    def _state_cb(self, msg: State):
        prev_mode = self._fc_state.mode
        self._fc_state = msg
        if msg.mode == 'OFFBOARD' and prev_mode != 'OFFBOARD':
            self.get_logger().info('OFFBOARD confirmed — staying here')
        elif prev_mode == 'OFFBOARD' and msg.mode != 'OFFBOARD':
            self._mode_sent = False
            self.get_logger().warn(f'Lost OFFBOARD → {msg.mode} — will retry')

    def _pub_zero(self):
        self._vel_pub.publish(Twist())

    def _loop(self):
        pass
        # now = time.monotonic()

        # # Always stream zeros — PX4 requires continuous setpoints
        # self._pub_zero()

        # # Priming phase
        # if (now - self._start_t) < _PRIME_S:
        #     return

        # in_ob = self._fc_state.mode == 'OFFBOARD'

        # # Request OFFBOARD once
        # if not self._mode_sent:
        #     if self._set_mode_cl.service_is_ready():
        #         req = SetMode.Request()
        #         req.custom_mode = 'OFFBOARD'
        #         self._set_mode_cl.call_async(req)
        #         self._mode_sent = True
        #         self._mode_sent_t = now
        #         self.get_logger().info('OFFBOARD requested')
        #     return

        # # Retry if not confirmed
        # if not in_ob and (now - self._mode_sent_t) > _RETRY_S:
        #     self._mode_sent = False
        #     self.get_logger().warn('OFFBOARD not confirmed — retrying')


def main(args=None):
    rclpy.init(args=args)
    node = OffboardModeTest()
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(0)))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.get_logger().warn('Shutting down.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
