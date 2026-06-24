#!/usr/bin/env python3
"""
Waits for OFFBOARD mode confirmation, then arms once.
Run alongside offboard_mode_test — it handles the OFFBOARD switch,
this node handles the arm.
"""
import signal
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool
from mavros_msgs.msg import State


class ArmNode(Node):

    def __init__(self):
        super().__init__('arm_node')
        self.create_subscription(State, '/mavros/state', self._state_cb, 10)
        self._arming_cl = self.create_client(CommandBool, '/mavros/mavros/arming')
        self._fc_state = State()
        self._arm_sent = False
        self.get_logger().info('ArmNode ready — waiting for OFFBOARD confirmation before arming')

    def _state_cb(self, msg: State):
        prev = self._fc_state
        self._fc_state = msg

        if msg.mode == 'OFFBOARD' and prev.mode != 'OFFBOARD':
            self.get_logger().info('OFFBOARD confirmed — sending arm')
            self._try_arm()

        if prev.armed and not msg.armed:
            self._arm_sent = False
            self.get_logger().warn('Disarmed — will retry arm on next OFFBOARD confirmation')

    def _try_arm(self):
        if self._arm_sent:
            return
        if self._arming_cl.service_is_ready():
            req = CommandBool.Request()
            req.value = True
            self._arming_cl.call_async(req)
            self._arm_sent = True
            self.get_logger().info('Arm command sent')
        else:
            self.get_logger().warn('Arming service not ready — will retry on next OFFBOARD confirmation')
            self._arm_sent = False


def main(args=None):
    rclpy.init(args=args)
    node = ArmNode()
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(0)))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
