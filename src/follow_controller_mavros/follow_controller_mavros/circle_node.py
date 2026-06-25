#!/usr/bin/env python3
# Fly a horizontal circle at hover altitude.
# Waits for /flight_ready, then publishes ENU velocity setpoints.
# flight_manager detects the stream and stands aside; stopping this node → AUTO.LAND.

import math
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


class CircleNode(Node):
    _LOOP_DT = 0.02  # 50 Hz

    def __init__(self):
        super().__init__('circle_node')

        self.declare_parameter('radius',    3.0)   # metres
        self.declare_parameter('speed',     0.5)   # tangential m/s
        self.declare_parameter('clockwise', False)

        self._R  = self.get_parameter('radius').value
        spd      = self.get_parameter('speed').value
        cw       = self.get_parameter('clockwise').value

        # Negative omega = CW (from above); positive = CCW
        self._omega = (spd / self._R) * (-1.0 if cw else 1.0)

        latch_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(Bool, '/flight_ready', self._ready_cb, latch_qos)
        self._vel_pub = self.create_publisher(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        self._ready   = False
        self._start_t = None

        self.create_timer(self._LOOP_DT, self._loop)
        self.get_logger().info(
            f'CircleNode ready — R={self._R} m  speed={spd} m/s  '
            f'ω={self._omega:.3f} rad/s  {"CW" if cw else "CCW"} — waiting for /flight_ready')

    def _ready_cb(self, msg: Bool):
        if msg.data and not self._ready:
            self._ready   = True
            self._start_t = time.monotonic()
            self.get_logger().info('Flight ready — starting circle')

    def _loop(self):
        if not self._ready:
            return

        theta = self._omega * (time.monotonic() - self._start_t)

        # ENU world-frame velocities for circular motion of radius R at angular rate omega.
        # Integrating these from (x0, y0) traces a circle centred at (x0 - R*sign, y0).
        msg = Twist()
        msg.linear.x  = -self._R * self._omega * math.sin(theta)
        msg.linear.y  =  self._R * self._omega * math.cos(theta)
        msg.linear.z  = 0.0
        msg.angular.z = self._omega  # yaw tracks tangent direction
        self._vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CircleNode()
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(0)))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
