#!/usr/bin/env python3
# Fly a horizontal circle using position setpoints.
# Waits for /flight_ready, then captures hover pose as the starting point on the
# circle (drone is at theta=0, i.e. the easternmost point). The centre is
# displaced R metres to the west so the drone is already on the circle and
# transitions smoothly without any initial lateral jump.
# Publishes /ext_pos_active heartbeat so flight_manager steps aside.

import math
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped


class CirclePosNode(Node):
    _LOOP_DT = 0.02  # 50 Hz

    def __init__(self):
        super().__init__('circle_pos_node')

        self.declare_parameter('radius',    3.0)   # metres
        self.declare_parameter('speed',     0.5)   # tangential m/s
        self.declare_parameter('clockwise', False)

        self._R     = self.get_parameter('radius').value
        spd         = self.get_parameter('speed').value
        cw          = self.get_parameter('clockwise').value
        self._omega = (spd / self._R) * (-1.0 if cw else 1.0)

        latch_qos  = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Bool,        '/flight_ready',               self._ready_cb, latch_qos)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._pose_cb,  sensor_qos)

        self._pos_pub    = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self._active_pub = self.create_publisher(Bool,        '/ext_pos_active',                10)

        self._ready               = False
        self._flight_ready_pending = False
        self._cx = self._cy = self._cz = None
        self._cur_x = self._cur_y = self._cur_z = None
        self._start_t = None

        self.create_timer(self._LOOP_DT, self._loop)
        self.get_logger().info(
            f'CirclePosNode — R={self._R} m  speed={spd} m/s  '
            f'ω={self._omega:.3f} rad/s  {"CW" if cw else "CCW"} — waiting for /flight_ready')

    def _pose_cb(self, msg: PoseStamped):
        p = msg.pose.position
        self._cur_x, self._cur_y, self._cur_z = p.x, p.y, p.z
        if self._flight_ready_pending:
            self._activate()

    def _ready_cb(self, msg: Bool):
        if msg.data and not self._ready:
            if self._cur_x is not None:
                self._activate()
            else:
                self._flight_ready_pending = True

    def _activate(self):
        self._ready                = True
        self._flight_ready_pending = False
        # Hover position sits at theta=0 on the circle (easternmost point).
        # Centre is R metres to the west of hover.
        self._cx    = self._cur_x - self._R
        self._cy    = self._cur_y
        self._cz    = self._cur_z
        self._start_t = time.monotonic()
        self.get_logger().info(
            f'Flight ready — circle centre ({self._cx:.2f}, {self._cy:.2f}, {self._cz:.2f}) m, '
            f'starting at hover ({self._cur_x:.2f}, {self._cur_y:.2f})')

    def _loop(self):
        if not self._ready:
            return

        theta = self._omega * (time.monotonic() - self._start_t)

        x   = self._cx + self._R * math.cos(theta)
        y   = self._cy + self._R * math.sin(theta)
        # Yaw tracks the tangent direction of travel
        yaw = math.atan2(
            self._R * self._omega * math.cos(theta),
            -self._R * self._omega * math.sin(theta),
        )

        sp = PoseStamped()
        sp.header.stamp    = self.get_clock().now().to_msg()
        sp.header.frame_id = 'map'
        sp.pose.position.x = x
        sp.pose.position.y = y
        sp.pose.position.z = self._cz
        sp.pose.orientation.z = math.sin(yaw / 2.0)
        sp.pose.orientation.w = math.cos(yaw / 2.0)
        self._pos_pub.publish(sp)

        self._active_pub.publish(Bool(data=True))


def main(args=None):
    rclpy.init(args=args)
    node = CirclePosNode()
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
