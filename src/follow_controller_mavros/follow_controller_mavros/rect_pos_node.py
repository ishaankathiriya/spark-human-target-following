#!/usr/bin/env python3
# Fly a horizontal rectangle using position setpoints.
# Waits for /flight_ready, captures hover pose as the SW corner (start),
# then streams position targets along the rectangle perimeter at the requested
# speed. Yaw tracks the edge direction the drone is currently flying along.
# Publishes /ext_pos_active heartbeat so flight_manager steps aside.
#
# CCW (default, viewed from above): E → N → W → S → repeat
# CW:                               N → E → S → W → repeat

import math
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped


class RectPosNode(Node):
    _LOOP_DT = 0.02  # 50 Hz

    def __init__(self):
        super().__init__('rect_pos_node')

        self.declare_parameter('width',     4.0)   # metres in ENU X (east)
        self.declare_parameter('height',    4.0)   # metres in ENU Y (north)
        self.declare_parameter('speed',     0.5)   # m/s along edges
        self.declare_parameter('clockwise', False)

        self._W   = self.get_parameter('width').value
        self._H   = self.get_parameter('height').value
        self._spd = self.get_parameter('speed').value
        self._cw  = self.get_parameter('clockwise').value

        latch_qos  = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Bool,        '/flight_ready',               self._ready_cb, latch_qos)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._pose_cb,  sensor_qos)

        self._pos_pub    = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self._active_pub = self.create_publisher(Bool,        '/ext_pos_active',                10)

        self._ready                = False
        self._flight_ready_pending = False
        self._cur_x = self._cur_y = self._cur_z = None
        self._hz        = None
        self._start_t   = None
        self._segments  = None   # [(x0,y0, x1,y1, yaw, length), ...]
        self._perimeter = None

        self.create_timer(self._LOOP_DT, self._loop)
        self.get_logger().info(
            f'RectPosNode — {self._W}×{self._H} m  speed={self._spd} m/s  '
            f'{"CW" if self._cw else "CCW"} — waiting for /flight_ready')

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
        hx, hy = self._cur_x, self._cur_y
        self._hz    = self._cur_z
        self._start_t = time.monotonic()
        self._build_segments(hx, hy)
        self.get_logger().info(
            f'Flight ready — rectangle corner at hover '
            f'({hx:.2f}, {hy:.2f}, {self._hz:.2f}) m, '
            f'{"CW" if self._cw else "CCW"}, '
            f'perimeter {self._perimeter:.1f} m')

    def _build_segments(self, hx: float, hy: float):
        W, H = self._W, self._H

        # Hover = SW corner (start).
        # CCW: go east first → north → west → south back to start.
        # CW:  go north first → east → south → west back to start.
        if not self._cw:
            corners = [
                (hx,     hy),       # A SW (start)
                (hx + W, hy),       # B SE
                (hx + W, hy + H),   # C NE
                (hx,     hy + H),   # D NW
            ]
        else:
            corners = [
                (hx,     hy),       # A SW (start)
                (hx,     hy + H),   # D NW
                (hx + W, hy + H),   # C NE
                (hx + W, hy),       # B SE
            ]

        segs = []
        n = len(corners)
        for i in range(n):
            x0, y0 = corners[i]
            x1, y1 = corners[(i + 1) % n]
            length  = math.hypot(x1 - x0, y1 - y0)
            yaw     = math.atan2(y1 - y0, x1 - x0)
            segs.append((x0, y0, x1, y1, yaw, length))

        self._segments  = segs
        self._perimeter = sum(s[5] for s in segs)

    def _pos_at(self, s: float):
        """Return (x, y, yaw) given arc-length s along the rectangle path."""
        for x0, y0, x1, y1, yaw, length in self._segments:
            if s <= length:
                frac = s / length
                return x0 + frac * (x1 - x0), y0 + frac * (y1 - y0), yaw
            s -= length
        x0, y0, _, _, yaw, _ = self._segments[-1]
        return x0, y0, yaw

    def _loop(self):
        if not self._ready:
            return

        s = (self._spd * (time.monotonic() - self._start_t)) % self._perimeter
        x, y, yaw = self._pos_at(s)

        sp = PoseStamped()
        sp.header.stamp    = self.get_clock().now().to_msg()
        sp.header.frame_id = 'map'
        sp.pose.position.x = x
        sp.pose.position.y = y
        sp.pose.position.z = self._hz
        sp.pose.orientation.z = math.sin(yaw / 2.0)
        sp.pose.orientation.w = math.cos(yaw / 2.0)
        self._pos_pub.publish(sp)

        self._active_pub.publish(Bool(data=True))


def main(args=None):
    rclpy.init(args=args)
    node = RectPosNode()
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
