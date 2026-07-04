#!/usr/bin/env python3
# Real-time plot of velocity setpoints + alpha-beta filtered human depth and depth velocity.
# Publishes /flight_ready so pd_follow_controller starts without flight_manager.

import collections
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist

import matplotlib.pyplot as plt
import matplotlib.animation as animation

_WINDOW_S = 10.0
_MAX_PTS  = 2000


class VelPlotNode(Node):

    def __init__(self):
        super().__init__('vel_plot_node')

        self.declare_parameter('follow_distance', 2.5)
        self._follow_dist = self.get_parameter('follow_distance').value

        latch_qos  = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._ready_pub = self.create_publisher(Bool, '/flight_ready', latch_qos)
        self._ready_pub.publish(Bool(data=True))
        self.get_logger().info('/flight_ready published — controller will activate')

        self.create_subscription(Twist,   '/mavros/setpoint_velocity/cmd_vel_unstamped',
                                 self._vel_cb,       sensor_qos)
        self.create_subscription(Float32, '/follow_controller/filtered_depth',
                                 self._depth_cb,     sensor_qos)
        self.create_subscription(Float32, '/follow_controller/filtered_depth_vel',
                                 self._depth_vel_cb, sensor_qos)

        self._lock = threading.Lock()
        self._t  = collections.deque(maxlen=_MAX_PTS)
        self._vx = collections.deque(maxlen=_MAX_PTS)
        self._vy = collections.deque(maxlen=_MAX_PTS)
        self._vz = collections.deque(maxlen=_MAX_PTS)
        self._wz = collections.deque(maxlen=_MAX_PTS)
        self._td    = collections.deque(maxlen=_MAX_PTS)
        self._depth = collections.deque(maxlen=_MAX_PTS)
        self._tdv    = collections.deque(maxlen=_MAX_PTS)
        self._dvel   = collections.deque(maxlen=_MAX_PTS)
        self._t0 = None

    def _vel_cb(self, msg: Twist):
        now = time.monotonic()
        with self._lock:
            if self._t0 is None:
                self._t0 = now
            self._t.append(now - self._t0)
            self._vx.append(msg.linear.x)
            self._vy.append(msg.linear.y)
            self._vz.append(msg.linear.z)
            self._wz.append(msg.angular.z)

    def _depth_cb(self, msg: Float32):
        now = time.monotonic()
        with self._lock:
            if self._t0 is None:
                self._t0 = now
            self._td.append(now - self._t0)
            self._depth.append(msg.data)

    def _depth_vel_cb(self, msg: Float32):
        now = time.monotonic()
        with self._lock:
            if self._t0 is None:
                self._t0 = now
            self._tdv.append(now - self._t0)
            self._dvel.append(msg.data)

    def snapshot(self):
        with self._lock:
            return (
                list(self._t),   list(self._vx),    list(self._vy),
                list(self._vz),  list(self._wz),
                list(self._td),  list(self._depth),
                list(self._tdv), list(self._dvel),
            )


def main(args=None):
    rclpy.init(args=args)
    node = VelPlotNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fig, axes = plt.subplots(6, 1, figsize=(11, 12), sharex=False)
    fig.suptitle('Follow controller — velocity setpoints and filtered human depth', fontsize=10)

    _vel_cfg = [
        ('vx  East (m/s)',   '#2196F3'),
        ('vy  North (m/s)',  '#FF9800'),
        ('vz  Up (m/s)',     '#4CAF50'),
        ('yaw rate (rad/s)', '#E91E63'),
    ]

    lines = []
    for ax, (label, color) in zip(axes[:4], _vel_cfg):
        line, = ax.plot([], [], color=color, linewidth=1.2)
        ax.axhline(0.0, color='#888888', linewidth=0.6, linestyle='--')
        ax.set_ylabel(label, fontsize=8)
        ax.grid(True, alpha=0.25)
        lines.append(line)

    # Filtered depth (position)
    depth_line, = axes[4].plot([], [], color='#00BCD4', linewidth=1.4)
    axes[4].axhline(node._follow_dist, color='#FF5722', linewidth=0.9,
                    linestyle='--', label=f'follow_dist ({node._follow_dist} m)')
    axes[4].set_ylabel('filtered depth (m)', fontsize=8)
    axes[4].set_ylim(0.0, 6.0)
    axes[4].legend(fontsize=7, loc='upper right')
    axes[4].grid(True, alpha=0.25)
    lines.append(depth_line)

    # Filtered depth velocity
    dvel_line, = axes[5].plot([], [], color='#8BC34A', linewidth=1.4)
    axes[5].axhline(0.0, color='#888888', linewidth=0.6, linestyle='--')
    axes[5].set_ylabel('filtered depth vel (m/s)', fontsize=8)
    axes[5].grid(True, alpha=0.25)
    lines.append(dvel_line)

    axes[-1].set_xlabel('time (s)', fontsize=9)
    fig.tight_layout()

    def _update(_frame):
        t, vx, vy, vz, wz, td, depth, tdv, dvel = node.snapshot()

        t_now = t[-1] if t else (td[-1] if td else 0.0)
        x_lo  = max(0.0, t_now - _WINDOW_S)
        x_hi  = t_now + 0.3

        for ax, line, data in zip(axes[:4], lines[:4], [vx, vy, vz, wz]):
            if t:
                line.set_data(t, data)
                ax.set_xlim(x_lo, x_hi)
                peak = max(abs(min(data)), abs(max(data))) if data else 0.1
                ax.set_ylim(-(peak or 0.1) * 1.3, (peak or 0.1) * 1.3)

        if td:
            depth_line.set_data(td, depth)
            axes[4].set_xlim(max(0.0, td[-1] - _WINDOW_S), td[-1] + 0.3)

        if tdv:
            dvel_line.set_data(tdv, dvel)
            axes[5].set_xlim(max(0.0, tdv[-1] - _WINDOW_S), tdv[-1] + 0.3)
            peak = max(abs(min(dvel)), abs(max(dvel))) if dvel else 0.1
            axes[5].set_ylim(-(peak or 0.1) * 1.3, (peak or 0.1) * 1.3)

        return lines

    _ani = animation.FuncAnimation(fig, _update, interval=100, blit=True)  # noqa: F841
    plt.show()

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
