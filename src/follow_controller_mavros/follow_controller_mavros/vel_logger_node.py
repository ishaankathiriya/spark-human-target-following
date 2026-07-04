#!/usr/bin/env python3
# Records cmd_vel, filtered depth and depth velocity until killed; saves a 6-panel PNG on exit.

import os
import signal
import time
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


class VelLoggerNode(Node):

    def __init__(self):
        super().__init__('vel_logger_node')

        self.declare_parameter('output_dir',      os.path.expanduser('~/vel_logs'))
        self.declare_parameter('follow_distance', 2.5)
        self._out_dir     = self.get_parameter('output_dir').value
        self._follow_dist = self.get_parameter('follow_distance').value
        os.makedirs(self._out_dir, exist_ok=True)

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Twist,   '/mavros/setpoint_velocity/cmd_vel_unstamped',
                                 self._vel_cb,       sensor_qos)
        self.create_subscription(Float32, '/follow_controller/filtered_depth',
                                 self._depth_cb,     sensor_qos)
        self.create_subscription(Float32, '/follow_controller/filtered_depth_vel',
                                 self._depth_vel_cb, sensor_qos)

        self._lock = threading.Lock()
        self._t  = []; self._vx = []; self._vy = []; self._vz = []; self._wz = []
        self._td  = []; self._depth = []
        self._tdv = []; self._dvel  = []
        self._t0 = None

        self.get_logger().info(f'Logging to {self._out_dir} — Ctrl+C to save and exit')

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

    def save(self):
        with self._lock:
            t     = list(self._t);   vx    = list(self._vx);    vy    = list(self._vy)
            vz    = list(self._vz);  wz    = list(self._wz)
            td    = list(self._td);  depth = list(self._depth)
            tdv   = list(self._tdv); dvel  = list(self._dvel)

        if not t and not td:
            self.get_logger().warn('No data received — nothing to save')
            return

        fig, axes = plt.subplots(6, 1, figsize=(14, 13), sharex=False)
        fig.suptitle('Velocity log — world-frame setpoints and filtered human depth', fontsize=11)

        vel_panels = [
            (axes[0], t, vx, 'vx  East (m/s)',   '#2196F3'),
            (axes[1], t, vy, 'vy  North (m/s)',  '#FF9800'),
            (axes[2], t, vz, 'vz  Up (m/s)',     '#4CAF50'),
            (axes[3], t, wz, 'yaw rate (rad/s)', '#E91E63'),
        ]
        for ax, ts, data, label, color in vel_panels:
            if ts:
                ax.plot(ts, data, color=color, linewidth=1.0)
            ax.axhline(0.0, color='#888888', linewidth=0.6, linestyle='--')
            ax.set_ylabel(label, fontsize=8)
            ax.grid(True, alpha=0.25)

        if td:
            axes[4].plot(td, depth, color='#00BCD4', linewidth=1.0)
        axes[4].axhline(self._follow_dist, color='#FF5722', linewidth=0.9,
                        linestyle='--', label=f'follow_dist ({self._follow_dist} m)')
        axes[4].set_ylabel('filtered depth (m)', fontsize=8)
        axes[4].set_ylim(bottom=0.0)
        axes[4].legend(fontsize=7, loc='upper right')
        axes[4].grid(True, alpha=0.25)

        if tdv:
            axes[5].plot(tdv, dvel, color='#8BC34A', linewidth=1.0)
        axes[5].axhline(0.0, color='#888888', linewidth=0.6, linestyle='--')
        axes[5].set_ylabel('filtered depth vel (m/s)', fontsize=8)
        axes[5].grid(True, alpha=0.25)

        axes[-1].set_xlabel('time (s)', fontsize=9)
        fig.tight_layout()

        stamp    = datetime.now().strftime('%Y%m%d_%H%M%S')
        out_path = os.path.join(self._out_dir, f'vel_log_{stamp}.png')
        fig.savefig(out_path, dpi=150)
        plt.close(fig)
        self.get_logger().info(f'Saved {out_path}')


def main(args=None):
    rclpy.init(args=args)
    node = VelLoggerNode()
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(0)))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
