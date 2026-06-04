#!/usr/bin/env python3
"""
HTF debug plotter — live 4-panel plot for following behaviour.

Run AFTER all 4 simulation terminals are up:
    python3 src/htf_control/scripts/debug_plotter.py

Panels (rolling 30-second window):
  1. Human velocity (depth axis, m/s)  — alpha-beta vel[2] from /htf/human_vel
  2. Drone horizontal speed (m/s)       — sqrt(vx²+vy²) from /fmu/out/vehicle_local_position_v1
  3. Relative distance (m)              — alpha-beta pos[2] from /htf/human_pos
  4. Drone yaw (degrees)                — heading from /fmu/out/vehicle_local_position_v1

Data comes directly from the pd_follow_controller's alpha-beta filter outputs
and PX4's EKF2 state estimate — no recalculation in this script.
"""

import math
import threading
import collections
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from geometry_msgs.msg import PointStamped, Vector3Stamped
from px4_msgs.msg import VehicleLocalPosition


# ─────────────────────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────────────────────
WINDOW_S      = 30      # seconds of history to show
UPDATE_MS     = 200     # plot refresh interval (ms)
FOLLOW_DIST   = 2.5     # metres — dashed reference line on distance panel
MAX_XY_SPEED  = 1.5     # m/s — dashed reference line on drone speed panel
BUF           = int(WINDOW_S * 50)  # max buffer size (50 Hz × 30 s)

_PX4_SUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


# ─────────────────────────────────────────────────────────────────────────────
# ROS 2 subscriber node (runs in a background thread)
# ─────────────────────────────────────────────────────────────────────────────

class DataCollector(Node):

    def __init__(self):
        super().__init__('htf_debug_plotter')

        self._lock = threading.Lock()
        self._t0   = time.monotonic()

        # Deque buffers: (timestamp_seconds, value)
        def buf():
            return collections.deque(maxlen=BUF)

        self.t_vel     = buf(); self.human_vel_z  = buf()
        self.t_spd     = buf(); self.drone_speed  = buf()
        self.t_dist    = buf(); self.rel_dist      = buf()
        self.t_yaw     = buf(); self.drone_yaw     = buf()

        self.create_subscription(
            Vector3Stamped, '/htf/human_vel',
            self._human_vel_cb, 10)

        self.create_subscription(
            PointStamped, '/htf/human_pos',
            self._human_pos_cb, 10)

        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._local_pos_cb, _PX4_SUB_QOS)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _human_vel_cb(self, msg: Vector3Stamped):
        t = time.monotonic() - self._t0
        with self._lock:
            self.t_vel.append(t)
            self.human_vel_z.append(msg.vector.z)   # depth velocity (m/s)

    def _human_pos_cb(self, msg: PointStamped):
        t = time.monotonic() - self._t0
        with self._lock:
            self.t_dist.append(t)
            self.rel_dist.append(msg.point.z)        # depth = relative distance (m)

    def _local_pos_cb(self, msg: VehicleLocalPosition):
        t    = time.monotonic() - self._t0
        spd  = math.sqrt(msg.vx ** 2 + msg.vy ** 2)
        yaw  = math.degrees(msg.heading)
        with self._lock:
            self.t_spd.append(t);  self.drone_speed.append(spd)
            self.t_yaw.append(t);  self.drone_yaw.append(yaw)

    # ── Thread-safe snapshot for the plotter ──────────────────────────────────

    def snapshot(self):
        with self._lock:
            return (
                list(self.t_vel),    list(self.human_vel_z),
                list(self.t_spd),    list(self.drone_speed),
                list(self.t_dist),   list(self.rel_dist),
                list(self.t_yaw),    list(self.drone_yaw),
            )


# ─────────────────────────────────────────────────────────────────────────────
# Build figure
# ─────────────────────────────────────────────────────────────────────────────

def build_figure():
    fig, axes = plt.subplots(4, 1, figsize=(12, 9), sharex=True)
    fig.suptitle('HTF Debug — live following metrics', fontsize=13, fontweight='bold')
    fig.patch.set_facecolor('#1a1a2e')

    labels = [
        ('Human velocity — depth axis  (+ = moving away)', 'm/s',    '#00d4ff'),
        ('Drone horizontal speed',                          'm/s',    '#4da6ff'),
        ('Relative distance  (human depth)',                'm',      '#00ff88'),
        ('Drone yaw',                                       'degrees','#ffaa00'),
    ]

    lines = []
    for ax, (title, ylabel, colour) in zip(axes, labels):
        ax.set_facecolor('#0d0d1a')
        ax.set_title(title, color='white', fontsize=9, loc='left', pad=3)
        ax.set_ylabel(ylabel, color='white', fontsize=8)
        ax.tick_params(colors='white', labelsize=8)
        for spine in ax.spines.values():
            spine.set_edgecolor('#444466')
        ax.grid(True, color='#333355', linewidth=0.5)
        line, = ax.plot([], [], color=colour, linewidth=1.4)
        lines.append(line)

    # Reference lines
    axes[0].axhline(0,            color='white',   linewidth=0.8, linestyle='--', alpha=0.4)
    axes[1].axhline(MAX_XY_SPEED, color='#ff6666', linewidth=0.8, linestyle='--', alpha=0.6,
                    label=f'max_xy={MAX_XY_SPEED} m/s')
    axes[1].legend(fontsize=7, labelcolor='white', facecolor='#1a1a2e', framealpha=0.6)
    axes[2].axhline(FOLLOW_DIST,  color='#ffff66', linewidth=0.8, linestyle='--', alpha=0.7,
                    label=f'follow_dist={FOLLOW_DIST} m')
    axes[2].legend(fontsize=7, labelcolor='white', facecolor='#1a1a2e', framealpha=0.6)

    axes[-1].set_xlabel('Time (s)', color='white', fontsize=9)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    return fig, axes, lines


# ─────────────────────────────────────────────────────────────────────────────
# Animation update
# ─────────────────────────────────────────────────────────────────────────────

def make_update(collector, axes, lines):

    def update(_frame):
        (t_vel,  vel_z,
         t_spd,  spd,
         t_dist, dist,
         t_yaw,  yaw) = collector.snapshot()

        datasets = [
            (t_vel,  vel_z),
            (t_spd,  spd),
            (t_dist, dist),
            (t_yaw,  yaw),
        ]

        now = time.monotonic() - collector._t0
        x_min = max(0.0, now - WINDOW_S)
        x_max = max(now, WINDOW_S)

        for ax, line, (ts, ys) in zip(axes, lines, datasets):
            if len(ts) < 2:
                continue
            # Trim to rolling window
            start = 0
            for i, t in enumerate(ts):
                if t >= x_min:
                    start = i
                    break
            ts_w = ts[start:]
            ys_w = ys[start:]
            line.set_data(ts_w, ys_w)
            ax.set_xlim(x_min, x_max)

            if ys_w:
                pad = max(abs(max(ys_w) - min(ys_w)) * 0.15, 0.1)
                ax.set_ylim(min(ys_w) - pad, max(ys_w) + pad)

        return lines

    return update


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    collector = DataCollector()

    # Spin ROS 2 in a daemon thread so the plot stays on the main thread
    ros_thread = threading.Thread(
        target=rclpy.spin, args=(collector,), daemon=True)
    ros_thread.start()

    fig, axes, lines = build_figure()
    update_fn = make_update(collector, axes, lines)

    ani = animation.FuncAnimation(
        fig, update_fn,
        interval=UPDATE_MS,
        blit=False,
        cache_frame_data=False,
    )

    print('HTF debug plotter running. Close the window to exit.')
    plt.show()

    collector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
