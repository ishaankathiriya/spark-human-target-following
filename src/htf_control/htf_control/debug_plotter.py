#!/usr/bin/env python3
"""
HTF debug plotter — live 4-panel plot with ground truth overlay.

Panels (rolling 30-second window):
  1. Human velocity  — filter estimate (cyan) vs ground truth (red dashed)
  2. Drone horizontal speed
  3. Relative distance  (filter depth estimate)
  4. Drone yaw

Ground truth comes from /ground_truth/human_vel (published by ground_truth_node).
It shows the WORLD-FRAME X velocity of the walking_human actor (exact, no noise).
For a hovering drone facing +X, this should match the filter's depth velocity.

Run:
  ros2 run htf_control ground_truth_node   &
  ros2 run htf_control debug_plotter
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
WINDOW_S     = 30
UPDATE_MS    = 200
FOLLOW_DIST  = 2.5
MAX_XY_SPEED = 1.5
BUF          = int(WINDOW_S * 50)

_PX4_SUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


# ─────────────────────────────────────────────────────────────────────────────
class DataCollector(Node):

    def __init__(self):
        super().__init__('htf_debug_plotter')

        self._lock = threading.Lock()
        self._t0   = time.monotonic()

        def buf():
            return collections.deque(maxlen=BUF)

        # Filter estimates
        self.t_vel    = buf(); self.human_vel_z = buf()
        self.t_spd    = buf(); self.drone_speed = buf()
        self.t_dist   = buf(); self.rel_dist    = buf()
        self.t_yaw    = buf(); self.drone_yaw   = buf()

        # Ground truth (from ground_truth_node)
        self.t_gt  = buf(); self.gt_vel_x = buf()   # world-X velocity of actor

        self.create_subscription(
            Vector3Stamped, '/htf/human_vel',
            self._human_vel_cb, 10)
        self.create_subscription(
            PointStamped, '/htf/human_pos',
            self._human_pos_cb, 10)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._local_pos_cb, _PX4_SUB_QOS)
        self.create_subscription(
            Vector3Stamped, '/ground_truth/human_vel',
            self._gt_vel_cb, 10)

    def _human_vel_cb(self, msg: Vector3Stamped):
        t = time.monotonic() - self._t0
        with self._lock:
            self.t_vel.append(t)
            self.human_vel_z.append(msg.vector.z)

    def _human_pos_cb(self, msg: PointStamped):
        t = time.monotonic() - self._t0
        with self._lock:
            self.t_dist.append(t)
            self.rel_dist.append(msg.point.z)

    def _local_pos_cb(self, msg: VehicleLocalPosition):
        t = time.monotonic() - self._t0
        with self._lock:
            self.t_spd.append(t);  self.drone_speed.append(math.sqrt(msg.vx**2 + msg.vy**2))
            self.t_yaw.append(t);  self.drone_yaw.append(math.degrees(msg.heading))

    def _gt_vel_cb(self, msg: Vector3Stamped):
        t = time.monotonic() - self._t0
        with self._lock:
            self.t_gt.append(t)
            # Human walks along world X axis — vx is the relevant component
            self.gt_vel_x.append(msg.vector.x)

    def snapshot(self):
        with self._lock:
            return (
                list(self.t_vel),  list(self.human_vel_z),
                list(self.t_spd),  list(self.drone_speed),
                list(self.t_dist), list(self.rel_dist),
                list(self.t_yaw),  list(self.drone_yaw),
                list(self.t_gt),   list(self.gt_vel_x),
            )


# ─────────────────────────────────────────────────────────────────────────────
def build_figure():
    fig, axes = plt.subplots(4, 1, figsize=(13, 10), sharex=True)
    fig.suptitle('HTF Debug — filter estimate vs ground truth', fontsize=13, fontweight='bold')
    fig.patch.set_facecolor('#1a1a2e')

    panel_meta = [
        ('Human velocity  [cyan=filter est.  red=ground truth]', 'm/s',    '#00d4ff'),
        ('Drone horizontal speed',                                'm/s',    '#4da6ff'),
        ('Relative distance  (filter depth)',                     'm',      '#00ff88'),
        ('Drone yaw',                                             'degrees','#ffaa00'),
    ]

    lines = []
    gt_line = None

    for i, (ax, (title, ylabel, colour)) in enumerate(zip(axes, panel_meta)):
        ax.set_facecolor('#0d0d1a')
        ax.set_title(title, color='white', fontsize=9, loc='left', pad=3)
        ax.set_ylabel(ylabel, color='white', fontsize=8)
        ax.tick_params(colors='white', labelsize=8)
        for spine in ax.spines.values():
            spine.set_edgecolor('#444466')
        ax.grid(True, color='#333355', linewidth=0.5)

        line, = ax.plot([], [], color=colour, linewidth=1.4,
                        label='filter' if i == 0 else None)
        lines.append(line)

        if i == 0:
            # Ground truth overlay on velocity panel
            gt_line, = ax.plot([], [], color='#ff4444', linewidth=1.2,
                               linestyle='--', label='ground truth (world vx)')
            ax.axhline(0, color='white', linewidth=0.7, linestyle='--', alpha=0.3)
            ax.legend(fontsize=7, labelcolor='white', facecolor='#1a1a2e',
                      framealpha=0.6, loc='upper right')

    axes[1].axhline(MAX_XY_SPEED, color='#ff6666', linewidth=0.8, linestyle='--',
                    alpha=0.7, label=f'max_xy={MAX_XY_SPEED} m/s')
    axes[1].legend(fontsize=7, labelcolor='white', facecolor='#1a1a2e', framealpha=0.6)

    axes[2].axhline(FOLLOW_DIST, color='#ffff66', linewidth=0.8, linestyle='--',
                    alpha=0.8, label=f'follow_dist={FOLLOW_DIST} m')
    axes[2].legend(fontsize=7, labelcolor='white', facecolor='#1a1a2e', framealpha=0.6)

    axes[-1].set_xlabel('Time (s)', color='white', fontsize=9)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    return fig, axes, lines, gt_line


# ─────────────────────────────────────────────────────────────────────────────
def make_update(collector, axes, lines, gt_line):

    def update(_frame):
        (t_vel,  vel_z,
         t_spd,  spd,
         t_dist, dist,
         t_yaw,  yaw,
         t_gt,   gt_vx) = collector.snapshot()

        now   = time.monotonic() - collector._t0
        x_min = max(0.0, now - WINDOW_S)
        x_max = max(now, WINDOW_S)

        datasets = [
            (t_vel, vel_z),
            (t_spd, spd),
            (t_dist, dist),
            (t_yaw, yaw),
        ]

        for i, (ax, line, (ts, ys)) in enumerate(zip(axes, lines, datasets)):
            if len(ts) < 2:
                continue
            # Trim to rolling window
            start = next((j for j, t in enumerate(ts) if t >= x_min), 0)
            ts_w = ts[start:]
            ys_w = ys[start:]
            line.set_data(ts_w, ys_w)
            ax.set_xlim(x_min, x_max)

            # Autoscale Y
            all_y = list(ys_w)
            if i == 0 and len(t_gt) >= 2:
                start_gt = next((j for j, t in enumerate(t_gt) if t >= x_min), 0)
                gt_w = list(gt_vx)[start_gt:]
                gt_line.set_data(list(t_gt)[start_gt:], gt_w)
                all_y += gt_w

            if all_y:
                pad = max(abs(max(all_y) - min(all_y)) * 0.15, 0.05)
                ax.set_ylim(min(all_y) - pad, max(all_y) + pad)

        return lines + [gt_line]

    return update


# ─────────────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    collector = DataCollector()

    ros_thread = threading.Thread(
        target=rclpy.spin, args=(collector,), daemon=True)
    ros_thread.start()

    fig, axes, lines, gt_line = build_figure()
    update_fn = make_update(collector, axes, lines, gt_line)

    ani = animation.FuncAnimation(
        fig, update_fn, interval=UPDATE_MS,
        blit=False, cache_frame_data=False)

    print('HTF debug plotter running. Close the window to exit.')
    plt.show()

    collector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
