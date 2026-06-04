#!/usr/bin/env python3
"""
HTF debug logger — collects data for 4 minutes, then saves the plot as a PNG.

Run AFTER all simulation terminals are up:
    ros2 run htf_control debug_logger

The PNG is saved in the current working directory:
    htf_debug_YYYYMMDD_HHMMSS.png
"""

import math
import threading
import time
import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import matplotlib
matplotlib.use('Agg')   # non-interactive backend — no window needed
import matplotlib.pyplot as plt

from geometry_msgs.msg import PointStamped, Vector3Stamped
from px4_msgs.msg import VehicleLocalPosition


# ─────────────────────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────────────────────
RECORD_S     = 240      # 4 minutes
FOLLOW_DIST  = 2.5      # metres — reference line on distance panel
MAX_XY_SPEED = 1.5      # m/s   — reference line on drone speed panel

_PX4_SUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


# ─────────────────────────────────────────────────────────────────────────────
# ROS 2 subscriber node
# ─────────────────────────────────────────────────────────────────────────────

class DataLogger(Node):

    def __init__(self):
        super().__init__('htf_debug_logger')

        self._lock = threading.Lock()
        self._t0   = time.monotonic()

        # Raw lists — no maxlen, keep everything
        self.t_vel   = []; self.human_vel_z = []
        self.t_spd   = []; self.drone_speed = []
        self.t_dist  = []; self.rel_dist    = []
        self.t_yaw   = []; self.drone_yaw   = []

        self.create_subscription(
            Vector3Stamped, '/htf/human_vel',
            self._human_vel_cb, 10)
        self.create_subscription(
            PointStamped, '/htf/human_pos',
            self._human_pos_cb, 10)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._local_pos_cb, _PX4_SUB_QOS)

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
        t   = time.monotonic() - self._t0
        spd = math.sqrt(msg.vx ** 2 + msg.vy ** 2)
        yaw = math.degrees(msg.heading)
        with self._lock:
            self.t_spd.append(t);  self.drone_speed.append(spd)
            self.t_yaw.append(t);  self.drone_yaw.append(yaw)

    def snapshot(self):
        with self._lock:
            return (
                list(self.t_vel),   list(self.human_vel_z),
                list(self.t_spd),   list(self.drone_speed),
                list(self.t_dist),  list(self.rel_dist),
                list(self.t_yaw),   list(self.drone_yaw),
            )


# ─────────────────────────────────────────────────────────────────────────────
# Plot and save
# ─────────────────────────────────────────────────────────────────────────────

def save_plot(logger: DataLogger, filepath: str):
    (t_vel,  vel_z,
     t_spd,  spd,
     t_dist, dist,
     t_yaw,  yaw) = logger.snapshot()

    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
    fig.suptitle('HTF Debug — 4-minute following session', fontsize=13, fontweight='bold')
    fig.patch.set_facecolor('#1a1a2e')

    panels = [
        (t_vel,  vel_z, '#00d4ff', 'Human velocity — depth axis  (+ = moving away)', 'm/s'),
        (t_spd,  spd,   '#4da6ff', 'Drone horizontal speed',                          'm/s'),
        (t_dist, dist,  '#00ff88', 'Relative distance  (human depth)',                'm'),
        (t_yaw,  yaw,   '#ffaa00', 'Drone yaw',                                       'degrees'),
    ]

    for ax, (ts, ys, colour, title, ylabel) in zip(axes, panels):
        ax.set_facecolor('#0d0d1a')
        ax.set_title(title, color='white', fontsize=9, loc='left', pad=3)
        ax.set_ylabel(ylabel, color='white', fontsize=8)
        ax.tick_params(colors='white', labelsize=8)
        for spine in ax.spines.values():
            spine.set_edgecolor('#444466')
        ax.grid(True, color='#333355', linewidth=0.5)
        if ts:
            ax.plot(ts, ys, color=colour, linewidth=1.2)

    # Reference lines
    axes[0].axhline(0,            color='white',   linewidth=0.8, linestyle='--', alpha=0.4)
    axes[1].axhline(MAX_XY_SPEED, color='#ff6666', linewidth=0.8, linestyle='--', alpha=0.7,
                    label=f'max_xy = {MAX_XY_SPEED} m/s')
    axes[1].legend(fontsize=7, labelcolor='white', facecolor='#1a1a2e', framealpha=0.6)
    axes[2].axhline(FOLLOW_DIST,  color='#ffff66', linewidth=0.8, linestyle='--', alpha=0.8,
                    label=f'follow_dist = {FOLLOW_DIST} m')
    axes[2].legend(fontsize=7, labelcolor='white', facecolor='#1a1a2e', framealpha=0.6)

    axes[-1].set_xlabel('Time (s)', color='white', fontsize=9)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(filepath, dpi=150, bbox_inches='tight', facecolor=fig.get_facecolor())
    plt.close(fig)


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    logger = DataLogger()

    ros_thread = threading.Thread(
        target=rclpy.spin, args=(logger,), daemon=True)
    ros_thread.start()

    print(f'HTF debug logger — recording for {RECORD_S // 60} min {RECORD_S % 60} s ...')
    print('Ctrl+C to stop early and save now.\n')

    t_start = time.monotonic()
    try:
        while True:
            elapsed = time.monotonic() - t_start
            remaining = RECORD_S - elapsed
            if remaining <= 0:
                break
            mins, secs = divmod(int(remaining), 60)
            print(f'  {mins:02d}:{secs:02d} remaining ...', end='\r', flush=True)
            time.sleep(1.0)
    except KeyboardInterrupt:
        elapsed = time.monotonic() - t_start
        print(f'\n  Stopped early at {elapsed:.0f} s.')

    print('\nGenerating plot ...')
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    filepath  = f'htf_debug_{timestamp}.png'
    save_plot(logger, filepath)
    print(f'Saved → {filepath}')

    logger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
