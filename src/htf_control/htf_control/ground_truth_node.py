#!/usr/bin/env python3
"""
HTF ground truth publisher.

Subscribes to Gazebo's physics-level entity poses (/gz/world/pose_info,
bridged from /world/default/pose/info), extracts the walking_human actor's
exact position, and publishes:

  /ground_truth/human_pos   — PointStamped  (world x, y, z in metres)
  /ground_truth/human_vel   — Vector3Stamped (velocity in m/s, wall-clock time)

Velocity is computed by finite-differencing consecutive positions using
wall-clock time (same reference as the alpha-beta filter), so you can
overlay this directly on the debug_plotter for a fair comparison.

Since this is ground truth (no sensor noise), finite differencing is clean.

Run alongside hover_test and debug_plotter:
  ros2 run htf_control ground_truth_node
"""

import time
import math

import rclpy
from rclpy.node import Node

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PointStamped, Vector3Stamped


class GroundTruthNode(Node):

    def __init__(self):
        super().__init__('ground_truth_node')

        self.declare_parameter('actor_name', 'walking_human')
        self._actor = self.get_parameter('actor_name').value

        self._pos_pub = self.create_publisher(PointStamped,   '/ground_truth/human_pos', 10)
        self._vel_pub = self.create_publisher(Vector3Stamped, '/ground_truth/human_vel', 10)

        self.create_subscription(TFMessage, '/gz/world/pose_info',
                                 self._pose_cb, 10)

        # Previous pose for finite differencing (wall-clock time)
        self._prev_x   = None
        self._prev_y   = None
        self._prev_z   = None
        self._prev_t   = None

        self.get_logger().info(
            f'Ground truth node ready — tracking actor "{self._actor}". '
            f'Subscribing to /gz/world/pose_info.')

    def _pose_cb(self, msg: TFMessage):
        now = time.monotonic()

        for tf in msg.transforms:
            if tf.child_frame_id != self._actor:
                continue

            x = tf.transform.translation.x
            y = tf.transform.translation.y
            z = tf.transform.translation.z

            stamp = self.get_clock().now().to_msg()

            # ── Publish position ───────────────────────────────────────────
            pos = PointStamped()
            pos.header.stamp    = stamp
            pos.header.frame_id = 'world'
            pos.point.x = x
            pos.point.y = y
            pos.point.z = z
            self._pos_pub.publish(pos)

            # ── Compute and publish velocity ───────────────────────────────
            if self._prev_x is not None:
                dt = now - self._prev_t
                if dt > 1e-3:
                    vx = (x - self._prev_x) / dt
                    vy = (y - self._prev_y) / dt
                    vz = (z - self._prev_z) / dt

                    vel = Vector3Stamped()
                    vel.header.stamp    = stamp
                    vel.header.frame_id = 'world'
                    vel.vector.x = vx
                    vel.vector.y = vy
                    vel.vector.z = vz
                    self._vel_pub.publish(vel)

                    speed = math.sqrt(vx**2 + vy**2)
                    self.get_logger().info(
                        f'GT  pos=({x:.2f}, {y:.2f})  '
                        f'vel=({vx:.3f}, {vy:.3f})  speed={speed:.3f} m/s',
                        throttle_duration_sec=1.0)

            self._prev_x = x
            self._prev_y = y
            self._prev_z = z
            self._prev_t = now
            break   # only one human — stop after finding it


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
