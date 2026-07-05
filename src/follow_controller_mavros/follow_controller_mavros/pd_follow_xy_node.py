#!/usr/bin/env python3
"""
Simplified follow controller — depth + lateral only.
vz = 0.0   : FC holds hover altitude set by flight_manager.
yaw_rate = 0.0 : drone heading stays fixed throughout.

Identical to pd_follow_controller_mavros except altitude and yaw axes are disabled.
"""

import csv
import math
import os
import signal
import time
from datetime import datetime
from enum import IntEnum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import PointStamped, Twist, PoseStamped
from mavros_msgs.msg import State


# Alpha-Beta filter

class _AxisEstimator:
    _MIN_DT = 1e-3

    def __init__(self, alpha, beta, max_plausible_speed=8.0):
        self._alpha = alpha
        self._beta  = beta
        self._max_v = max_plausible_speed
        self._pos   = 0.0
        self._vel   = 0.0
        self._t     = 0.0
        self._ready = False

    def update(self, meas, t):
        if not self._ready:
            self._pos, self._vel, self._t, self._ready = meas, 0.0, t, True
            return self._pos, 0.0
        dt         = max(t - self._t, self._MIN_DT)
        pred       = self._pos + self._vel * dt
        innovation = meas - pred
        if abs(innovation) > self._max_v * dt:
            self._t = t
            return self._pos, self._vel
        self._t   = t
        self._pos = pred + self._alpha * innovation
        self._vel = self._vel + (self._beta / dt) * innovation
        return self._pos, self._vel

    def predict(self, t):
        if not self._ready:
            return 0.0, 0.0
        dt = max(t - self._t, 0.0)
        return self._pos + self._vel * dt, self._vel


# State machine

class _State(IntEnum):
    HOLD   = 0
    FOLLOW = 1


# Node

class PDFollowXYNode(Node):
    _TARGET_TIMEOUT = 1.0
    _LOOP_DT        = 0.02  # 50 Hz

    def __init__(self):
        super().__init__('pd_follow_xy')

        self.declare_parameter('follow_distance',     2.5)
        self.declare_parameter('max_xy_speed',        0.5)
        self.declare_parameter('ab_alpha',            0.5)
        self.declare_parameter('ab_beta',             0.08)
        self.declare_parameter('max_plausible_speed', 8.0)
        self.declare_parameter('ff_dist',             1.0)
        self.declare_parameter('kp_dist',             0.5)
        self.declare_parameter('kd_dist',             0.3)
        self.declare_parameter('ff_lat',              0.0)
        self.declare_parameter('kp_lat',              0.0)
        self.declare_parameter('kd_lat',              0.0)
        self.declare_parameter('lat_deadband',        0.08)
        self.declare_parameter('lookahead_t',         0.0)
        self.declare_parameter('max_detection_depth', 5.0)
        self.declare_parameter('no_target_timeout',   5.0)

        self._follow_dist       = self.get_parameter('follow_distance').value
        self._max_xy            = self.get_parameter('max_xy_speed').value
        ab_alpha                = self.get_parameter('ab_alpha').value
        ab_beta                 = self.get_parameter('ab_beta').value
        max_spd                 = self.get_parameter('max_plausible_speed').value
        self._ff_dist           = self.get_parameter('ff_dist').value
        self._kp_dist           = self.get_parameter('kp_dist').value
        self._kd_dist           = self.get_parameter('kd_dist').value
        self._ff_lat            = self.get_parameter('ff_lat').value
        self._kp_lat            = self.get_parameter('kp_lat').value
        self._kd_lat            = self.get_parameter('kd_lat').value
        self._lat_db            = self.get_parameter('lat_deadband').value
        self._lookahead         = self.get_parameter('lookahead_t').value
        self._max_depth         = self.get_parameter('max_detection_depth').value
        self._no_target_timeout = self.get_parameter('no_target_timeout').value

        if ab_beta >= 2.0 * ab_alpha:
            ab_beta = 0.99 * 2.0 * ab_alpha
            self.get_logger().warn(f'ab_beta clamped to {ab_beta:.3f}')

        # [0] = lateral (camera x), [1] = depth (camera z)
        self._ab = [_AxisEstimator(ab_alpha, ab_beta, max_spd) for _ in range(2)]

        _sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        _latch_qos  = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(State,        '/mavros/state',               self._state_cb,    10)
        self.create_subscription(PoseStamped,  '/mavros/local_position/pose', self._pose_cb,     _sensor_qos)
        self.create_subscription(PointStamped, '/tracker/centroid_3d',        self._centroid_cb, 10)
        self.create_subscription(Bool,         '/flight_ready',               self._ready_cb,    _latch_qos)

        self._vel_pub       = self.create_publisher(Twist,   '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self._depth_pub     = self.create_publisher(Float32, '/follow_controller/filtered_depth',     10)
        self._depth_vel_pub = self.create_publisher(Float32, '/follow_controller/filtered_depth_vel', 10)
        # self._body_pub = self.create_publisher(Twist, '/follow_controller/body_vel', 10)

        self._fc_state       = State()
        self._pose_enu       = None
        self._yaw_enu        = 0.0
        self._ever_detected  = False
        self._last_detection = 0.0
        self._lost_t         = None
        self._land_logged    = False
        self._state          = _State.HOLD
        self._flight_ready   = False

        # CSV logging
        log_dir = os.path.expanduser('~/follow_logs')
        os.makedirs(log_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_path = os.path.join(log_dir, f'pd_follow_xy_{stamp}.csv')
        self._log_file = open(log_path, 'w', newline='')
        self._log_writer = csv.writer(self._log_file)
        self._log_writer.writerow([
            'time_s',
            'pos_x_m', 'pos_y_m', 'pos_z_m',
            'vel_x_ms', 'vel_y_ms', 'vel_z_ms',
            'cmd_vx_ms', 'cmd_vy_ms', 'cmd_vz_ms', 'cmd_yaw_rate_rads',
        ])
        self._log_t0 = time.monotonic()
        self.get_logger().info(f'Logging to {log_path}')

        self.create_timer(self._LOOP_DT, self._control_loop)
        self.get_logger().info(
            f'PDFollowXY ready — waiting for /flight_ready | '
            f'α={ab_alpha} β={ab_beta} gate={max_spd} m/s | '
            f'dist(ff={self._ff_dist} kp={self._kp_dist} kd={self._kd_dist}) | '
            f'max_depth={self._max_depth} m | '
            f'no_target_timeout={self._no_target_timeout} s | '
            f'vz=0 yaw_rate=0'
        )

    def _state_cb(self, msg: State):
        prev = self._fc_state
        self._fc_state = msg
        if msg.connected and not prev.connected:
            self.get_logger().info('FC connected via MAVROS')

    def _pose_cb(self, msg: PoseStamped):
        self._pose_enu = msg
        q = msg.pose.orientation
        self._yaw_enu = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _centroid_cb(self, msg: PointStamped):
        if msg.point.z > self._max_depth:
            return  # ignore detections beyond max range
        t = time.monotonic()
        self._ab[0].update(msg.point.x, t)   # lateral
        self._ab[1].update(msg.point.z, t)   # depth
        self._ever_detected  = True
        self._last_detection = t

    def _ready_cb(self, msg: Bool):
        if msg.data and not self._flight_ready:
            self._flight_ready = True
            self.get_logger().info(
                'Flight ready — waiting for first detection before taking setpoints')

    def _control_loop(self):
        if not self._flight_ready:
            return

        now = time.monotonic()

        has_target = (self._ever_detected and
                      now - self._last_detection < self._TARGET_TIMEOUT)

        if has_target:
            if self._state != _State.FOLLOW:
                self._state       = _State.FOLLOW
                self._lost_t      = None
                self._land_logged = False
                self.get_logger().info('Target acquired — FOLLOW')
            self._follow_target(now)
            return

        # No target
        if not self._ever_detected:
            return  # flight_manager controls until first detection

        # Lost target after at least one detection
        if self._state == _State.FOLLOW:
            self._state  = _State.HOLD
            self._lost_t = now
            self.get_logger().warn('Target lost — hovering (zero vel)')
        elif self._lost_t is None:
            self._lost_t = now  # guard: ever_detected went True but FOLLOW was never entered

        elapsed = now - self._lost_t
        if elapsed < self._no_target_timeout:
            self._publish_vel(0.0, 0.0, 0.0)
        else:
            if not self._land_logged:
                self.get_logger().warn(
                    f'No target for {elapsed:.1f}s — stopping setpoints, '
                    f'flight_manager will AUTO.LAND')
                self._land_logged = True
            # publish nothing — flight_manager VEL_TIMEOUT → AUTO.LAND

    def _follow_target(self, now: float):
        pos = [self._ab[i].predict(now)[0] for i in range(2)]
        vel = [self._ab[i].predict(now)[1] for i in range(2)]

        pred_dist_err = (pos[1] + vel[1] * self._lookahead) - self._follow_dist
        kp = self._kp_dist * 2.0 if pred_dist_err < 0 else self._kp_dist
        kd = self._kd_dist * 2.0 if pred_dist_err < 0 else self._kd_dist

        body_fwd = _clamp(
            self._ff_dist * vel[1] + kp * pred_dist_err + kd * vel[1],
            -self._max_xy, self._max_xy,
        )

        lat_p      = pos[0] if abs(pos[0]) >= self._lat_db else 0.0
        body_right = _clamp(
            self._ff_lat * vel[0] + self._kp_lat * lat_p + self._kd_lat * vel[0],
            -self._max_xy, self._max_xy,
        )

        # bmsg = Twist()
        # bmsg.linear.x = float(body_fwd)
        # bmsg.linear.y = float(body_right)
        # bmsg.linear.z = 0.0
        # self._body_pub.publish(bmsg)

        self._depth_pub.publish(Float32(data=float(pos[1])))
        self._depth_vel_pub.publish(Float32(data=float(vel[1])))

        cy, sy = math.cos(self._yaw_enu), math.sin(self._yaw_enu)
        cmd_vx = body_fwd * cy + body_right * sy
        cmd_vy = body_fwd * sy - body_right * cy
        self._publish_vel(vx=cmd_vx, vy=cmd_vy, vz=0.0, yaw_rate=0.0)

        self._log_writer.writerow([
            f'{now - self._log_t0:.4f}',
            f'{pos[0]:.4f}', f'{0.0:.4f}',    f'{pos[1]:.4f}',
            f'{vel[0]:.4f}', f'{0.0:.4f}',    f'{vel[1]:.4f}',
            f'{cmd_vx:.4f}', f'{cmd_vy:.4f}', f'{0.0:.4f}', f'{0.0:.4f}',
        ])

    def _publish_vel(self, vx, vy, vz, yaw_rate=0.0):
        msg = Twist()
        msg.linear.x  = float(vx)
        msg.linear.y  = float(vy)
        msg.linear.z  = float(vz)
        msg.angular.z = float(yaw_rate)
        self._vel_pub.publish(msg)

    def send_land(self):
        self.get_logger().warn('Shutting down — publishing zero velocity.')
        self._publish_vel(0.0, 0.0, 0.0)

    def close_log(self):
        self._log_file.flush()
        self._log_file.close()
        self.get_logger().info('CSV log saved.')


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def main(args=None):
    rclpy.init(args=args)
    node = PDFollowXYNode()
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(0)))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.send_land()
        deadline = time.monotonic() + 0.8
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.close_log()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
