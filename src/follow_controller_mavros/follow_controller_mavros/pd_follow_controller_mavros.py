#!/usr/bin/env python3
"""
Port of pf_follow_controller.py to MAVROS transport.
Control logic (alpha-beta filter, feedforward PD, yaw bearing, lookahead)
is identical to the original. Only the transport layer changes.

State machine: PRIMING -> ARMING -> HOLD -> FOLLOW
(no autonomous takeoff - pilot arms and takes off manually, then switches to OFFBAORD)

Altitude control disabled - camera Y pitch-coupling makes it unrealiable
on a fixed-mount camera. FC altitude hold (linear.z=0) is used instead
"""

import math
import signal
import time
from enum import IntEnum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PointStamped, Twist, PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State

# Alpha - Beta filter

class _AxisEstimator:
    _MIN_DT = 1e-3

    def __init__(self, alpha, beta, max_plausible_speed=8.0):
        self._alpha = alpha
        self._beta = beta 
        self._max_v = max_plausible_speed
        self._pos = 0.0
        self._vel = 0.0
        self._t = 0.0
        self._ready = False

    def update(self, meas, t):
        if not self._ready:
            self._pos, self._vel, self._t, self._ready = meas, 0.0, t, True
            return self._pos, 0.0

        dt = max(t - self._t, self._MIN_DT)
        pred = self._pos + self._vel * dt
        innovation = meas - pred 
        if abs(innovation) > self._max_v * dt:
            self._t = t
            return self._pos, self._vel
        self._t = t
        self._pos = pred + self._alpha * innovation
        self._vel = self._vel + (self._beta / dt) * innovation
        return self._pos, self._vel

    def predict(self, t):
        if not self._ready:
            return 0.0, 0.0
        dt = max(t - self._t, 0.0)
        return self._pos + self._vel * dt, self._vel

    @property
    def ready(self):
        return self._ready

    

# State - Machine

class _State(IntEnum):
    PRIMING = 0
    ARMING = 1
    HOLD = 2
    FOLLOW = 3


# Node

class PDFollowControllerMavros(Node):
    _PREARM_CYCLES = 100         # PRIMING: zero-vel cycles before entering ARMING (~2s at 50Hz)
    _ARMING_STREAM_CYCLES = 150  # ARMING: additional zero-vel cycles before first mode/arm attempt (~3s)
    _TARGET_TIMEOUT = 1.0  # seconds without detection before Follow -> HOLD
    _LOOP_DT = 0.02 # 50 Hz control loop

    def __init__(self):
        super().__init__('pd_follow_controller_mavros')

        # Params
        self.declare_parameter('follow_distance', 2.5)
        self.declare_parameter('max_xy_speed', 1.5)
        self.declare_parameter('max_yaw_rate', 0.8)
        self.declare_parameter('ab_alpha', 0.5)
        self.declare_parameter('ab_beta', 0.08)
        self.declare_parameter('max_plausible_speed', 8.0)
        self.declare_parameter('ff_dist', 1.0)
        self.declare_parameter('kp_dist', 0.5)
        self.declare_parameter('kd_dist', 0.3)
        self.declare_parameter('kp_yaw', 1.5)
        self.declare_parameter('yaw_deadband', 0.05)
        self.declare_parameter('ff_lat', 0.0)
        self.declare_parameter('kp_lat', 0.0)
        self.declare_parameter('kd_lat', 0.0)
        self.declare_parameter('lat_deadband', 0.08)
        self.declare_parameter('lookahead_t', 0.4)

        self._follow_dist = self.get_parameter('follow_distance').value
        self._max_xy      = self.get_parameter('max_xy_speed').value
        self._max_yaw     = self.get_parameter('max_yaw_rate').value
        ab_alpha          = self.get_parameter('ab_alpha').value
        ab_beta           = self.get_parameter('ab_beta').value
        max_spd           = self.get_parameter('max_plausible_speed').value
        self._ff_dist     = self.get_parameter('ff_dist').value
        self._kp_dist     = self.get_parameter('kp_dist').value
        self._kd_dist     = self.get_parameter('kd_dist').value
        self._kp_yaw      = self.get_parameter('kp_yaw').value
        self._yaw_db      = self.get_parameter('yaw_deadband').value
        self._ff_lat      = self.get_parameter('ff_lat').value
        self._kp_lat      = self.get_parameter('kp_lat').value
        self._kd_lat      = self.get_parameter('kd_lat').value
        self._lat_db      = self.get_parameter('lat_deadband').value
        self._lookahead   = self.get_parameter('lookahead_t').value

        if ab_beta >= 2.0 * ab_alpha:
            ab_beta = 0.99 * 2.0 * ab_alpha
            self.get_logger().warn(f'ab_beta clamped to {ab_beta:.3f}')

        # [0] = lateral(x), [1] = altitude(y, updated but unused), [2] = depth(z)
        self._ab = [_AxisEstimator(ab_alpha, ab_beta, max_spd) for _ in range(3)]

        # MAVROS 2.x inner UAS node exposes sensor topics with BEST_EFFORT QoS
        _sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscriptions
        self.create_subscription(State, '/mavros/state', self._state_cb, 10)
        self.create_subscription(PoseStamped, '/mavros/mavros/pose', self._pose_cb, _sensor_qos)
        self.create_subscription(PointStamped, '/tracker/centroid_3d', self._centroid_cb, 10)

        # Pubs — inner UAS node topics (mavros/mavros/...)
        self._vel_pub = self.create_publisher(Twist, '/mavros/mavros/cmd_vel_unstamped', 10)
        self._pos_pub = self.create_publisher(PoseStamped, '/mavros/mavros/local', 10)

        # Service clients
        self._set_mode_cl = self.create_client(SetMode, '/mavros/set_mode')
        self._arming_cl  = self.create_client(CommandBool, '/mavros/mavros/arming')

        # Internal State
        self._fc_state = State()
        self._pose_enu = None        # latest PoseStamped from MAVROS (ENU)
        self._yaw_enu = 0.0         # current heading, extracted from quaternion
        self._centroid_seen = False
        self._last_detection = 0.0
        self._cycle = 0
        self._arming_stream = 0
        self._last_arm_t = 0.0
        self._hold_pose = None        # latched ENU PoseStamped for HOLd mode
        self._state = _State.PRIMING

        self.create_timer(self._LOOP_DT, self._control_loop)
        self.get_logger().info(
            f'PDFollowControllerMavros ready '
            f'α={ab_alpha} β={ab_beta} gate={max_spd}m/s '
            f'dist(ff={self._ff_dist} kp={self._kp_dist} kd={self._kd_dist})'
            f'yaw(kp={self._kp_yaw} db={self._yaw_db:.2f}rad)')
        

    # Callbacks

    def _state_cb(self, msg:State):
        prev = self._fc_state.connected
        self._fc_state = msg
        if msg.connected and not prev:
            self.get_logger().info('FC connected via MAVROS')

    def _pose_cb(self, msg: PoseStamped):
        self._pose_enu = msg
        q = msg.pose.orientation
        self._yaw_enu = math.atan2 ( 2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))  # Check this

    def _centroid_cb(self, msg: PointStamped):
        t = time.monotonic()
        self._ab[0].update(msg.point.x, t)
        self._ab[1].update(msg.point.y, t)  # altitude - updated but not used in control
        self._ab[2].update(msg.point.z, t)
        self._centroid_seen = True
        self._last_detection = t

    
    # Main control loop (50 Hz)

    def _control_loop(self):
        now = time.monotonic()
        is_armed = self._fc_state.armed
        in_ob = self._fc_state.mode == 'OFFBOARD'

        # PRIMING: stream zero velocity so PX4 will accept OFFBOARD mode switch.
        # Only count cycles once FC is connected — PX4 needs ~0.5s of setpoints
        # actually arriving before it will accept the OFFBOARD switch request.
        if self._state == _State.PRIMING:
            self._publish_vel(0.0, 0.0, 0.0, 0.0)
            if self._fc_state.connected:
                self._cycle += 1
            if self._cycle >= self._PREARM_CYCLES:
                self._state = _State.ARMING
                self.get_logger().info('Priming done - starting arm attempts')
                
            return

        # ARMING: stream zero vel for _ARMING_STREAM_CYCLES, then retry set_mode+arm every 1s
        if self._state == _State.ARMING:
            self._publish_vel(0.0, 0.0, 0.0, 0.0)
            if is_armed and in_ob:
                self._enter_hold('armed and in OFFBOARD')
                return
            if self._arming_stream < self._ARMING_STREAM_CYCLES:
                self._arming_stream += 1
                if self._arming_stream == self._ARMING_STREAM_CYCLES:
                    self.get_logger().info('Arming stream done - sending mode/arm commands')
                return
            if now - self._last_arm_t > 1.0:
                self._last_arm_t = now
                self.get_logger().info(f'Arm attempt - armed={is_armed}, mode={self._fc_state.mode}')
                if self._set_mode_cl.service_is_ready():
                    req = SetMode.Request()
                    req.custom_mode = 'OFFBOARD'
                    self._set_mode_cl.call_async(req)
                if self._arming_cl.service_is_ready():
                    req = CommandBool.Request()
                    req.value = True
                    self._arming_cl.call_async(req)
            return

        
        # Safety: if we lose arm or OFFBOARD mid-flight, go back to ARMING
        # if not(is_armed and in_ob):
        #     self.get_logger().warn(f'Lost arm/OFFBOARD (armed={is_armed}, mode={self._fc_state.mode})' ' - returning to ARMING.')
        #     self._state = _State.ARMING
        #     self._last_arm_t = 0.0
        #     self._publish_vel(0.0, 0.0, 0.0, 0.0)
        #     return

        # HOLD / FOLLOW
        has_target = (self._centroid_seen and now - self._last_detection < self._TARGET_TIMEOUT)

        if has_target:
            if self._state != _State.FOLLOW:
                self._state = _State.FOLLOW
                self.get_logger().info('Target acquired - entering FOLLOW')
            self._follow_target(now)
        else:
            if self._state != _State.HOLD:
                self._enter_hold('target lost')
            if self._hold_pose is not None:
                self._pos_pub.publish(self._hold_pose)

    
    # HOLD helper

    def _enter_hold(self, reason=''):
        if self._pose_enu is not None:
            self._hold_pose = self._pose_enu
        self._state = _State.HOLD
        if self._hold_pose is not None:
            p = self._hold_pose.pose.position 
            self.get_logger().info(f'HOLD ({reason}) - ENU ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})'  f'yaw={math.degrees(self._yaw_enu):.1f}°')


    # PD follow 
    def _follow_target(self, now: float):
        pos = [self._ab[i].predict(now)[0] for i in range(3)]
        vel = [self._ab[i].predict(now)[1] for i in range(3)]

        # Depth axis: PD + feedforward + lookahead
        dist_err = pos[2] - self._follow_dist
        pred_dist_err = (pos[2] + vel[2] * self._lookahead) - self._follow_dist
        kp = self._kp_dist * 2.0 if pred_dist_err < 0 else self._kp_dist
        kd = self._kd_dist * 2.0 if pred_dist_err < 0 else self._kd_dist

        body_fwd = _clamp(self._ff_dist * vel[2] + kp * pred_dist_err + kd * vel[2], -self._max_xy, self._max_xy)

        # Yaw: bearing angle P
        bearing = math.atan2(pos[0], pos[2])
        if abs(bearing) < self._yaw_db:
            bearing = 0.0
        yaw_rate = _clamp(self._kp_yaw * bearing, -self._max_yaw, self._max_yaw)

        # Lateral strafe: off by default\
        lat_p = pos[0] if abs(pos[0]) >= self._lat_db else 0.0
        body_right = _clamp(self._ff_lat * vel[0] + self._kp_lat * lat_p + self._kd_lat * vel[0], -self._max_xy, self._max_xy)

        # Altitude: disabled - linear.z = 0 lets FC hold altitude
        body_up = 0.0

        # Body -> ENU world frame rotation
        cy, sy = math.cos(self._yaw_enu), math.sin(self._yaw_enu)
        self._publish_vel(
            vx = body_fwd * cy + body_right * sy,
            vy = body_fwd * sy - body_right * cy,
            vz = body_up,
            yaw_rate = yaw_rate
        )


    # Publish Helpers


    def _publish_vel(self, vx, vy, vz, yaw_rate=0.0):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = float(vz)
        msg.angular.z = float(yaw_rate)
        self._vel_pub.publish(msg)

    def send_land(self):
        self.get_logger().warn('Shutting down - publishing zero velocity.' 'Switch RC to LAND or STABILIZED mode.')
        self._publish_vel(0.0, 0.0, 0.0, 0.0)

    
def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def main(args=None):
    rclpy.init(args=args)
    node = PDFollowControllerMavros()
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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()





