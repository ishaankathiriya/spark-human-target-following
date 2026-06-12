#!/usr/bin/env python3
"""
HTF human world-frame state estimator (Phase 1).

Converts the camera-relative human centroid (/tracker/centroid_3d) into an
absolute position in the drone's NED world frame, using the drone's FULL
3D orientation (roll, pitch, yaw) -- not just yaw -- so the estimate stays
correct even when the drone is tilted during aggressive maneuvers.

Pipeline per centroid measurement:
    1. Camera optical frame -> Body FRD frame   (fixed axis permutation,
       0 deg camera mount tilt)
         body_fwd   = z_cam   (depth)
         body_right = x_cam
         body_down  = y_cam
    2. Body FRD -> NED                          (full quaternion rotation,
       from VehicleAttitude -- not yaw-only)
    3. + drone position (VehicleLocalPosition)  -> human's absolute NED
       position

This raw world-frame measurement feeds a 6-state constant-velocity Kalman
filter (state = [px,py,pz,vx,vy,vz] in NED), with measurement noise scaled
by depth and Mahalanobis-distance outlier gating.

Topics consumed:
    /fmu/out/vehicle_attitude            (px4_msgs/VehicleAttitude)
    /fmu/out/vehicle_local_position_v1   (px4_msgs/VehicleLocalPosition)
    /tracker/centroid_3d                 (geometry_msgs/PointStamped)
        camera optical frame: x=right, y=down, z=forward (depth)

Topics produced:
    /htf/human_world_meas   (geometry_msgs/PointStamped)  raw transformed
                             measurement (pre-filter), NED
    /htf/human_world_odom   (nav_msgs/Odometry)           filtered position
                             + velocity, NED

This node is purely observational -- it sends nothing to PX4 and does not
modify drone behaviour. It is meant to run alongside the existing
pd_follow_controller for side-by-side validation.
"""

import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry


_PX4_SUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=1)


def _quat_to_dcm(q):
    """Hamilton (w,x,y,z) quaternion -> 3x3 DCM, body-frame vec -> NED vec."""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ])


def _yaw_from_quat(q):
    """Hamilton (w,x,y,z) quaternion -> yaw (rad), NED convention."""
    w, x, y, z = q
    return np.arctan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))


# ──────────────────────────────────────────────────────────────────────────────
# 6-state constant-velocity Kalman filter: state = [px,py,pz,vx,vy,vz] in NED
# ──────────────────────────────────────────────────────────────────────────────

class _WorldKalmanFilter:

    def __init__(self, accel_noise_xy, accel_noise_z, gate_threshold):
        self._accel_noise_xy = accel_noise_xy
        self._accel_noise_z = accel_noise_z
        self._gate_threshold = gate_threshold
        self.x = np.zeros(6)
        self.P = np.eye(6)
        self.initialized = False

    def initialize(self, meas):
        self.x[:3] = meas
        self.x[3:] = 0.0
        self.P = np.eye(6)
        self.P[:3, :3] *= 0.5    # initial position uncertainty (m^2)
        self.P[3:, 3:] *= 4.0    # initial velocity uncertainty -- no idea yet (m/s)^2
        self.initialized = True

    def predict(self, dt):
        if not self.initialized or dt <= 0.0:
            return

        F = np.eye(6)
        F[0, 3] = F[1, 4] = F[2, 5] = dt

        # Discretized white-noise-acceleration process model, per axis
        Q = np.zeros((6, 6))
        for i, sigma_a in enumerate((self._accel_noise_xy,
                                      self._accel_noise_xy,
                                      self._accel_noise_z)):
            q = sigma_a ** 2
            Q[i, i]     = q * dt**4 / 4.0
            Q[i, i+3]   = q * dt**3 / 2.0
            Q[i+3, i]   = q * dt**3 / 2.0
            Q[i+3, i+3] = q * dt**2

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, meas, meas_std):
        """meas: 3-vector NED position. meas_std: 3-vector of std devs."""
        if not self.initialized:
            self.initialize(meas)
            return True

        H = np.zeros((3, 6))
        H[0, 0] = H[1, 1] = H[2, 2] = 1.0
        R = np.diag(np.square(meas_std))

        y = meas - H @ self.x
        S = H @ self.P @ H.T + R
        d2 = float(y.T @ np.linalg.solve(S, y))
        if d2 > self._gate_threshold:
            return False  # outlier -- keep prediction only

        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P
        return True


# ──────────────────────────────────────────────────────────────────────────────
# Node
# ──────────────────────────────────────────────────────────────────────────────

class HumanWorldEstimator(Node):

    _PREDICT_RATE_HZ = 50.0

    def __init__(self):
        super().__init__('human_world_estimator')

        self.declare_parameter('accel_noise_xy', 2.0)     # m/s^2, how fast a human's horizontal velocity can change
        self.declare_parameter('accel_noise_z', 0.5)      # m/s^2, vertical
        self.declare_parameter('meas_noise_coeff', 0.02)  # measurement std dev as a fraction of depth (D435i ~2%)
        self.declare_parameter('meas_noise_min', 0.05)    # floor on measurement std dev (m)
        self.declare_parameter('gate_threshold', 7.815)   # chi-square 95% for 3 DOF

        self._meas_noise_coeff = self.get_parameter('meas_noise_coeff').value
        self._meas_noise_min = self.get_parameter('meas_noise_min').value

        self._kf = _WorldKalmanFilter(
            self.get_parameter('accel_noise_xy').value,
            self.get_parameter('accel_noise_z').value,
            self.get_parameter('gate_threshold').value)

        self._q_body_to_ned = None   # latest VehicleAttitude quaternion (w,x,y,z)
        self._drone_pos_ned = None   # latest drone position [x,y,z] NED
        self._drone_pos_valid = False
        self._last_predict_t = None

        self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self._attitude_cb, _PX4_SUB_QOS)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._local_pos_cb, _PX4_SUB_QOS)
        self.create_subscription(
            PointStamped, '/tracker/centroid_3d',
            self._centroid_cb, 10)

        self._pub_meas = self.create_publisher(PointStamped, '/htf/human_world_meas', 10)
        self._pub_odom = self.create_publisher(Odometry, '/htf/human_world_odom', 10)

        self.create_timer(1.0 / self._PREDICT_RATE_HZ, self._predict_tick)

        self.get_logger().info('Human world estimator started')

    # ── Subscriptions ────────────────────────────────────────────────────────

    def _attitude_cb(self, msg: VehicleAttitude):
        self._q_body_to_ned = (msg.q[0], msg.q[1], msg.q[2], msg.q[3])

    def _local_pos_cb(self, msg: VehicleLocalPosition):
        self._drone_pos_ned = np.array([msg.x, msg.y, msg.z])
        self._drone_pos_valid = bool(msg.xy_valid and msg.z_valid)

    def _centroid_cb(self, pt_msg: PointStamped):
        if self._q_body_to_ned is None or not self._drone_pos_valid:
            return  # can't transform without attitude + a valid drone position

        x_cam, y_cam, z_cam = pt_msg.point.x, pt_msg.point.y, pt_msg.point.z
        if z_cam <= 0.0:
            return

        # Step 1: Camera optical -> Body FRD (fixed axis permutation)
        body_vec = np.array([z_cam, x_cam, y_cam])  # forward, right, down

        # Step 2: Body FRD -> NED, using FULL attitude (roll+pitch+yaw)
        R = _quat_to_dcm(self._q_body_to_ned)
        world_offset = R @ body_vec

        # Step 3: + drone's own NED position -> human's absolute NED position
        world_pos = self._drone_pos_ned + world_offset

        self.get_logger().info(
            f'yaw={np.degrees(_yaw_from_quat(self._q_body_to_ned)):.1f}deg '
            f'drone_pos={self._drone_pos_ned.round(2).tolist()} '
            f'body_vec={body_vec.round(2).tolist()} '
            f'world_pos={world_pos.round(2).tolist()}',
            throttle_duration_sec=1.0)

        self._predict_to(time.time())

        meas_std = max(self._meas_noise_coeff * z_cam, self._meas_noise_min)
        accepted = self._kf.update(world_pos, np.array([meas_std, meas_std, meas_std]))
        if not accepted:
            self.get_logger().debug('Human world estimator: rejected outlier measurement')

        meas_msg = PointStamped()
        meas_msg.header = pt_msg.header
        meas_msg.header.frame_id = 'map'
        meas_msg.point.x, meas_msg.point.y, meas_msg.point.z = world_pos.tolist()
        self._pub_meas.publish(meas_msg)

    # ── Prediction / publishing ─────────────────────────────────────────────

    def _predict_tick(self):
        self._predict_to(time.time())
        if self._kf.initialized:
            self._publish_odom()

    def _predict_to(self, now):
        if self._last_predict_t is None:
            self._last_predict_t = now
            return
        dt = now - self._last_predict_t
        self._last_predict_t = now
        self._kf.predict(dt)

    def _publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'human'

        x = self._kf.x
        msg.pose.pose.position.x = float(x[0])
        msg.pose.pose.position.y = float(x[1])
        msg.pose.pose.position.z = float(x[2])
        msg.pose.pose.orientation.w = 1.0

        msg.twist.twist.linear.x = float(x[3])
        msg.twist.twist.linear.y = float(x[4])
        msg.twist.twist.linear.z = float(x[5])

        self._pub_odom.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HumanWorldEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
