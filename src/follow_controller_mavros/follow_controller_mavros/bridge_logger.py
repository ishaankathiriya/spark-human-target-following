#!/usr/bin/env python3
"""
MAVROS bridge logger — opens the MAVROS bridge and records every meaningful
state transition to both stdout and a timestamped log file.

Run this to collect a diagnostic log for GitHub issue reporting.
Kill with Ctrl+C — the log file path is printed at startup.
"""
import os
import signal
import time
import logging
import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped


# ── logging setup ────────────────────────────────────────────────────────────

def _setup_file_logger() -> tuple[logging.Logger, str]:
    log_dir = os.path.expanduser('~/mavros_logs')
    os.makedirs(log_dir, exist_ok=True)
    ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    log_path = os.path.join(log_dir, f'mavros_bridge_{ts}.log')

    fmt = logging.Formatter('%(asctime)s.%(msecs)03d  %(levelname)-7s  %(message)s',
                            datefmt='%H:%M:%S')

    fh = logging.FileHandler(log_path)
    fh.setFormatter(fmt)
    fh.setLevel(logging.DEBUG)

    ch = logging.StreamHandler()
    ch.setFormatter(fmt)
    ch.setLevel(logging.DEBUG)

    logger = logging.getLogger('bridge_logger')
    logger.setLevel(logging.DEBUG)
    logger.addHandler(fh)
    logger.addHandler(ch)
    return logger, log_path


# ── node ─────────────────────────────────────────────────────────────────────

class BridgeLogger(Node):

    _SYSTEM_STATUS_NAMES = {
        0: 'UNINIT', 1: 'BOOT', 2: 'CALIBRATING', 3: 'STANDBY',
        4: 'ACTIVE', 5: 'CRITICAL', 6: 'EMERGENCY', 7: 'POWEROFF', 8: 'FLIGHT_TERMINATION',
    }

    def __init__(self, logger: logging.Logger):
        super().__init__('bridge_logger')
        self._log = logger
        self._start_t = time.monotonic()

        # State tracking
        self._prev_state = State()
        self._first_state = True
        self._connect_t: float | None = None
        self._arm_t: float | None = None
        self._ob_t: float | None = None
        self._state_count = 0

        # Subscriptions
        _best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(State, '/mavros/state', self._state_cb, 10)
        self.create_subscription(BatteryState, '/mavros/mavros/battery', self._battery_cb, _best_effort)
        self.create_subscription(PoseStamped,  '/mavros/mavros/pose',    self._pose_cb,    _best_effort)

        # Periodic heartbeat log (every 5 s)
        self.create_timer(5.0, self._heartbeat)

        self._battery_v: float | None = None
        self._pose_z: float | None = None
        self._pose_count = 0

        self._log.info('=' * 70)
        self._log.info('BridgeLogger node started — waiting for MAVROS/FC connection')
        self._log.info('Subscribing to: /mavros/state, /mavros/mavros/battery, /mavros/mavros/pose')
        self._log.info('=' * 70)

    # ── callbacks ────────────────────────────────────────────────────────────

    def _state_cb(self, msg: State):
        self._state_count += 1
        now = time.monotonic()
        elapsed = now - self._start_t

        # ── connection change ────────────────────────────────────────────────
        if msg.connected and not self._prev_state.connected:
            self._connect_t = now
            self._log.info(f'[t={elapsed:.2f}s] FC CONNECTED via MAVROS')

        if not msg.connected and self._prev_state.connected:
            self._log.warning(f'[t={elapsed:.2f}s] FC DISCONNECTED')

        # ── flight mode change ───────────────────────────────────────────────
        if msg.mode != self._prev_state.mode:
            since_connect = f'{now - self._connect_t:.2f}s after connect' if self._connect_t else 'before connect'
            since_arm     = f', {now - self._arm_t:.3f}s after arm' if self._arm_t else ''
            since_ob      = f', {now - self._ob_t:.3f}s after OFFBOARD' if self._ob_t else ''

            if msg.mode == 'OFFBOARD':
                self._ob_t = now
                self._log.info(
                    f'[t={elapsed:.2f}s] MODE → OFFBOARD  '
                    f'(armed={msg.armed}, guided={msg.guided}, {since_connect}{since_arm})'
                )
            elif self._prev_state.mode == 'OFFBOARD':
                self._log.warning(
                    f'[t={elapsed:.2f}s] MODE OFFBOARD → {msg.mode}  *** OFFBOARD LOST ***  '
                    f'(armed={msg.armed}, guided={msg.guided}{since_arm}{since_ob}, {since_connect})'
                )
                self._ob_t = None
            else:
                self._log.info(
                    f'[t={elapsed:.2f}s] MODE {self._prev_state.mode!r} → {msg.mode!r}  '
                    f'(armed={msg.armed}, {since_connect}{since_arm})'
                )

        # ── arm / disarm ─────────────────────────────────────────────────────
        if msg.armed and not self._prev_state.armed:
            self._arm_t = now
            since_ob = f', {now - self._ob_t:.3f}s after OFFBOARD' if self._ob_t else ''
            batt_str = f', battery={self._battery_v:.2f}V' if self._battery_v is not None else ''
            self._log.info(
                f'[t={elapsed:.2f}s] ARMED  '
                f'(mode={msg.mode}, guided={msg.guided}, manual_input={msg.manual_input}'
                f'{since_ob}{batt_str})'
            )

        if not msg.armed and self._prev_state.armed:
            since_arm = f'{now - self._arm_t:.2f}s armed' if self._arm_t else '?'
            self._log.info(
                f'[t={elapsed:.2f}s] DISARMED  (was armed for {since_arm}, mode={msg.mode})'
            )
            self._arm_t = None

        # ── guided / manual_input flips (useful diagnostics) ─────────────────
        if msg.guided != self._prev_state.guided:
            self._log.debug(
                f'[t={elapsed:.2f}s] guided: {self._prev_state.guided} → {msg.guided}  '
                f'(mode={msg.mode}, armed={msg.armed})'
            )

        if msg.manual_input != self._prev_state.manual_input:
            self._log.debug(
                f'[t={elapsed:.2f}s] manual_input: {self._prev_state.manual_input} → {msg.manual_input}  '
                f'(mode={msg.mode}, armed={msg.armed})'
            )

        # ── system_status change ─────────────────────────────────────────────
        if msg.system_status != self._prev_state.system_status:
            prev_name = self._SYSTEM_STATUS_NAMES.get(self._prev_state.system_status, str(self._prev_state.system_status))
            curr_name = self._SYSTEM_STATUS_NAMES.get(msg.system_status, str(msg.system_status))
            self._log.debug(
                f'[t={elapsed:.2f}s] system_status: {prev_name} → {curr_name}'
            )

        # ── first message dump ───────────────────────────────────────────────
        if self._first_state:
            self._first_state = False
            status_name = self._SYSTEM_STATUS_NAMES.get(msg.system_status, str(msg.system_status))
            self._log.info(
                f'[t={elapsed:.2f}s] First state received: '
                f'connected={msg.connected}, armed={msg.armed}, guided={msg.guided}, '
                f'manual_input={msg.manual_input}, mode={msg.mode!r}, '
                f'system_status={status_name}'
            )

        self._prev_state = msg

    def _battery_cb(self, msg: BatteryState):
        if msg.voltage > 0:
            prev = self._battery_v
            self._battery_v = msg.voltage
            # Only log on first reading or big change (>0.2V)
            if prev is None:
                self._log.info(f'Battery: {msg.voltage:.2f} V  (first reading)')
            elif abs(msg.voltage - prev) > 0.2:
                self._log.warning(f'Battery voltage change: {prev:.2f}V → {msg.voltage:.2f}V')

    def _pose_cb(self, msg: PoseStamped):
        self._pose_count += 1
        self._pose_z = msg.pose.position.z
        # Log every 100th pose to confirm pose data is flowing
        if self._pose_count == 1:
            p = msg.pose.position
            self._log.info(f'Pose data flowing: first pose = ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})')
        elif self._pose_count % 500 == 0:
            p = msg.pose.position
            self._log.debug(f'Pose sample #{self._pose_count}: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})')

    def _heartbeat(self):
        now = time.monotonic()
        elapsed = now - self._start_t
        s = self._prev_state
        status_name = self._SYSTEM_STATUS_NAMES.get(s.system_status, str(s.system_status))
        batt_str = f', battery={self._battery_v:.2f}V' if self._battery_v is not None else ''
        self._log.debug(
            f'[t={elapsed:.2f}s] HEARTBEAT — connected={s.connected}, armed={s.armed}, '
            f'guided={s.guided}, manual_input={s.manual_input}, mode={s.mode!r}, '
            f'system_status={status_name}, state_msgs_rx={self._state_count}'
            f'{batt_str}'
        )

    def on_shutdown(self):
        now = time.monotonic()
        elapsed = now - self._start_t
        self._log.info('=' * 70)
        self._log.info(f'BridgeLogger shutting down after {elapsed:.1f}s')
        self._log.info(f'Total /mavros/state messages received: {self._state_count}')
        self._log.info(f'Total pose messages received: {self._pose_count}')
        self._log.info(f'Final state: armed={self._prev_state.armed}, mode={self._prev_state.mode!r}, '
                       f'guided={self._prev_state.guided}, connected={self._prev_state.connected}')
        self._log.info('=' * 70)


# ── main ─────────────────────────────────────────────────────────────────────

def main(args=None):
    file_logger, log_path = _setup_file_logger()
    file_logger.info(f'Log file: {log_path}')

    rclpy.init(args=args)
    node = BridgeLogger(file_logger)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(0)))

    file_logger.info(f'ROS 2 node started. Log will be saved to: {log_path}')

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()
        file_logger.info(f'Log saved to: {log_path}')


if __name__ == '__main__':
    main()
