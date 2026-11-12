#!/usr/bin/env python3
"""Interactive SLAM mapping helper for the balance robot ROS 2 container.

Run inside ros2-start, after sourcing /home/ros2_ws/install/setup.bash.
Does not move the robot or start RViz2 on the laptop.
"""

import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

MAP_DIR = Path('/home/ros2_ws/install/balance_mapping/share/balance_mapping/maps')
MAP_NAME = 'balance_map_current'
SLAM_CMD = ['ros2', 'launch', 'balance_bringup', 'balance_slam_bringup.launch.py']



READINESS_TIMEOUT = 60.0  # seconds; SLAM map may need initial scans or movement


def wait_for_slam_ready(proc, timeout=READINESS_TIMEOUT):
    """Wait for real LiDAR and map messages, not just topic names.

    Returns (scan_received, map_received). A missing map is reported rather
    than treated as a fatal error, since mapping may require robot movement.
    """
    rclpy.init()
    node = Node('mapping_manager_readiness_check')
    received = {'scan': False, 'map': False}
    try:
        node.create_subscription(
            LaserScan, '/scan_fixed',
            lambda msg: received.__setitem__('scan', bool(msg.ranges)),
            qos_profile_sensor_data)
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        node.create_subscription(
            OccupancyGrid, '/map',
            lambda msg: received.__setitem__('map', bool(msg.data)),
            map_qos)
        print('Waiting for live /scan_fixed and /map messages...', flush=True)
        deadline = time.monotonic() + timeout
        announced = set()
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                print('SLAM launch exited before readiness checks completed.', file=sys.stderr)
                return received['scan'], received['map']
            rclpy.spin_once(node, timeout_sec=0.5)
            for key, label in [('scan', '/scan_fixed'), ('map', '/map')]:
                if received[key] and key not in announced:
                    print(f'[READY] Received {label} data.', flush=True)
                    announced.add(key)
            if all(received.values()):
                print('SLAM topics are publishing data.', flush=True)
                return True, True
        print('Readiness check timed out; see status below.', flush=True)
        return received['scan'], received['map']
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def stop_slam(proc):
    if proc.poll() is not None:
        return
    print('\nStopping SLAM...', flush=True)
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=12)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()


def save_map(prefix):
    cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(prefix)]
    print('Saving map; keep SLAM running until this completes...', flush=True)
    try:
        result = subprocess.run(cmd, timeout=45, check=False)
    except subprocess.TimeoutExpired:
        print('Map saver timed out.', file=sys.stderr)
        return False
    yaml_file = prefix.with_suffix('.yaml')
    pgm_file = prefix.with_suffix('.pgm')
    ok = result.returncode == 0 and yaml_file.is_file() and pgm_file.is_file()
    if not ok:
        print('Map saving failed or expected .pgm/.yaml files missing.', file=sys.stderr)
    return ok


def main():
    if not os.environ.get('AMENT_PREFIX_PATH'):
        print('ROS 2 environment not detected. Run ros2-start and source install/setup.bash.', file=sys.stderr)
        return 1
    if not MAP_DIR.is_dir():
        print(f'Map directory does not exist: {MAP_DIR}', file=sys.stderr)
        return 1
    prefix = MAP_DIR / MAP_NAME
    print('Starting lab-provided SLAM launch file...', flush=True)
    try:
        proc = subprocess.Popen(SLAM_CMD, start_new_session=True)
    except OSError as exc:
        print(f'Cannot start SLAM: {exc}', file=sys.stderr)
        return 1
    try:
        scan_ready, map_ready = wait_for_slam_ready(proc)
        if proc.poll() is not None:
            print('SLAM stopped during startup; cannot continue mapping.', file=sys.stderr)
            return 1
        if not scan_ready:
            print('[WARNING] No /scan_fixed messages received. Check the LiDAR and scan resampler.', flush=True)
        if not map_ready:
            print('[WARNING] No /map message received yet. SLAM may need initial scans or movement.', flush=True)
        print('\nOpen RViz2 on the Ubuntu VM, then manually map the sandbox.', flush=True)
        print('Wait for the map to appear. Do not press Enter until mapping is complete.', flush=True)

        while True:
            input('\nPress ENTER to save the map (or Ctrl+C to abort): ')
            if proc.poll() is not None:
                print('SLAM has exited; cannot save from this session.', file=sys.stderr)
                return 1
            if save_map(prefix):
                print(f'\nSaved: {prefix}.pgm and {prefix}.yaml')
                print('Use this YAML path with the navigation launch map:= argument:')
                print(f'  {prefix}.yaml')
                return 0
            print('SLAM is still running. Check the errors above, then retry saving.')
    except (KeyboardInterrupt, EOFError):
        print('\nMapping cancelled; no map save confirmed.')
        return 130
    finally:
        stop_slam(proc)


if __name__ == '__main__':
    sys.exit(main())
