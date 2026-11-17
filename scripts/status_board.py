#!/usr/bin/env python3
# filepath: scripts/status_board.py
"""One-screen status of the running stack for the ground operator. Read-only.

Run it ON THE VEHICLE over ssh (recommended: nothing crosses the WiFi but the
terminal, so a roaming access point cannot break it):
    ssh neric@<jetson> 'python3 ~/colcon_ws/scripts/status_board.py'
or on a ground station that shares the ROS graph. --once prints one snapshot.

Each line shows the value and its age; a line turns to "SILENT" when its topic
has not been heard for 3 s. This is a display, not a safety function.
"""
import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String, UInt8

BRIDGE = {0: 'WAITING', 1: 'FORWARDING', 2: 'STALE', 3: 'BLOCKED'}
FENCE = {0: 'OK', 1: 'SOFT', 2: 'HARD'}
HEALTH = {0: 'OK', 1: 'DEGRADED'}
NAV = {2: 'POSITION', 1: 'ALTITUDE', 14: 'OFFBOARD', 18: 'LAND', 15: 'STABILIZED', 0: 'MANUAL'}
PLATFORM = {-1: 'EMERGENCY', 0: 'DISARMED', 1: 'LANDED', 2: 'TAKING_OFF', 3: 'FLYING', 4: 'LANDING'}


class Board(Node):
    def __init__(self, ns, lio_odom_topic):
        super().__init__('status_board')
        self.v = {}          # name -> (text, time)
        self.alerts = []
        q = qos_profile_sensor_data
        from as2_msgs.msg import PlatformInfo
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from px4_msgs.msg import VehicleStatus
        from sensor_msgs.msg import BatteryState
        s = self.create_subscription
        s(UInt8, '/px4_odom_bridge/state', lambda m: self.put('bridge', BRIDGE.get(m.data, m.data)), q)
        s(String, '/px4_odom_bridge/status', lambda m: self.put('bridge detail', m.data), 10)
        s(UInt8, '/geofence/state', lambda m: self.put('geofence', FENCE.get(m.data, m.data)), q)
        s(UInt8, '/lio_health/state', lambda m: self.put('LIO health', HEALTH.get(m.data, m.data)), q)
        s(Odometry, lio_odom_topic, lambda m: self.put('LIO position', self.xyz(m.pose.pose.position)), q)
        s(VehicleStatus, '/fmu/out/vehicle_status', lambda m: self.put(
            'PX4', f"{'ARMED' if m.arming_state == 2 else 'disarmed'} {NAV.get(m.nav_state, m.nav_state)}"), q)
        s(PlatformInfo, f'/{ns}/platform/info', lambda m: self.put(
            'Aerostack2', f"{PLATFORM.get(m.status.state, m.status.state)} "
                          f"offboard={m.offboard} armed={m.armed}"), 10)
        s(PoseStamped, f'/{ns}/self_localization/pose',
          lambda m: self.put('PX4 position', self.xyz(m.pose.position)), q)
        s(BatteryState, f'/{ns}/sensor_measurements/battery', lambda m: self.put(
            'battery', f'{m.percentage / (100.0 if m.percentage > 1.5 else 1.0):.0%} {m.voltage:.1f} V'), q)
        s(String, '/go_to_with_avoidance/status', self.on_planner, 10)
        s(String, '/frontier_explorer/status', self.on_explorer, 10)
        for t in ('/geofence/alert', '/lio_health/alert', '/go_to_with_avoidance/alert',
                  '/frontier_explorer/alert'):
            s(String, t, lambda m, t=t: self.alerts.append(
                (time.time(), f"{t.split('/')[1]}: {m.data}")), 10)

    @staticmethod
    def xyz(p):
        return f'x {p.x:6.2f}  y {p.y:6.2f}  z {p.z:5.2f}'

    def put(self, name, text):
        self.v[name] = (str(text), time.time())

    def on_planner(self, msg):
        d = json.loads(msg.data)
        goal = d.get('goal')
        g = '' if not goal else f" goal ({goal[0]:.1f}, {goal[1]:.1f}, {goal[2]:.1f})"
        self.put('planner', f"{d['state']} {d.get('reason', '')}{g}")

    def on_explorer(self, msg):
        d = json.loads(msg.data)
        self.put('explorer', f"{d['state']} {d.get('reason', '')}  "
                             f"t {d.get('mission_elapsed_s', 0):.0f} s  "
                             f"free {d.get('known_free_m2', 0)} m2  "
                             f"clusters {d.get('clusters', 0)}  "
                             f"interlocks {d.get('interlocks', [])}")

    def render(self):
        now = time.time()
        order = ['PX4', 'Aerostack2', 'battery', 'bridge', 'bridge detail', 'LIO health',
                 'geofence', 'LIO position', 'PX4 position', 'planner', 'explorer']
        lines = [time.strftime('%H:%M:%S') + '   stack status (read-only)']
        for name in order:
            if name not in self.v:
                lines.append(f'{name:14s} -')
                continue
            text, t = self.v[name]
            age = now - t
            lines.append(f'{name:14s} {"SILENT " + f"{age:.0f} s  (last: " + text + ")" if age > 3.0 else text}')
        lines.append('')
        lines.append('last alerts:')
        for t, a in self.alerts[-6:]:
            lines.append(f'  {time.strftime("%H:%M:%S", time.localtime(t))}  {a[:110]}')
        return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--namespace', default='drone0')
    ap.add_argument('--lio-odom-topic', default='/Odometry')
    ap.add_argument('--once', action='store_true', help='print one snapshot after 4 s and exit')
    args = ap.parse_args()
    rclpy.init()
    board = Board(args.namespace, args.lio_odom_topic)
    try:
        last = 0.0
        start = time.time()
        while rclpy.ok():
            rclpy.spin_once(board, timeout_sec=0.1)
            if args.once:
                if time.time() - start > 4.0:
                    print(board.render())
                    break
            elif time.time() - last > 0.5:
                last = time.time()
                sys.stdout.write('\x1b[2J\x1b[H' + board.render() + '\n')
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        board.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
