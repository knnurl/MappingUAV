#!/usr/bin/env python3
# filepath: scripts/preflight_check.py
"""Pre-flight check of the RUNNING stack. Read-only: it subscribes, reads
parameters and counts publishers; it publishes nothing and commands nothing.
Run it in the same environment as the stack, before arming.

  scripts/preflight_check.py                  # perception + watchdog (Gate 4)
  scripts/preflight_check.py --as2            # + Aerostack2 (Gate 5)
  scripts/preflight_check.py --as2 --mapping  # + map, planner (Gate 6)
  scripts/preflight_check.py --as2 --mapping --explorer   # (Gate 7)

Exit code 0 only if every check passes. A pass means the software is in the
state the runbooks expect; it says nothing about the airframe.
"""
import argparse
import json
import os
import shutil
import sys
import time

import rclpy
import yaml
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String, UInt8

EV_TOPIC = '/fmu/in/vehicle_visual_odometry'
HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_VENUE = os.path.join(HERE, '..', 'src', 'drone_bringup', 'config', 'venue_params.yaml')


class Check(Node):
    def __init__(self, args):
        super().__init__('preflight_check')
        self.args = args
        self.last = {}
        self.count = {}
        q = qos_profile_sensor_data
        self.watch('/px4_odom_bridge/state', UInt8, q)
        self.watch('/px4_odom_bridge/status', String, 10)
        self.watch('/geofence/state', UInt8, q)
        from nav_msgs.msg import Odometry
        from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
        self.watch(args.lio_odom_topic, Odometry, q)
        self.watch('/fmu/out/vehicle_local_position', VehicleLocalPosition, q)
        self.watch('/fmu/out/vehicle_status', VehicleStatus, q)
        ns = args.namespace
        if args.as2:
            from as2_msgs.msg import PlatformInfo
            from geometry_msgs.msg import PoseStamped
            from sensor_msgs.msg import BatteryState
            self.watch(f'/{ns}/platform/info', PlatformInfo, 10)
            self.watch(f'/{ns}/self_localization/pose', PoseStamped, q)
            self.watch(f'/{ns}/sensor_measurements/battery', BatteryState, q)
        if args.mapping:
            self.watch('/lio_health/state', UInt8, q)
            self.watch('/go_to_with_avoidance/status', String, 10)
        if args.explorer:
            self.watch('/frontier_explorer/status', String, 10)

    def watch(self, topic, msg_type, qos):
        def cb(msg, t=topic):
            self.last[t] = msg
            self.count[t] = self.count.get(t, 0) + 1
        self.create_subscription(msg_type, topic, cb, qos)

    def params(self, node, names):
        client = self.create_client(GetParameters, f'{node}/get_parameters')
        if not client.wait_for_service(timeout_sec=3.0):
            return None
        fut = client.call_async(GetParameters.Request(names=names))
        end = time.monotonic() + 3.0
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not fut.done():
            return None
        out = {}
        for n, v in zip(names, fut.result().values):
            out[n] = {1: v.bool_value, 2: v.integer_value, 3: v.double_value,
                      4: v.string_value}.get(v.type)
        return out


def run(args):
    c = Check(args)
    end = time.monotonic() + args.listen
    while time.monotonic() < end:
        rclpy.spin_once(c, timeout_sec=0.05)
    results = []

    def check(name, ok, detail=''):
        results.append((bool(ok), name, detail))

    def seen(topic):
        return c.last.get(topic)

    rate = {t: n / args.listen for t, n in c.count.items()}
    # --- estimation
    lio = rate.get(args.lio_odom_topic, 0.0)
    check('LIO odometry arriving at about 10 Hz', 7.0 <= lio <= 13.0, f'{lio:.1f} Hz')
    st = seen('/px4_odom_bridge/state')
    status = seen('/px4_odom_bridge/status')
    text = status.data if status else 'no status'
    check('odometry bridge FORWARDING', st is not None and st.data == 1, text)
    check('odometry bridge has rejected nothing', status is not None and 'rejected=0' in text, text)
    writers = sorted(f"{i.node_namespace.rstrip('/')}/{i.node_name}"
                     for i in c.get_publishers_info_by_topic(EV_TOPIC))
    allowed = {'/px4_odom_bridge', f'/{args.namespace}/platform'}
    check('only the bridge (and the silent AS2 endpoint) on the EV topic',
          '/px4_odom_bridge' in writers and set(writers) <= allowed, ', '.join(writers) or 'none')
    lp = seen('/fmu/out/vehicle_local_position')
    check('PX4 local position valid', lp is not None and lp.xy_valid and lp.z_valid,
          'no message' if lp is None else f'xy_valid={lp.xy_valid} z_valid={lp.z_valid}')
    check('PX4 vehicle_status arriving (watchdog arming gate)',
          seen('/fmu/out/vehicle_status') is not None)
    # --- watchdog
    f = seen('/geofence/state')
    hb = rate.get('/geofence/state', 0.0)
    check('watchdog heartbeat about 2 Hz, state OK', f is not None and f.data == 0 and hb >= 1.5,
          'silent' if f is None else f'state {f.data}, {hb:.1f} Hz')
    wp = c.params('/geofence_watchdog', ['require_streams_after_s', 'use_bridge_state',
                                        'x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max',
                                        'soft_margin'])
    check('watchdog trips on silent streams (flight setting)',
          wp is not None and wp['require_streams_after_s'] > 0, str(wp and wp['require_streams_after_s']))
    check('watchdog reads the bridge state', wp is not None and wp['use_bridge_state'] is True)
    try:
        venue = yaml.safe_load(open(args.venue_params))
        want = venue['geofence_watchdog']['ros__parameters']
        same = wp is not None and all(abs(wp[k] - want[k]) < 1e-6 for k in want)
        check('watchdog box equals venue_params', same,
              '' if same else f'running {wp}, file {want}')
    except (OSError, KeyError, TypeError) as exc:
        venue = None
        check('venue_params readable', False, str(exc))
    # --- host
    free_gb = shutil.disk_usage(os.path.expanduser('~')).free / 1024 ** 3
    check('at least 4 GB free disk', free_gb >= 4.0, f'{free_gb:.0f} GB')
    # --- Aerostack2
    ns = args.namespace
    if args.as2:
        info = seen(f'/{ns}/platform/info')
        check('AS2 platform info arriving', info is not None)
        check('AS2 platform fresh: not armed, not in Offboard (restart it between flights)',
              info is not None and not info.armed and not info.offboard,
              '' if info is None else f'armed={info.armed} offboard={info.offboard}')
        check('AS2 pose arriving', seen(f'/{ns}/self_localization/pose') is not None)
        b = seen(f'/{ns}/sensor_measurements/battery')
        frac = None if b is None else b.percentage / (100.0 if b.percentage > 1.5 else 1.0)
        check(f'battery above {args.min_battery:.0%}', frac is not None and frac >= args.min_battery,
              'no battery topic' if frac is None else f'{frac:.0%}')
        nodes = {f"{n_ns.rstrip('/')}/{n}" for n, n_ns in c.get_node_names_and_namespaces()}
        for n in ('TakeoffBehavior', 'LandBehavior', 'FollowPathBehavior', 'controller_manager'):
            check(f'AS2 node {n} running', f'/{ns}/{n}' in nodes)
    if args.mapping:
        h = seen('/lio_health/state')
        check('LIO health guard reports OK', h is not None and h.data == 0,
              'silent' if h is None else f'state {h.data}')
        check('planner status arriving', seen('/go_to_with_avoidance/status') is not None)
        mp = c.params('/map_interface', ['x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max'])
        pp = c.params('/go_to_with_avoidance', ['x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max'])
        if venue:
            for label, got, key in (('map bounds', mp, 'map_interface'),
                                    ('planner box', pp, 'go_to_with_avoidance')):
                want = {k: v for k, v in venue[key]['ros__parameters'].items() if k in (got or {})}
                check(f'{label} equal venue_params',
                      got is not None and all(abs(got[k] - want[k]) < 1e-6 for k in want),
                      '' if got else 'node not answering')
    if args.explorer:
        s = seen('/frontier_explorer/status')
        d = json.loads(s.data) if s else {}
        check('explorer IDLE and not faulted', d.get('state') == 'IDLE',
              f"{d.get('state')} {d.get('reason')}" if d else 'silent')
    c.destroy_node()
    width = max(len(n) for _, n, _ in results)
    for ok, name, detail in results:
        print(f"{'PASS' if ok else 'FAIL'}  {name:<{width}}  {detail}")
    failed = [n for ok, n, _ in results if not ok]
    print(f'\n{len(results) - len(failed)}/{len(results)} checks passed'
          + ('' if not failed else '.  DO NOT ARM.'))
    return 1 if failed else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--as2', action='store_true')
    ap.add_argument('--mapping', action='store_true')
    ap.add_argument('--explorer', action='store_true')
    ap.add_argument('--namespace', default='drone0')
    ap.add_argument('--venue-params', default=DEFAULT_VENUE)
    ap.add_argument('--min-battery', type=float, default=0.5)
    ap.add_argument('--lio-odom-topic', default='/Odometry',
                    help='/fastlio2/lio_odom when the fork runs')
    ap.add_argument('--listen', type=float, default=6.0, help='seconds to listen')
    args = ap.parse_args()
    rclpy.init()
    try:
        code = run(args)
    finally:
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
