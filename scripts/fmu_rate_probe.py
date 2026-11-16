#!/usr/bin/env python3
# filepath: scripts/fmu_rate_probe.py
"""Rate, gap and dropout probe for PX4 /fmu/out topics.

`ros2 topic hz` (Humble) has no QoS options and subscribes RELIABLE, so it never
receives PX4's BEST_EFFORT uXRCE-DDS publishers. This probe subscribes
BEST_EFFORT and resolves each topic's type from the ROS graph.

Usage:
  fmu_rate_probe.py [--duration S] [--dropout-gap S] [--discovery S]
                    [--report-every S] TOPIC [TOPIC ...]

A dropout is any gap between messages longer than --dropout-gap, including a
topic going silent before the end of the run. Exit code 0 when every topic was
discovered, received messages and had zero dropouts; otherwise 1.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('topics', nargs='+')
    ap.add_argument('--duration', type=float, default=15.0, help='measurement time (s)')
    ap.add_argument('--dropout-gap', type=float, default=1.5, help='a longer gap counts as a dropout (s)')
    ap.add_argument('--discovery', type=float, default=5.0, help='time allowed to discover topics (s)')
    ap.add_argument('--report-every', type=float, default=0.0, help='print interim results (s); 0 = off')
    a = ap.parse_args()

    rclpy.init()
    node = Node('fmu_rate_probe')

    types = {}
    deadline = time.monotonic() + a.discovery
    while time.monotonic() < deadline and len(types) < len(a.topics):
        for name, tys in node.get_topic_names_and_types():
            if name in a.topics and tys:
                types[name] = tys[0]
        rclpy.spin_once(node, timeout_sec=0.1)
    missing = [t for t in a.topics if t not in types]
    for t in missing:
        print(f'MISSING {t}: not discovered within {a.discovery:.0f} s '
              '(is the agent running? does ROS_DOMAIN_ID equal PX4 UXRCE_DDS_DOM_ID?)')

    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=50)
    stats = {t: dict(n=0, last=None, maxgap=0.0, drops=0) for t in types}

    def make_cb(topic):
        def cb(_msg):
            s = stats[topic]
            now = time.monotonic()
            if s['last'] is not None:
                gap = now - s['last']
                s['maxgap'] = max(s['maxgap'], gap)
                if gap > a.dropout_gap:
                    s['drops'] += 1
            s['last'] = now
            s['n'] += 1
        return cb

    for t, ty in types.items():
        node.create_subscription(get_message(ty), t, make_cb(t), qos)

    settle = time.monotonic() + 1.0
    while time.monotonic() < settle:
        rclpy.spin_once(node, timeout_sec=0.05)
    for s in stats.values():
        s.update(n=0, last=None, maxgap=0.0, drops=0)

    t0 = time.monotonic()
    end = t0 + a.duration
    next_report = t0 + a.report_every if a.report_every > 0 else float('inf')

    def report(tag):
        elapsed = max(time.monotonic() - t0, 1e-9)
        for t, s in stats.items():
            print(f'[{tag} {elapsed:5.0f} s] {t:34s} {s["n"] / elapsed:7.1f} Hz  '
                  f'max gap {s["maxgap"] * 1e3:7.1f} ms  dropouts {s["drops"]}  msgs {s["n"]}', flush=True)

    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.monotonic() >= next_report:
            report('t')
            next_report += a.report_every

    now = time.monotonic()
    for s in stats.values():  # a topic that went silent near the end is a dropout too
        if s['last'] is not None and now - s['last'] > a.dropout_gap:
            s['maxgap'] = max(s['maxgap'], now - s['last'])
            s['drops'] += 1
    report('final')

    ok = not missing and all(s['n'] > 0 and s['drops'] == 0 for s in stats.values())
    print('RESULT: PASS' if ok else 'RESULT: FAIL')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
