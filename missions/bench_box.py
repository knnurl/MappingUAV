# filepath: missions/bench_box.py
"""WP-B mission entry point: 2 x 2 m square at 0.5 m altitude (masterplan §7).

EXECUTION PRECONDITION: gate5_as2_behaviors CONFIRMED in
docs/gate_status.yaml, human pilot on the sticks, tether per the Gate 4
checklist. The agent never runs this; it is the Gate 5 acceptance mission.

Usage (vehicle, gate5 session):
    python3 missions/bench_box.py [--namespace drone0] [--speed 0.5]

Status: BUILT_UNVERIFIED. Closed-loop behavior transitions are untestable
without sim or vehicle (VD-004).
"""
import argparse
import sys

import rclpy
from as2_python_api.drone_interface import DroneInterface

SQUARE = [  # ENU waypoints, metres, relative to takeoff point
    (1.0, 1.0, 0.5),
    (-1.0, 1.0, 0.5),
    (-1.0, -1.0, 0.5),
    (1.0, -1.0, 0.5),
    (1.0, 1.0, 0.5),   # close the loop
]
TAKEOFF_HEIGHT = 0.5
LAND_SPEED = 0.3


def run(namespace: str, speed: float) -> int:
    drone = DroneInterface(drone_id=namespace, use_sim_time=False, verbose=True)
    try:
        print('>> offboard + arm')
        if not drone.offboard():
            print('offboard switch REJECTED', file=sys.stderr)
            return 1
        if not drone.arm():
            print('arming REJECTED', file=sys.stderr)
            return 1

        print(f'>> takeoff to {TAKEOFF_HEIGHT} m')
        if not drone.takeoff(height=TAKEOFF_HEIGHT, speed=0.3):
            print('takeoff behavior FAILED', file=sys.stderr)
            return 1

        for i, wp in enumerate(SQUARE):
            print(f'>> go_to waypoint {i + 1}/{len(SQUARE)}: {wp}')
            if not drone.go_to(*wp, speed=speed):
                print(f'go_to {wp} FAILED; landing', file=sys.stderr)
                drone.land(speed=LAND_SPEED)
                return 1

        print('>> land')
        if not drone.land(speed=LAND_SPEED):
            print('land behavior FAILED — pilot takes over', file=sys.stderr)
            return 1
        print('mission complete')
        return 0
    finally:
        drone.shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--namespace', default='drone0')
    ap.add_argument('--speed', type=float, default=0.5,
                    help='cruise speed m/s (Gate 4 envelope caps at 1.0)')
    args = ap.parse_args()
    if args.speed > 1.0:
        print('speed > 1.0 m/s violates the Gate 4 safety envelope', file=sys.stderr)
        return 2
    rclpy.init()
    try:
        return run(args.namespace, args.speed)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
