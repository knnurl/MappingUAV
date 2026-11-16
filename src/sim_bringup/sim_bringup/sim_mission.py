# filepath: src/sim_bringup/sim_bringup/sim_mission.py
"""Runs one exploration mission in the rig, the way the operator would at
Gate 7: Offboard, arm, AS2 takeoff, start the explorer, wait for the end, AS2
land. Exit code 0 only if the explorer finished with DONE/COMPLETE and nothing
landed or faulted on the way. RIG ONLY: never run this against the vehicle.

  ros2 run sim_bringup sim_mission [--height 1.3] [--timeout 600] [--no-explore]
"""
import argparse
import json
import sys
import time

import rclpy
from as2_msgs.action import Land, Takeoff
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


class Mission(Node):
    def __init__(self, ns):
        super().__init__('sim_mission')
        self.ns = ns
        self.explorer = {}
        self.px4 = ''
        self.alerts = []
        self.create_subscription(String, '/frontier_explorer/status',
                                 lambda m: setattr(self, 'explorer', json.loads(m.data)), 10)
        self.create_subscription(String, '/sim/px4_state',
                                 lambda m: setattr(self, 'px4', m.data), 10)
        for topic in ('/frontier_explorer/alert', '/go_to_with_avoidance/alert',
                      '/geofence/alert'):
            self.create_subscription(
                String, topic, lambda m, t=topic: self.alerts.append(f'{t}: {m.data}'), 10)

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait(self, cond, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if cond():
                return True
        return False

    def call(self, srv_type, name, request, timeout=10.0):
        client = self.create_client(srv_type, name)
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'service {name} not available')
        fut = client.call_async(request)
        if not self.wait(fut.done, timeout):
            raise RuntimeError(f'service {name} did not answer')
        return fut.result()

    def action(self, action_type, name, goal, timeout):
        client = ActionClient(self, action_type, name)
        if not client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError(f'action {name} not available')
        fut = client.send_goal_async(goal)
        if not self.wait(fut.done, 10.0) or not fut.result().accepted:
            raise RuntimeError(f'action {name} goal not accepted')
        res = fut.result().get_result_async()
        if not self.wait(res.done, timeout):
            raise RuntimeError(f'action {name} did not finish in {timeout} s')
        return res.result().result


def run(args):
    m = Mission(args.namespace)
    ns = f'/{args.namespace}'
    log = m.get_logger().info
    try:
        m.spin(3.0)
        log('offboard + arm')
        if not m.call(SetBool, f'{ns}/set_offboard_mode', SetBool.Request(data=True)).success:
            raise RuntimeError('offboard refused')
        if not m.call(SetBool, f'{ns}/set_arming_state', SetBool.Request(data=True)).success:
            raise RuntimeError('arming refused')
        log(f'takeoff to {args.height} m')
        r = m.action(Takeoff, f'{ns}/TakeoffBehavior',
                     Takeoff.Goal(takeoff_height=args.height, takeoff_speed=0.4), 60.0)
        if not r.takeoff_success:
            raise RuntimeError('takeoff failed')
        m.spin(3.0)
        if not args.no_explore:
            log('starting the explorer')
            ok = False
            for _ in range(10):                      # map and parameter checks need a moment
                res = m.call(Trigger, '/frontier_explorer/start', Trigger.Request())
                if res.success:
                    ok = True
                    break
                log(f'start refused: {res.message}')
                m.spin(3.0)
            if not ok:
                raise RuntimeError('explorer would not start')
            done = m.wait(lambda: m.explorer.get('state') in ('DONE', 'PAUSED', 'FAULT'),
                          args.timeout)
            s = m.explorer
            log(f"explorer ended: {s.get('state')} {s.get('reason')} "
                f"known_free_m2={s.get('known_free_m2')} "
                f"unexplored={len(s.get('unexplored_clusters', []))} "
                f"elapsed={s.get('mission_elapsed_s')}")
            if not done or s.get('state') != 'DONE' or s.get('reason') != 'COMPLETE':
                raise RuntimeError(f'mission did not complete: {s.get("state")} {s.get("reason")}')
            m.spin(4.0)
        log('land')
        r = m.action(Land, f'{ns}/LandBehavior', Land.Goal(land_speed=0.4), 60.0)
        m.wait(lambda: 'armed=False' in m.px4, 15.0)
        log(f'final PX4 state: {m.px4}')
        if 'stray_setpoints=0' not in m.px4:
            raise RuntimeError('setpoints reached PX4 outside Offboard')
        log('MISSION OK')
        return 0
    except RuntimeError as exc:
        m.get_logger().error(f'MISSION FAILED: {exc}')
        for a in m.alerts[-15:]:
            m.get_logger().error(f'  {a}')
        m.get_logger().error(f'  px4: {m.px4}')
        return 1
    finally:
        m.destroy_node()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--namespace', default='drone0')
    ap.add_argument('--height', type=float, default=1.3)
    ap.add_argument('--timeout', type=float, default=600.0)
    ap.add_argument('--no-explore', action='store_true')
    args = ap.parse_args(rclpy.utilities.remove_ros_args()[1:])
    rclpy.init()
    try:
        code = run(args)
    finally:
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
