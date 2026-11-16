# filepath: src/sim_bringup/sim_bringup/sim_scenarios.py
"""Failure scenarios in the rig. Each takes off, starts a long goal, injects
one event and checks what the REAL stack does. RIG ONLY.

  ros2 run sim_bringup sim_scenarios hover_cancel | takeover | lio_jump | lio_silent
Start the rig with explorer:=false. Exit code 0 on pass.
"""
import math
import sys
import time

import rclpy
from as2_msgs.action import Takeoff
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty, String
from std_srvs.srv import SetBool

from .sim_mission import Mission

GOAL = (6.0, 0.0, 1.3)      # in the second room: several seconds of flight


class Scenario(Mission):
    def __init__(self):
        super().__init__('drone0')
        self.pose = None
        self.planner = []
        self.create_subscription(PoseStamped, '/drone0/self_localization/pose',
                                 self.on_pose, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(String, '/go_to_with_avoidance/alert',
                                 lambda m: self.planner.append(m.data), 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/go_to_with_avoidance/goal', 10)
        self.hover_pub = self.create_publisher(Empty, '/geofence/hover_request', 10)
        self.takeover_pub = self.create_publisher(Empty, '/sim/pilot_takeover', 10)
        self.fault_pub = self.create_publisher(String, '/sim/lio_fault', 10)

    def on_pose(self, msg):
        p = msg.pose.position
        self.pose = (p.x, p.y, p.z)

    def send_goal(self, xyz=GOAL):
        g = PoseStamped()
        g.header.frame_id = 'odom'
        g.pose.position.x, g.pose.position.y, g.pose.position.z = xyz
        g.pose.orientation.w = 1.0
        self.goal_pub.publish(g)

    def airborne(self):
        self.spin(3.0)
        assert self.call(SetBool, '/drone0/set_offboard_mode', SetBool.Request(data=True)).success
        assert self.call(SetBool, '/drone0/set_arming_state', SetBool.Request(data=True)).success
        r = self.action(Takeoff, '/drone0/TakeoffBehavior',
                        Takeoff.Goal(takeoff_height=1.3, takeoff_speed=0.4), 60.0)
        assert r.takeoff_success, 'takeoff failed'
        self.spin(6.0)                                 # let the map fill
        for _ in range(5):
            self.send_goal()
            if self.wait(lambda: self.pose and self.pose[0] > 1.0, 6.0):
                return
        raise AssertionError(f'vehicle never moved towards the goal: {self.planner[-3:]}')

    def moved_over(self, seconds):
        a = self.pose
        self.spin(seconds)
        return math.dist(a, self.pose)


def hover_cancel(s):
    s.airborne()
    s.hover_pub.publish(Empty())
    assert s.wait(lambda: any('hover requested' in a for a in s.planner), 3.0), s.planner
    s.spin(2.0)                                        # let it stop
    drift = s.moved_over(5.0)
    assert drift < 0.15, f'vehicle kept moving after the cancel: {drift:.2f} m in 5 s'
    assert s.pose[0] < GOAL[0] - 1.0, 'vehicle reached the goal anyway'
    assert 'OFFBOARD' in s.px4, s.px4
    return f'cancelled at x={s.pose[0]:.2f}, drift {drift:.3f} m in 5 s, still in Offboard'


def takeover(s):
    s.airborne()
    s.takeover_pub.publish(Empty())
    assert s.wait(lambda: 'POSCTL' in s.px4, 3.0), s.px4
    s.spin(2.0)
    drift = s.moved_over(4.0)
    assert drift < 0.15, f'vehicle moved {drift:.2f} m after the pilot took over'
    assert 'stray_setpoints=0' in s.px4, s.px4
    n = len(s.planner)
    s.send_goal((1.0, 0.0, 1.3))
    assert s.wait(lambda: any('not in Offboard' in a for a in s.planner[n:]), 4.0), s.planner[n:]
    return f'held in POSCTL (drift {drift:.3f} m), no stray setpoints, new goal refused'


def lio_fault(kind):
    def run(s):
        s.airborne()
        s.fault_pub.publish(String(data=kind))
        assert s.wait(lambda: 'AUTO_LAND' in s.px4 or 'armed=False' in s.px4, 6.0), \
            f'no land within 6 s of LIO {kind}: {s.px4} {s.alerts[-3:]}'
        t0 = time.monotonic()
        assert s.wait(lambda: 'armed=False' in s.px4, 30.0), s.px4
        assert 'stray_setpoints=0' in s.px4, s.px4
        why = [a for a in s.alerts if a.startswith('/geofence/alert: LAND')][-1][:150]
        return f'landed and disarmed {time.monotonic() - t0:.0f} s later; {why}'
    return run


SCENARIOS = {'hover_cancel': hover_cancel, 'takeover': takeover,
             'lio_jump': lio_fault('jump'), 'lio_silent': lio_fault('silent')}


def main():
    name = rclpy.utilities.remove_ros_args()[1:][0]
    rclpy.init()
    s = Scenario()
    code = 0
    try:
        print(f'SCENARIO {name} PASS: {SCENARIOS[name](s)}')
    except (AssertionError, RuntimeError) as exc:
        print(f'SCENARIO {name} FAIL: {exc}')
        code = 1
    finally:
        s.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
