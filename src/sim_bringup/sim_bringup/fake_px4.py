# filepath: src/sim_bringup/sim_bringup/fake_px4.py
"""Kinematic stand-in for PX4 behind the uXRCE-DDS topics (closed-loop rig).

NOT a simulator of flight: no attitude, thrust, wind or estimator. Position
follows offboard setpoints through a speed limit and a first-order lag. It
exists so that the real Aerostack2 platform, the watchdog, the planner and the
explorer can run against something that answers, and so that their state
machines, frames and interfaces are exercised before flight. It discharges no
verification debt.

Modes kept (as far as those nodes depend on them):
  POSCTL    holds position (the "pilot" mode)
  OFFBOARD  entered by DO_SET_MODE while a heartbeat is fresh; left when the
            heartbeat stops for 0.5 s or the pilot takes over
  AUTO_LAND entered by NAV_LAND; descends, then disarms on the ground
A setpoint received outside OFFBOARD more than 0.25 s after leaving it is
counted and reported as an error: the real PX4 would mix it into the active
mode. Setpoints inside that window are counted separately (late_setpoints):
the platform stops only when it sees the mode change, so a few are in transit.

Frames: PX4 local NED. px4_odom_bridge maps LIO x (startup forward) to East,
so NED = (y_lio, x_lio, -z_lio) and the start heading is +pi/2.

Test hooks (std_msgs): /sim/pilot_takeover (Empty), /sim/battery (Float32,
fraction 0..1), /sim/kill (Empty).
"""
import math

import rclpy
from px4_msgs.msg import (
    BatteryStatus, EstimatorStatusFlags, OffboardControlMode, TrajectorySetpoint,
    VehicleCommand, VehicleControlMode, VehicleLocalPosition, VehicleOdometry,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Empty, Float32, String

DT = 0.02
POSCTL, OFFBOARD, LAND = 'POSCTL', 'OFFBOARD', 'AUTO_LAND'
NAV = {POSCTL: VehicleStatus.NAVIGATION_STATE_POSCTL,
       OFFBOARD: VehicleStatus.NAVIGATION_STATE_OFFBOARD,
       LAND: VehicleStatus.NAVIGATION_STATE_AUTO_LAND}


class FakePx4(Node):
    def __init__(self):
        super().__init__('fake_px4')
        self.vmax_xy = float(self.declare_parameter('max_speed_xy', 1.0).value)
        self.vmax_z = float(self.declare_parameter('max_speed_z', 0.5).value)
        self.tau = float(self.declare_parameter('velocity_tau_s', 0.3).value)
        self.kp = float(self.declare_parameter('position_gain', 1.5).value)
        self.pos = [0.0, 0.0, 0.0]          # NED
        self.vel = [0.0, 0.0, 0.0]
        self.yaw = math.pi / 2
        self.armed = False
        self.mode = POSCTL
        self.hold = None                    # POSCTL hold point
        self.sp = None                      # last TrajectorySetpoint
        self.sp_t = None
        self.heartbeat_t = None
        self.battery = 0.9
        self.stray_setpoints = 0
        self.late_setpoints = 0
        self.left_offboard_t = None

        q = qos_profile_sensor_data
        mk = self.create_publisher
        self.odom_pub = mk(VehicleOdometry, '/fmu/out/vehicle_odometry', q)
        self.mode_pub = mk(VehicleControlMode, '/fmu/out/vehicle_control_mode', q)
        self.flags_pub = mk(EstimatorStatusFlags, '/fmu/out/estimator_status_flags', q)
        self.lpos_pub = mk(VehicleLocalPosition, '/fmu/out/vehicle_local_position', q)
        self.status_pub = mk(VehicleStatus, '/fmu/out/vehicle_status', q)
        self.batt_pub = mk(BatteryStatus, '/fmu/out/battery_status', q)
        self.state_pub = mk(String, '/sim/px4_state', 10)
        sub = self.create_subscription
        sub(OffboardControlMode, '/fmu/in/offboard_control_mode', self.on_heartbeat, q)
        sub(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self.on_setpoint, q)
        sub(VehicleCommand, '/fmu/in/vehicle_command', self.on_command, q)
        sub(Empty, '/sim/pilot_takeover', lambda _m: self.pilot_takeover(), 10)
        sub(Empty, '/sim/kill', lambda _m: self.disarm('kill switch'), 10)
        sub(Float32, '/sim/battery', lambda m: setattr(self, 'battery', float(m.data)), 10)
        self.n = 0
        self.create_timer(DT, self.step)
        self.get_logger().info('fake PX4 up (kinematic; not a flight simulator)')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @property
    def in_air(self):
        return bool(self.pos[2] < -0.05)

    # ----------------------------------------------------------- inputs
    def on_heartbeat(self, _msg):
        self.heartbeat_t = self.now()

    def on_setpoint(self, msg):
        if self.mode != OFFBOARD:
            if self.left_offboard_t is not None and self.now() - self.left_offboard_t < 0.25:
                self.late_setpoints += 1
                return
            self.stray_setpoints += 1
            if self.stray_setpoints in (1, 50, 500):
                self.get_logger().error(
                    f'setpoint received outside Offboard (mode {self.mode}); '
                    f'{self.stray_setpoints} so far. Real PX4 would mix this into the mode.')
            return
        self.sp, self.sp_t = msg, self.now()

    def on_command(self, msg):
        c = msg.command
        if c == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM:
            if msg.param1 >= 0.5:
                self.armed = True
                self.get_logger().info('ARMED')
            else:
                self.disarm('command')
        elif c == VehicleCommand.VEHICLE_CMD_DO_SET_MODE and int(msg.param2) == 6:
            if self.heartbeat_t is not None and self.now() - self.heartbeat_t < 0.5:
                self.set_mode(OFFBOARD)
                self.sp = None
            else:
                self.get_logger().warn('Offboard refused: no fresh offboard_control_mode')
        elif c == VehicleCommand.VEHICLE_CMD_NAV_LAND:
            if self.armed:
                self.set_mode(LAND)

    def pilot_takeover(self):
        self.set_mode(POSCTL)

    def set_mode(self, mode):
        if mode != self.mode:
            self.get_logger().info(f'mode {self.mode} -> {mode}')
            if self.mode == OFFBOARD:
                self.left_offboard_t = self.now()
            self.mode = mode
            self.hold = list(self.pos)
            self.publish_modes(int(self.now() * 1e6))    # PX4 publishes on change

    def disarm(self, why):
        if self.armed:
            self.get_logger().info(f'DISARMED ({why})')
        self.armed = False
        self.publish_modes(int(self.now() * 1e6))
        self.vel = [0.0, 0.0, 0.0]
        if self.mode != POSCTL:
            self.set_mode(POSCTL)

    # --------------------------------------------------------- dynamics
    def target_velocity(self):
        def towards(p):
            return [self.kp * (p[k] - self.pos[k]) for k in range(3)]
        if not self.armed:
            return [0.0, 0.0, 0.0]
        if self.mode == LAND:
            return [0.0, 0.0, 0.4]
        if self.mode == POSCTL:
            return towards(self.hold) if self.in_air else [0.0, 0.0, 0.0]
        sp = self.sp
        if sp is None or self.now() - self.sp_t > 0.5:
            return towards(self.hold) if self.in_air else [0.0, 0.0, 0.0]
        if all(math.isfinite(v) for v in sp.position):
            v = towards([float(x) for x in sp.position])
        elif all(math.isfinite(v) for v in sp.velocity):
            v = [float(x) for x in sp.velocity]
            if not self.in_air and v[2] >= 0.0:
                v = [0.0, 0.0, 0.0]          # zero velocity on the ground: no takeoff
        else:
            v = [0.0, 0.0, 0.0]
        if math.isfinite(sp.yaw):
            err = (sp.yaw - self.yaw + math.pi) % (2 * math.pi) - math.pi
            self.yaw += max(-1.0, min(1.0, 2.0 * err)) * DT
        elif math.isfinite(sp.yawspeed):
            self.yaw += sp.yawspeed * DT
        return v

    def step(self):
        now = self.now()
        if self.mode == OFFBOARD and (self.heartbeat_t is None or now - self.heartbeat_t > 0.5):
            self.get_logger().warn('offboard heartbeat lost')
            self.set_mode(POSCTL)
        v = self.target_velocity()
        n_xy = math.hypot(v[0], v[1])
        if n_xy > self.vmax_xy:
            v[0], v[1] = v[0] / n_xy * self.vmax_xy, v[1] / n_xy * self.vmax_xy
        v[2] = max(-self.vmax_z, min(self.vmax_z, v[2]))
        a = DT / self.tau
        for k in range(3):
            self.vel[k] += a * (v[k] - self.vel[k])
            self.pos[k] += self.vel[k] * DT
        if self.pos[2] >= 0.0:                # on the ground
            self.pos[2] = 0.0
            self.vel[2] = min(self.vel[2], 0.0)
            if self.mode == LAND:
                self.disarm('landed')
        self.publish(now)

    def publish_modes(self, t):
        cm = VehicleControlMode()
        cm.timestamp = t
        cm.flag_armed = self.armed
        cm.flag_control_offboard_enabled = self.mode == OFFBOARD
        self.mode_pub.publish(cm)
        st = VehicleStatus()
        st.timestamp = t
        st.arming_state = (VehicleStatus.ARMING_STATE_ARMED if self.armed
                           else VehicleStatus.ARMING_STATE_DISARMED)
        st.nav_state = NAV[self.mode]
        self.status_pub.publish(st)

    def publish(self, now):
        t = int(now * 1e6)
        od = VehicleOdometry()
        od.timestamp = od.timestamp_sample = t
        od.pose_frame = VehicleOdometry.POSE_FRAME_NED
        od.position = [float(v) for v in self.pos]
        od.q = [math.cos(self.yaw / 2), 0.0, 0.0, math.sin(self.yaw / 2)]
        od.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        od.velocity = [float(v) for v in self.vel]
        od.angular_velocity = [0.0, 0.0, 0.0]
        self.odom_pub.publish(od)
        self.n += 1
        if self.n % 5:
            return
        self.publish_modes(t)
        fl = EstimatorStatusFlags()
        fl.timestamp = t
        fl.cs_in_air = self.in_air
        self.flags_pub.publish(fl)
        lp = VehicleLocalPosition()
        lp.timestamp = t
        lp.xy_valid = lp.z_valid = lp.v_xy_valid = lp.v_z_valid = True
        lp.x, lp.y, lp.z = (float(v) for v in self.pos)
        lp.vx, lp.vy, lp.vz = (float(v) for v in self.vel)
        lp.heading = float(self.yaw)
        self.lpos_pub.publish(lp)
        b = BatteryStatus()
        b.timestamp = t
        b.connected = True
        b.remaining = float(self.battery)
        b.voltage_v = 19.0 + 6.0 * float(self.battery)
        b.cell_count = 6
        self.batt_pub.publish(b)
        self.state_pub.publish(String(data=(
            f'{self.mode} armed={self.armed} in_air={self.in_air} '
            f'ned=({self.pos[0]:.2f},{self.pos[1]:.2f},{self.pos[2]:.2f}) '
            f'stray_setpoints={self.stray_setpoints} late_setpoints={self.late_setpoints}')))


def main():
    rclpy.init()
    node = FakePx4()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
