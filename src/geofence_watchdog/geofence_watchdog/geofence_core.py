# filepath: src/geofence_watchdog/geofence_watchdog/geofence_core.py
"""Pure logic for the geofence watchdog: box math, breach state machine with
hysteresis, and message-timeout tracking. No ROS imports — everything here is
unit-testable on a bare Python interpreter (masterplan WP-E: zero verification
debt allowed).

Frame convention [FIXED]: the box is axis-aligned in the PX4 local NED frame,
the same frame as /fmu/out/vehicle_local_position. z is DOWN-positive: a 2.5 m
flight ceiling is z_min = -2.5, and the floor bound sits at small positive z.
"""
from enum import IntEnum


class FenceState(IntEnum):
    OK = 0
    SOFT = 1    # inside the box but within soft_margin of a face
    HARD = 2    # outside the box (latched once entered)


class GeofenceBox:
    """Axis-aligned NED box with a soft inner margin.

    classify() is stateless geometry; hysteresis lives in BreachStateMachine.
    """

    def __init__(self, x_min, x_max, y_min, y_max, z_min, z_max,
                 soft_margin=0.5):
        if not (x_min < x_max and y_min < y_max and z_min < z_max):
            raise ValueError('geofence box min must be < max on every axis')
        if soft_margin < 0.0:
            raise ValueError('soft_margin must be >= 0')
        half_min = min(x_max - x_min, y_max - y_min, z_max - z_min) / 2.0
        if soft_margin >= half_min:
            raise ValueError(
                f'soft_margin {soft_margin} leaves no OK interior '
                f'(smallest half-extent is {half_min})')
        self.x_min, self.x_max = x_min, x_max
        self.y_min, self.y_max = y_min, y_max
        self.z_min, self.z_max = z_min, z_max
        self.soft_margin = soft_margin

    def margin_to_boundary(self, x, y, z):
        """Smallest distance to any face; negative when outside the box."""
        return min(
            x - self.x_min, self.x_max - x,
            y - self.y_min, self.y_max - y,
            z - self.z_min, self.z_max - z,
        )

    def classify(self, x, y, z):
        m = self.margin_to_boundary(x, y, z)
        if m < 0.0:
            return FenceState.HARD
        if m < self.soft_margin:
            return FenceState.SOFT
        return FenceState.OK


class BreachStateMachine:
    """Adds hysteresis and hard-latch on top of GeofenceBox.classify().

    - OK -> SOFT the instant the position enters the margin band.
    - SOFT -> OK only after the position is deeper inside than
      soft_margin + hysteresis (prevents alert flapping on the band edge).
    - Any -> HARD the instant the position leaves the box. HARD latches:
      the watchdog has commanded a land; only a node restart clears it.
    """

    def __init__(self, box: GeofenceBox, hysteresis=0.2):
        if hysteresis < 0.0:
            raise ValueError('hysteresis must be >= 0')
        self.box = box
        self.hysteresis = hysteresis
        self.state = FenceState.OK

    def update(self, x, y, z) -> FenceState:
        if self.state == FenceState.HARD:
            return self.state  # latched
        m = self.box.margin_to_boundary(x, y, z)
        if m < 0.0:
            self.state = FenceState.HARD
        elif self.state == FenceState.SOFT:
            if m >= self.box.soft_margin + self.hysteresis:
                self.state = FenceState.OK
        else:  # OK
            if m < self.box.soft_margin:
                self.state = FenceState.SOFT
        return self.state


class TimeoutMonitor:
    """Tracks the age of a message stream against a timeout.

    Starts 'never received': expired() is False until the first beat(), so a
    watchdog booted before the pipeline does not instantly trip; the node
    layer decides how long 'never received' is tolerable via require_after_s.
    """

    def __init__(self, timeout_s, require_after_s=None):
        if timeout_s <= 0.0:
            raise ValueError('timeout_s must be > 0')
        self.timeout_s = timeout_s
        self.require_after_s = require_after_s
        self._last = None
        self._start = None

    def start(self, now_s):
        """Record watchdog start time (for require_after_s grace)."""
        self._start = now_s

    def beat(self, now_s):
        self._last = now_s

    def expired(self, now_s) -> bool:
        if self._last is not None:
            return (now_s - self._last) > self.timeout_s
        # never received:
        if self.require_after_s is None or self._start is None:
            return False
        return (now_s - self._start) > self.require_after_s


class Action(IntEnum):
    NONE = 0
    ALERT_HOVER = 1   # soft breach: alert + request hover from behavior layer
    LAND = 2          # hard action: direct NAV_LAND via /fmu/in/vehicle_command


def decide(fence_state: FenceState, estimator_valid: bool,
           lio_silent: bool, estimate_silent: bool,
           bridge_bad: bool = False) -> Action:
    """The action ladder from masterplan §3 [FIXED]:
    hard breach OR estimator invalid OR LIO silence -> LAND;
    soft breach -> ALERT_HOVER; otherwise nothing.
    estimate_silent (vehicle_local_position stream dead) also lands: a
    watchdog that has lost the estimate it polices must fail safe.
    bridge_bad (operator decision 2026-11-16): px4_odom_bridge reports BLOCKED
    or STALE, or its state topic has gone silent. External vision has stopped
    reaching EKF2 although FAST-LIO may still be publishing, so /Odometry
    liveness alone does not show it. Land while the estimate is still good.
    """
    if (fence_state == FenceState.HARD or not estimator_valid
            or lio_silent or estimate_silent or bridge_bad):
        return Action.LAND
    if fence_state == FenceState.SOFT:
        return Action.ALERT_HOVER
    return Action.NONE


# px4_odom_bridge guard states on /px4_odom_bridge/state (std_msgs/UInt8).
BRIDGE_WAITING = 0
BRIDGE_FORWARDING = 1
BRIDGE_STALE = 2
BRIDGE_BLOCKED = 3


def bridge_state_bad(state) -> bool:
    """True for a bridge state that means EV has stopped after it had started.

    None (never received) and WAITING are not failures here: before the first
    LIO sample nothing has been lost. Never-received is covered by the node's
    TimeoutMonitor grace (require_streams_after_s). An unknown code is bad.
    """
    if state is None or state in (BRIDGE_WAITING, BRIDGE_FORWARDING):
        return False
    return True


class LandCommander:
    """Decides when a land command goes out (operator decision 2026-11-16).

    - Nothing is sent to a vehicle known to be disarmed. Unknown arming state
      (no vehicle_status yet) counts as armed: failing to land an airborne
      vehicle is worse than a spare command on the bench.
    - The command is repeated every resend_period_s until PX4 reports the Land
      mode once after a command of ours. From then on this object never sends
      again: PX4 has the order, and a pilot who then selects another mode is
      taking over deliberately and must not be overridden a second later.
    """

    def __init__(self, resend_period_s=1.0):
        if resend_period_s <= 0.0:
            raise ValueError('resend_period_s must be > 0')
        self.resend_period_s = resend_period_s
        self.last_sent_s = None
        self.confirmed = False

    def step(self, now_s, want_land: bool, armed, nav_is_land: bool) -> bool:
        """Return True when a land command must be sent now.

        armed: True, False, or None when unknown.
        """
        if self.last_sent_s is not None and nav_is_land:
            self.confirmed = True
        if not want_land or self.confirmed or armed is False:
            return False
        if (self.last_sent_s is None
                or now_s - self.last_sent_s >= self.resend_period_s):
            self.last_sent_s = now_s
            return True
        return False
