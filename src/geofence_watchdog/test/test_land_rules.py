# filepath: src/geofence_watchdog/test/test_land_rules.py
"""Land-command rules and the bridge-state input (operator decisions
2026-11-16). Pure logic, no ROS."""
import itertools

import pytest

from geofence_watchdog.geofence_core import (
    Action, FenceState, LandCommander, decide, bridge_state_bad,
    BRIDGE_WAITING, BRIDGE_FORWARDING, BRIDGE_STALE, BRIDGE_BLOCKED,
)


class TestBridgeInLadder:
    def test_bridge_bad_alone_lands(self):
        assert decide(FenceState.OK, True, False, False, True) == Action.LAND

    def test_default_keeps_the_old_ladder(self):
        assert decide(FenceState.OK, True, False, False) == Action.NONE
        assert decide(FenceState.SOFT, True, False, False) == Action.ALERT_HOVER

    def test_full_truth_table(self):
        for fence, est, lio, silent, bridge in itertools.product(
                FenceState, (True, False), (True, False), (True, False),
                (True, False)):
            hard = (fence == FenceState.HARD or not est or lio or silent
                    or bridge)
            want = (Action.LAND if hard else
                    Action.ALERT_HOVER if fence == FenceState.SOFT
                    else Action.NONE)
            assert decide(fence, est, lio, silent, bridge) == want

    @pytest.mark.parametrize('state,bad', [
        (None, False), (BRIDGE_WAITING, False), (BRIDGE_FORWARDING, False),
        (BRIDGE_STALE, True), (BRIDGE_BLOCKED, True), (7, True), (255, True)])
    def test_state_classification(self, state, bad):
        assert bridge_state_bad(state) is bad


class TestLandCommander:
    def test_nothing_without_a_land_decision(self):
        c = LandCommander(1.0)
        assert not c.step(0.0, False, True, False)
        assert c.last_sent_s is None

    def test_sends_at_once_then_rate_limited(self):
        c = LandCommander(1.0)
        assert c.step(10.0, True, True, False)
        assert not c.step(10.5, True, True, False)
        assert not c.step(10.99, True, True, False)
        assert c.step(11.0, True, True, False)

    def test_disarmed_vehicle_gets_no_command(self):
        c = LandCommander(1.0)
        for t in range(5):
            assert not c.step(float(t), True, False, False)
        assert c.last_sent_s is None

    def test_unknown_arming_state_counts_as_armed(self):
        assert LandCommander(1.0).step(0.0, True, None, False)

    def test_arming_later_starts_the_commands(self):
        c = LandCommander(1.0)
        assert not c.step(0.0, True, False, False)
        assert c.step(0.1, True, True, False)

    def test_stops_for_good_once_px4_reports_land(self):
        c = LandCommander(1.0)
        assert c.step(0.0, True, True, False)
        assert not c.step(1.0, True, True, True)      # PX4 is in Land
        assert c.confirmed
        # Pilot takes over: Land is left, the hard condition persists.
        for t in (2.0, 3.0, 10.0, 100.0):
            assert not c.step(t, True, True, False)

    def test_land_mode_before_any_command_is_not_a_confirmation(self):
        # The pilot's own Land must not disarm the watchdog's hard action.
        c = LandCommander(1.0)
        assert not c.step(0.0, False, True, True)
        assert not c.confirmed
        assert c.step(5.0, True, True, False)

    def test_keeps_sending_while_px4_never_confirms(self):
        c = LandCommander(1.0)
        sent = sum(c.step(float(t), True, True, False) for t in range(10))
        assert sent == 10

    def test_bad_period_rejected(self):
        with pytest.raises(ValueError):
            LandCommander(0.0)
