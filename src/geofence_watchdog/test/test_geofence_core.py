# filepath: src/geofence_watchdog/test/test_geofence_core.py
"""WP-E unit tests: box math, hysteresis, timeout logic, action ladder.
Pure Python, no ROS runtime required (masterplan §3: zero verification debt)."""
import pytest

from geofence_watchdog.geofence_core import (
    GeofenceBox, BreachStateMachine, TimeoutMonitor, FenceState, Action, decide,
)


def make_box(**kw):
    d = dict(x_min=-2.0, x_max=2.0, y_min=-2.0, y_max=2.0,
             z_min=-2.5, z_max=0.3, soft_margin=0.5)
    d.update(kw)
    return GeofenceBox(**d)


# ---------------------------------------------------------------- box math
class TestBoxMath:
    def test_center_is_ok(self):
        assert make_box().classify(0.0, 0.0, -1.0) == FenceState.OK

    @pytest.mark.parametrize('pos', [
        (2.5, 0, -1), (-2.5, 0, -1), (0, 2.5, -1), (0, -2.5, -1),
        (0, 0, -3.0), (0, 0, 0.5),
    ])
    def test_outside_each_face_is_hard(self, pos):
        assert make_box().classify(*pos) == FenceState.HARD

    @pytest.mark.parametrize('pos', [
        (1.8, 0, -1), (-1.8, 0, -1), (0, 1.8, -1), (0, -1.8, -1),
        (0, 0, -2.2), (0, 0, 0.0),
    ])
    def test_margin_band_each_face_is_soft(self, pos):
        assert make_box().classify(*pos) == FenceState.SOFT

    def test_exactly_on_boundary_is_soft_not_hard(self):
        # margin == 0: not negative, so inside-with-zero-margin -> SOFT.
        assert make_box().classify(2.0, 0.0, -1.0) == FenceState.SOFT

    def test_exactly_on_soft_edge_is_ok(self):
        # margin == soft_margin exactly -> OK (strict < comparison).
        assert make_box().classify(1.5, 0.0, -1.0) == FenceState.OK

    def test_ned_z_semantics_ceiling(self):
        """Flying HIGHER than the 2.5 m ceiling means z MORE NEGATIVE than
        z_min. This is the sign error the spec's Gate 3 test hunts for."""
        box = make_box()
        assert box.classify(0, 0, -2.6) == FenceState.HARD   # above ceiling
        assert box.classify(0, 0, -1.0) == FenceState.OK      # normal hover

    def test_corner_uses_min_margin(self):
        # 1.8 on x AND y: margin 0.2 -> SOFT even though each alone is SOFT
        assert make_box().classify(1.8, 1.8, -1.0) == FenceState.SOFT
        assert make_box().classify(2.1, 2.1, -1.0) == FenceState.HARD

    def test_margin_to_boundary_values(self):
        box = make_box()
        assert box.margin_to_boundary(0, 0, -1.0) == pytest.approx(1.3)  # z_max face: 0.3-(-1)=1.3
        assert box.margin_to_boundary(2.5, 0, -1.0) == pytest.approx(-0.5)

    def test_invalid_box_rejected(self):
        with pytest.raises(ValueError):
            make_box(x_min=2.0, x_max=-2.0)
        with pytest.raises(ValueError):
            make_box(soft_margin=-0.1)

    def test_margin_swallowing_interior_rejected(self):
        # z extent 2.8 -> half 1.4; margin 1.4 leaves no OK interior.
        with pytest.raises(ValueError):
            make_box(soft_margin=1.4)


# --------------------------------------------------------------- hysteresis
class TestHysteresis:
    def test_soft_entry_immediate(self):
        sm = BreachStateMachine(make_box(), hysteresis=0.2)
        assert sm.update(0, 0, -1) == FenceState.OK
        assert sm.update(1.8, 0, -1) == FenceState.SOFT

    def test_no_flapping_at_band_edge(self):
        """Oscillating +/-1 cm across the soft boundary (x=1.5 for this box)
        must produce ONE transition to SOFT, then hold."""
        sm = BreachStateMachine(make_box(), hysteresis=0.2)
        sm.update(1.49, 0, -1)          # margin 0.51: OK side of band edge
        assert sm.state == FenceState.OK
        sm.update(1.51, 0, -1)          # margin 0.49: entered band -> SOFT
        assert sm.state == FenceState.SOFT
        sm.update(1.49, 0, -1)          # 1 cm back out: 0.51 < 0.7, holds SOFT
        assert sm.state == FenceState.SOFT
        sm.update(1.51, 0, -1)
        assert sm.state == FenceState.SOFT

    def test_recovery_requires_hysteresis_depth(self):
        sm = BreachStateMachine(make_box(), hysteresis=0.2)
        sm.update(1.8, 0, -1)                    # SOFT (margin 0.2)
        sm.update(1.4, 0, -1)                    # margin 0.6 < 0.5+0.2
        assert sm.state == FenceState.SOFT
        sm.update(1.29, 0, -1)                   # margin 0.71 > 0.7 -> clears
        assert sm.state == FenceState.OK

    def test_zero_hysteresis_recovers_at_band_edge(self):
        sm = BreachStateMachine(make_box(), hysteresis=0.0)
        sm.update(1.8, 0, -1)
        assert sm.state == FenceState.SOFT
        sm.update(1.49, 0, -1)                   # margin 0.51 >= 0.5
        assert sm.state == FenceState.OK

    def test_hard_latches(self):
        sm = BreachStateMachine(make_box(), hysteresis=0.2)
        assert sm.update(2.5, 0, -1) == FenceState.HARD
        assert sm.update(0, 0, -1) == FenceState.HARD    # back inside: latched
        assert sm.update(0, 0, -1) == FenceState.HARD

    def test_soft_to_hard_direct(self):
        sm = BreachStateMachine(make_box())
        sm.update(1.8, 0, -1)
        assert sm.state == FenceState.SOFT
        sm.update(2.5, 0, -1)
        assert sm.state == FenceState.HARD

    def test_negative_hysteresis_rejected(self):
        with pytest.raises(ValueError):
            BreachStateMachine(make_box(), hysteresis=-0.1)


# ------------------------------------------------------------------ timeout
class TestTimeoutMonitor:
    def test_fresh_stream_not_expired(self):
        m = TimeoutMonitor(1.0)
        m.beat(10.0)
        assert not m.expired(10.5)
        assert not m.expired(11.0)      # exactly at timeout: not > timeout

    def test_stale_stream_expired(self):
        m = TimeoutMonitor(1.0)
        m.beat(10.0)
        assert m.expired(11.01)

    def test_beat_resets(self):
        m = TimeoutMonitor(1.0)
        m.beat(10.0)
        assert m.expired(11.5)
        m.beat(11.5)
        assert not m.expired(12.0)

    def test_never_received_default_never_expires(self):
        m = TimeoutMonitor(1.0)
        m.start(0.0)
        assert not m.expired(9999.0)

    def test_never_received_with_grace_expires(self):
        m = TimeoutMonitor(1.0, require_after_s=5.0)
        m.start(0.0)
        assert not m.expired(4.9)
        assert m.expired(5.1)

    def test_grace_satisfied_by_first_beat(self):
        m = TimeoutMonitor(1.0, require_after_s=5.0)
        m.start(0.0)
        m.beat(4.0)
        assert not m.expired(4.9)
        assert m.expired(5.1)           # now governed by timeout, 4.0+1.0

    def test_invalid_timeout_rejected(self):
        with pytest.raises(ValueError):
            TimeoutMonitor(0.0)


# ------------------------------------------------------------ action ladder
class TestActionLadder:
    def test_nominal_no_action(self):
        assert decide(FenceState.OK, True, False, False) == Action.NONE

    def test_soft_breach_alerts_and_hovers(self):
        assert decide(FenceState.SOFT, True, False, False) == Action.ALERT_HOVER

    def test_hard_breach_lands(self):
        assert decide(FenceState.HARD, True, False, False) == Action.LAND

    def test_estimator_invalid_lands_even_inside(self):
        assert decide(FenceState.OK, False, False, False) == Action.LAND

    def test_lio_silence_lands_even_inside(self):
        assert decide(FenceState.OK, True, True, False) == Action.LAND

    def test_estimate_silence_lands(self):
        assert decide(FenceState.OK, True, False, True) == Action.LAND

    def test_land_dominates_soft(self):
        assert decide(FenceState.SOFT, True, True, False) == Action.LAND
        assert decide(FenceState.SOFT, False, False, False) == Action.LAND

    def test_all_failures_at_once_lands(self):
        assert decide(FenceState.HARD, False, True, True) == Action.LAND
