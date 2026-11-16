# filepath: src/px4_odom_bridge/test/test_ev_guard_replay.py
"""Replay tests against the real 2026-09-14 incident.

Fixtures in test/data/ are the actual external-vision stream PX4 received, taken
from the flight logs (see each file's header line). These are regression tests:
the guard must forward every healthy sample, and must block the LiDAR power cut
at the moment it happened.
"""
import csv
import math
import os

from px4_odom_bridge.ev_guard import EvGuard, GuardConfig, State

DATA = os.path.join(os.path.dirname(__file__), 'data')
CFG = GuardConfig()          # the shipped defaults, not test-tuned values


def load(name):
    with open(os.path.join(DATA, name)) as f:
        rows = [r for r in csv.reader(f) if not r[0].startswith('#')]
    header, rows = rows[0], rows[1:]
    assert header == ['t_s', 'n', 'e', 'd', 'qw', 'qx', 'qy', 'qz']
    return [(float(r[0]), tuple(float(v) for v in r[1:4]), tuple(float(v) for v in r[4:8]))
            for r in rows]


def run(rows, cfg=CFG):
    g = EvGuard(cfg)
    blocked_at = None
    for t, p, q in rows:
        d = g.check(t, p, q)
        if not d.accept and blocked_at is None:
            blocked_at = (t, d.reason)
    return g, blocked_at


class TestHealthyStreamIsUntouched:
    """No false positives: 400 s of real bench traffic including hand carries."""

    def test_every_sample_forwarded(self):
        rows = load('healthy_ev.csv')
        assert len(rows) > 3000
        g, blocked_at = run(rows)
        assert blocked_at is None, f'false positive: {blocked_at}'
        assert g.state is State.FORWARDING
        assert g.counts.forwarded == len(rows)
        assert g.counts.rejected == 0

    def test_carries_keep_a_wide_margin_to_the_thresholds(self):
        """Measured headroom, asserted directly: a counter that stays 0 proves nothing.

        Over this 400 s of bench traffic (stationary, hand carries, the lift onto
        the bench) the largest 10 Hz step is 1.52 cm and the largest gap 0.144 s.
        That is ~20x margin to max_step_m and ~7x to max_gap_s, which is why the
        stream survives without a single warn. If a future fixture or a faster
        vehicle erodes this margin, this test fails and the thresholds get
        re-derived rather than silently drifting toward false positives.
        """
        rows = load('healthy_ev.csv')
        steps = [math.dist(rows[i][1], rows[i - 1][1]) for i in range(1, len(rows))]
        gaps = [rows[i][0] - rows[i - 1][0] for i in range(1, len(rows))]
        assert max(steps) < 0.5 * CFG.warn_step_m, f'largest step {max(steps):.3f} m'
        assert max(gaps) < 0.5 * CFG.max_gap_s, f'largest gap {max(gaps):.3f} s'
        g, _ = run(rows)
        assert g.counts.warned == 0


class TestPowerCutIsBlocked:
    """The incident: a 12.15 s gap at t+415 s, then a 3.26 m jump."""

    def test_blocks_at_the_outage(self):
        rows = load('outage_ev.csv')
        g, blocked_at = run(rows)
        assert blocked_at is not None
        t, reason = blocked_at
        assert 410 < t < 420, f'blocked at t+{t:.1f}s, expected the outage at ~t+415s'
        assert 'gap' in reason or 'jumped' in reason

    def test_nothing_is_forwarded_after_the_outage(self):
        rows = load('outage_ev.csv')
        g = EvGuard(CFG)
        blocked = False
        after = 0
        for t, p, q in rows:
            d = g.check(t, p, q)
            blocked = blocked or g.state is State.BLOCKED
            if blocked and d.accept:
                after += 1
        # `blocked` is asserted too: without it the loop body never runs on a
        # guard that fails open, and the test passes vacuously.
        assert blocked, 'the outage was never blocked'
        assert after == 0
        assert g.counts.rejected > 1000

    def test_the_runaway_never_reaches_px4(self):
        """Without the guard, EKF2 saw 1e5 m. With it, the last forwarded sample is sane."""
        rows = load('outage_ev.csv')
        g = EvGuard(CFG)
        last_ok = None
        for t, p, q in rows:
            if g.check(t, p, q).accept:
                last_ok = p
        assert max(abs(v) for v in last_ok) < CFG.max_radius_m


class TestDivergedStreamIsBlockedImmediately:
    def test_first_sample_refused(self):
        rows = load('diverged_ev.csv')
        g = EvGuard(CFG)
        d = g.check(*rows[0])
        assert not d.accept and 'max_radius_m' in d.reason
        assert g.counts.forwarded == 0


class TestGuardWouldHaveChangedTheOutcome:
    """Mutation check in test form: an unguarded pass-through forwards the garbage."""

    def test_unguarded_passthrough_would_forward_the_runaway(self):
        rows = load('outage_ev.csv')
        worst = max(max(abs(v) for v in p) for _, p, _ in rows)
        assert worst > 50, 'fixture no longer contains the runaway'
        g, blocked_at = run(rows)
        assert blocked_at is not None and blocked_at[0] < 420
