<!-- filepath: docs/px4_odom_bridge_ev_guard_spec.md -->
# Implementation Spec: external-vision sanity guard in `px4_odom_bridge`

| | |
|---|---|
| Status | **IMPLEMENTED on `main`, bench run outstanding (VD-011).** Steps 1–5 and the mutation check of §6 are done: 68 tests pass (`src/px4_odom_bridge/test/`). Step 6 needs the vehicle. |
| Date | 2026-09-14 |
| Package | `src/px4_odom_bridge` (on `main`) |
| Motivation | The Gate 3 incident of 2026-09-14: a LiDAR power cut corrupted FAST-LIO, the bridge forwarded the diverged estimate, and EKF2 destroyed its own state (`docs/gate3_verification_2026-09-14.md` §10) |
| Governing rules | Masterplan §1 (build now, integrate on tokens), §1.3 (verification debt), Gates 1–4 handover spec global rules |

## 0. Why this exists

On the bench, disarmed, the Mid-360 lost power. The sequence, measured from
`log_2_2026-9-14-17-57-14.ulg` and the agent's parallel capture:

| Time | EV stream |
|---|---|
| t+415 s | 12.15 s gap (the outage), then the first sample after it is **3.26 m** from the last |
| t+415–515 s | samples look normal (steps ≤ 62 mm) but FAST-LIO's map is corrupted |
| t+520 s | magnitude 11 m, creeping at ~1.6 m/s — a plausible walking speed |
| t+526 s | 15 m · t+560 s: 75 m · later: 10^5 m |
| throughout | EKF2 rejected the data (test ratios ~10^4) and then **reset position to vision 123 times**, dragging its estimate with it |

Three facts drive the design:
1. **EKF2 cannot protect itself.** Sustained rejection ends in a reset *to* the bad measurement.
2. **The runaway was gradual.** A speed or jump limit alone would not have caught the creep.
3. **The root cause was visible immediately**: a long gap followed by a large jump, 100 s before any harm.

## 1. Scope

**In scope.** A guard between `/Odometry` and `/fmu/in/vehicle_visual_odometry` that refuses to forward
untrustworthy samples, and stops forwarding entirely once trust is lost.

**Non-goals [FIXED].**
- No commanding: the bridge never publishes to `/fmu/in/vehicle_command`, never arms, lands or changes
  modes. The geofence watchdog (WP-E) owns actions.
- No estimation: no filtering, smoothing or correction of LIO output. Only accept or reject.
- Not a replacement for `lio_health_guard` (WP-D), which detects slow degradation and drives the hover
  path. This guard is the last-resort containment on the data path itself.
- No automatic recovery in flight (see §4, rule G6): a restarted FAST-LIO has a **new origin**, and
  resuming would hand EKF2 a step change.

## 2. Requirements

| ID | Requirement |
|---|---|
| R1 | Never forward non-finite position or orientation |
| R2 | Never forward a sample whose position jumps more than `max_step_m` from the last accepted one |
| R3 | Never forward a position whose magnitude exceeds `max_radius_m` |
| R4 | Never forward a sample whose orientation is not a unit quaternion within `quat_tolerance` |
| R5 | Never forward the first sample after a data gap longer than `max_gap_s` |
| R6 | Once trust is lost, stop forwarding permanently for that process: recovery requires a restart |
| R7 | Keep the existing stamp-age rule (0…`max_stamp_age_s`), which already protects against clock skew |
| R8 | Report state continuously: log transitions once, and publish a status topic for the operator and, later, for `lio_health_guard` and the watchdog |
| R9 | The decision logic contains no ROS types and is exhaustively unit tested |
| R10 | Never delay a good sample: the checks are arithmetic on one sample, no buffering |
| R11 | Parameters validated at startup; an invalid value refuses to start rather than running with a guess |

## 3. Thresholds, grounded in measured data

Healthy traffic, 470 s of bench operation including hand carries (`log_2`, before the outage):

| Quantity | Median | p99 | p99.9 | Max |
|---|---|---|---|---|
| Sample interval | 100.0 ms | 117 ms | 131 ms | — |
| Step between samples | 3.67 mm | 62.9 mm | 104 mm | — |
| Position magnitude | — | — | — | 3.31 m |

| Parameter | Default | Justification |
|---|---|---|
| `max_step_m` | **0.30** | 3× the healthy p99.9 (104 mm); catches the 3.26 m outage jump by 10×. At 10 Hz this is 3 m/s, well above any hand carry and above the Gate 4 envelope (`MPC_XY_VEL_MAX` 1.0 m/s) |
| `max_gap_s` | **1.0** | 7.6× the healthy p99.9 interval (131 ms); the outage gap was 12.15 s. Matches the watchdog's 1.0 s LIO-silence rule |
| `max_radius_m` | **20.0** (operator, 2026-09-14; was 5.0 at first implementation, 15.0 as originally proposed) | Backstop for gradual runaway. Bench used ≤ 3.31 m; a venue is ≤ 10 m. At 20 m it cannot latch during normal indoor operation while still catching the 1e4–1e6 m runaway by three orders of magnitude. Would have fired at t+526 s even if every other rule failed |
| `max_stamp_age_s` | 0.5 | Unchanged, already in the bridge |
| `quat_tolerance` | 0.01 | Norm within 1% of unity |
| `warn_step_m` | 0.10 | Below the reject threshold: counted and logged, not rejected. Gives early warning of degradation without acting on it |

Every threshold is a ROS parameter; the defaults above are what the data supports.

## 4. Design

A pure-Python core (`ev_guard.py`, no ROS imports) plus wiring in `odom_bridge_node.py`.

### 4.1 States
| State | Meaning | Publishes EV? |
|---|---|---|
| `WAITING` | No accepted sample yet | no |
| `FORWARDING` | Healthy | yes |
| `BLOCKED` | Trust lost, latched for the life of the process | **never again** |
| `STALE` | *Reported only* (added 2026-10-13): `FORWARDING` with no accepted sample for longer than `max_gap_s`, judged against the node's clock when the status is published. Not stored, so it never changes what `check()` decides | no (nothing is arriving) |

### 4.2 Rules, evaluated per sample
| ID | Rule | Violation action |
|---|---|---|
| G1 | position and quaternion all finite | → `BLOCKED` |
| G2 | quaternion norm within tolerance | → `BLOCKED` |
| G3 | stamp age within 0…`max_stamp_age_s` | drop the sample, stay in state (existing behaviour) |
| G4 | \|position\| ≤ `max_radius_m` | → `BLOCKED` |
| G5 | step from last accepted ≤ `max_step_m` | → `BLOCKED` |
| G6 | time since last accepted ≤ `max_gap_s` | → `BLOCKED` |
| G7 | step > `warn_step_m` | forward, but count and log at most once per second |

G3 stays a drop rather than a latch because clock skew is transient and recoverable, and the existing
behaviour has proven itself on this vehicle. Everything else is evidence of a corrupted source.

### 4.3 Why latch rather than recover
FAST-LIO does not recover on its own: the incident ran for minutes until restarted. A restart gives it a
**new origin**, so resuming would present EKF2 with a step change — the same failure by another route. In
flight, `BLOCKED` means EV stops, which is a condition PX4 and WP-E already handle: EKF2 ages out the EV,
declares the estimate invalid if nothing else supports it, and the watchdog lands on
`xy_valid`/`z_valid` false. That is a designed, tested path; following a diverged estimate is not.

### 4.4 Status output
`/px4_odom_bridge/status` (`std_msgs/String`, 1 Hz): state, reason, counters for forwarded, dropped and
warned samples. Deliberately a plain string: no new message package (the masterplan's IR-2 rule for WP-F
applies by analogy), and easy to read in a runbook.

**`STALE` (added 2026-10-13, operator-approved).** The voided bench run of 2026-09-14 showed the status
reading `FORWARDING` for minutes while no EV existed, so a pilot would have had to difference the counter to
notice. The guard stays clock-free; `reported_state(now_s)` derives `STALE` from the last accepted stamp at
publish time. A sample stamped more than `max_gap_s` after the last accepted one still trips G5 and latches
`BLOCKED`; one stamped inside the gap but delivered late is still accepted on its stamp, so `STALE` can
revert to `FORWARDING` only within transport latency. Tests: `TestStale` (7) and the node test
`test_silence_reports_stale`; a mutant that never reports `STALE` fails 3 of them. 76 tests in total.

## 5. Test plan

### 5.1 Unit tests (pure core, no ROS)
| ID | Test |
|---|---|
| U1 | NaN or Inf in any position or quaternion component latches `BLOCKED` |
| U2 | Non-unit quaternion latches; norm within tolerance passes |
| U3 | Stamp age outside the window drops without latching; a later good sample still forwards |
| U4 | Magnitude exactly at `max_radius_m` passes, just above latches |
| U5 | Step exactly at `max_step_m` passes, just above latches |
| U6 | Gap exactly at `max_gap_s` passes, just above latches |
| U7 | `BLOCKED` is absorbing: no later sample, however perfect, is forwarded |
| U8 | `warn_step_m` counts without rejecting |
| U9 | First sample after start is accepted without a jump or gap test |
| U10 | Parameter validation rejects negative, zero and non-finite thresholds |
| U11 | Determinism: identical input sequences give identical decisions |

### 5.2 Replay tests against the real incident (regression fixtures)
| ID | Test |
|---|---|
| R1 | **Healthy replay**: 470 s of real EV samples from `log_2` before the outage → 100% forwarded, zero drops, zero warnings above `warn_step_m` except the known carries |
| R2 | **Incident replay**: the same log across the outage → the guard latches at the 12.15 s gap / 3.26 m jump at t+415 s, and forwards nothing after it |
| R3 | **Runaway replay**: the diverged stream from `log_3` (sess102) → latched on the first sample |
| R4 | Fixtures are extracted to CSV and committed under `src/px4_odom_bridge/test/data/`, so the tests need neither pyulog nor the original logs |

### 5.3 Node test
| ID | Test |
|---|---|
| N1 | Fake `/Odometry` publisher: healthy stream → EV published at the input rate, status says `FORWARDING` |
| N2 | Inject one NaN sample → EV publication stops immediately and permanently; status says `BLOCKED` with the reason |
| N3 | Inject a 12 s gap then a 3 m jump → same |
| N4 | After `BLOCKED`, a further 30 s of perfect data produces no EV messages |
| N5 | Status topic publishes at 1 Hz in every state |

Node tests run on an isolated DDS domain with the XRCE agent stopped, as the offboard tests do, so they
cannot reach a flight controller.

## 6. Verification plan (Step → Verify → Check)
| # | Step | Verify | Check |
|---|---|---|---|
| # | Step | Result |
|---|---|---|
| 1 | Extract fixtures from both logs to CSV | **DONE.** `test/data/healthy_ev.csv` (3937 rows, t<400 s), `outage_ev.csv` (1694 rows, t 380–560 s, the 12.15 s gap and the 3.26 m jump), `diverged_ev.csv` (47 rows, 1e4–1e6 m) |
| 2 | Write `ev_guard.py` + unit tests | **DONE.** U1–U11 pass |
| 3 | Add replay tests | **DONE.** Zero false positives on 3937 healthy samples; the outage latches at t+415.0 s on the gap rule; the diverged stream is refused on its first sample |
| 4 | Wire into `odom_bridge_node.py` | **DONE.** Guard runs after the stamp-age check, before `VehicleOdometry` is built; `fatal` logged once on block |
| 5 | Node tests | **DONE.** N1–N5 pass on an isolated domain (skipped unless `ROS_DOMAIN_ID` is non-zero) |
| 6 | Bench run against the real vehicle | **OUTSTANDING — VD-011.** Needs the LiDAR and FC powered |
| 7 | Documentation | **DONE.** Gate 3 transcript item 5 updated, VD-011 opened, `graphify update` run |

Full suite: **68 passed** (`ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=1`, `CYCLONEDDS_URI` unset).

**Mutation check** (as used for the offboard patch): the replay tests must fail against the unguarded
bridge. If they pass on both, they are not testing anything.

**Mutation check result, 2026-09-14.** A copy of the package with two mutations — the guard call deleted
from the node callback, and `check()` forced to accept every sample — fails **27 of 47** guard tests. The
20 that survive are exactly the ones that should: config validation, at-limit acceptance, determinism, and
the healthy-stream tests (a guard that fails open has no false positives by construction). The check also
found two weak tests of my own, now fixed:

- `test_nothing_is_forwarded_after_the_outage` was **vacuous** — its loop body only ran once the state was
  `BLOCKED`, so a fail-open guard passed it. It now asserts that the block happened at all.
- `test_carries_stay_below_the_reject_threshold` asserted `warned < 2% of forwarded` while the real warn
  count on healthy data is **0**; it passed on `0 < 78` and proved nothing. Replaced by a direct margin
  assertion on the measured distribution: over 3937 samples the largest 10 Hz step is **1.52 cm** (p99
  0.97 cm, median 0.34 cm) and the largest gap **0.144 s** (p99 0.118 s). That is **19.8x** headroom to
  `max_step_m` and 6.6x to `warn_step_m`, which is why no sample warns. If a future fixture or a faster
  vehicle erodes that margin, the test fails and the thresholds get re-derived instead of drifting
  silently toward false positives.

## 7. Residual risks, stated plainly
| Risk | Status |
|---|---|
| **Slow drift** below `warn_step_m` per sample and inside `max_radius_m` | **Not detected by this guard.** It is `lio_health_guard`'s domain (rate decay, jump statistics) and ultimately shows up as EKF2 innovations. The guard is containment, not health monitoring |
| A corrupted map that stays inside the radius | Not detected; same as above |
| False positive in a large venue | `max_radius_m` is a parameter; size it per venue with the geofence and map bounds (the WP-F spec's OI-2 already asks for these three to be sized together) |
| Loss of EV in flight after latching | By design. EKF2 and the watchdog handle EV loss; the alternative is following a diverged estimate |
| Guard bug rejecting healthy data | Covered by R1 (no false positives on 470 s of real data) and by the bench run in step 6 |

## 8. Open decisions — **resolved by the operator 2026-09-14**
1. **Latch policy**: *permanent block, restart required* — matches the offboard guard. Implemented.
2. **`max_radius_m`**: **20.0 m** (revised 2026-09-14; briefly 5.0 m). The operator's own observation
   that the rule is a geofence rather than a divergence detector settled this: 5 m bounded normal
   operation and could latch mid-flight, which trades a divergence hazard for an EV-loss hazard. At 20 m
   the rule can only fire on values no indoor flight can reach, while still catching the incident's
   1e4–1e6 m runaway with three orders of magnitude to spare. Still re-size per venue alongside the
   geofence and map bounds (WP-F OI-2) if a venue ever approaches 20 m; the node logs the active radius
   at startup.
3. **Status topic**: *yes* — `/px4_odom_bridge/status` at 1 Hz, `<STATE> fwd=<n> rej=<n> warn=<n>`.
4. **Scope**: *guard only*; `reset_counter` handling stays a separate change.

## 9. Deferred work (noted 2026-09-14, not scheduled)
Raised by the operator's observation that the radius rule is a geofence, not a divergence detector.
None of this is implemented; the guard as shipped is described above.

1. **Separate containment from implausibility.** `max_radius_m` bounds absolute position, so it cannot
   tell a diverged estimate from a legitimate flight — that is geofence semantics, and the guard's
   reaction (kill EV permanently) is the wrong response to leaving an area. Containment belongs to the
   PX4 geofence, acting on the vehicle. Re-purpose the radius as an *implausibility* bound, set well
   beyond any venue, where it still catches the 1e4–1e6 m runaway with large margin but can never latch
   during normal flight. **Partly acted on 2026-09-14**: the radius was raised 5.0 -> 20.0 m, which
   removes the mid-flight latch hazard indoors. The conceptual split (containment belongs to the PX4
   geofence, acting on the vehicle) is still outstanding.
2. **Make the step rule physics, not a fixed bound.** `max_step_m` is compared against a constant
   regardless of the gap, so the implied speed limit scales with the LIO rate: 0.30 m is 3 m/s at 10 Hz
   but only 0.6 m/s if the stream drops to 2 Hz, which `max_gap_s` still permits. Legitimate motion
   during a rate dip would latch the guard in flight. Replace with `step / gap` against a
   `max_speed_m_s`, with a floor on the divisor. Today's fixtures cannot expose this (bench nearly
   stationary: max step 1.52 cm, max gap 0.144 s), so the replay set needs a moving-vehicle fixture.
3. **Add real drift detection via independent sensors.** Neither of the above detects slow drift inside
   the bounds, which is the failure this guard cannot see. Two cross-checks are already on the airframe:
   - **Pixhawk IMU**: compare LIO's short-horizon delta-position against inertial dead reckoning. They
     diverge under LIO degeneracy long before the position bound is reached.
   - **Benewake rangefinder**: compare EV height against AGL (independent, 0.40–12 m). A vertical LIO
     drift shows up immediately as EV height and rangefinder disagreeing over a flat floor.
   Both are consistency checks rather than bounds, so they are venue-independent. Both prerequisites are
   now met: the rangefinder runs at 100 Hz (`uorb top`, 2026-09-14 — an earlier reading of 1 Hz from the
   log was a logger throttle, corrected in `docs/gate3_verification_2026-09-14.md` §11) and
   `EKF2_RNG_POS_X/Y/Z` have been measured and set.
4. **Protect the rangefinder cross-check against tilt.** A downward rangefinder measures *slant* range:
   true AGL is `range x cos(tilt)`. At this airframe's `MPC_TILTMAX_AIR 20 deg` that is a 6% error if
   uncompensated — 9 cm at 1.5 m, which is the whole Gate 4 position-hold budget. Two consequences for
   §9.3's EV-height-versus-AGL check, both of which must be built in from the start:
   - **De-tilt before comparing**: use the attitude quaternion to project the range onto the vertical,
     never compare raw range against EV height.
   - **Gate on tilt, and skip rather than flag**: above a tilt threshold (order 15–20 deg) the beam
     footprint has moved far enough laterally — 0.55 m at 20 deg and 1.5 m AGL — that it is plausibly
     looking at a different surface (a person, the tether anchor, uneven floor). A disagreement there is
     not evidence of LIO drift, so the check must go quiet, not raise. A tilt-induced deviation must never
     be able to block EV.
   Also verify what PX4 itself does at high tilt: ECL rejects range beyond a maximum tilt, but no such
   parameter is exposed in this firmware's metadata, so the threshold is a source constant and should be
   read from the PX4 version actually flashed rather than assumed.
