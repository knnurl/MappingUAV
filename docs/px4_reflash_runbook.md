<!-- filepath: docs/px4_reflash_runbook.md -->
# Reflash Record: ArduPilot 4.5.4 → PX4 v1.15 (Pixhawk 6C)

**Status: COMPLETED** (operator-reported 2026-09-13). This file was the operator
runbook; it is now the as-executed record. Do not re-run it.

Decision 2026-07-06: FC found running ArduPilot 4.5.4 (contradicting the
Gate-1 premise); operator chose reflash to PX4 v1.15 so the built stack
stays valid. HUMAN-EXECUTED — the agent never touches the vehicle.
Pre-reflash state archived: docs/ardupilot_4.5.4_params_pre_reflash.txt
(3.2 h flight time, learned hover thrust, calibrations — keep it forever).

## As-applied state
- Firmware: **PX4 v1.15.0** (parameter dump of 2026-09-12), airframe
  `SYS_AUTOSTART 4019` (Holybro X500 V2). Matches the `px4_msgs` release/1.15 pin.
- Parameter sources of truth: `src/drone_bringup/config/px4/gate3_ekf2.params`
  and `src/drone_bringup/config/px4/migration_from_ardupilot.params`.
- Loaded through the generated QGC files `qgc_load_pass1.params` → reboot →
  `qgc_load_pass2.params` (same directory). Regenerate them if either source
  file changes.
- QGC ran on the Jetson in its container: `./scripts/run_qgc.sh` (Ubuntu 24.04;
  see `docker/qgc/Dockerfile` — the AppImage cannot run on JetPack 6 directly).

## Deviations from the plan as first written
1. **Rangefinder port:** the Benewake went to **TELEM3**, not GPS2/UART4
   (`SENS_TFMINI_CFG 103`). There is no `SER_TEL3_BAUD`: the tfmini driver
   hardcodes 115200 and declares no baud parameter, and ArduPilot also used 115200.
2. **Two-pass parameter load:** `SER_TEL2_BAUD` only exists once TELEM2 is
   assigned to uXRCE-DDS, so it and `UXRCE_DDS_SYNCT` are in pass 2, after a reboot.
3. **`EKF2_EV_DELAY` = 5.0 ms** on the FC, not the 50.0 of the original plan.
   `px4_odom_bridge` now stamps `timestamp_sample` with the LIO scan time
   (`main` 20e851e), so the delay is only the residual; `gate3_ekf2.params`
   says 5.0 and `qgc_load_pass1.params` was corrected to 5.0 on 2026-09-13.

## A. Flash + base setup (bench, props OFF throughout)
[x] 1. QGC → Firmware: flash **PX4 v1.15.x stable** (matches px4_msgs pin).
[x] 2. Airframe: **Holybro X500 V2** preset → reboot.
[x] 3. Sensor calibration: accel, level horizon, compass (yes, even though
       indoor profile sets EKF2_MAG_TYPE=5 — bench sanity + outdoor fallback).
[x] 4. RC calibration; reassign:
       - kill switch (RC_MAP_KILL_SW) — AP had ArmDisarm on ch5, note the
         channels differ per your transmitter setup, TEST DISARMED;
       - flight mode switch (POSITION primary; AP used ch7).
[x] 5. **Motor mapping**: QGC Actuators tab. PX4 Quad-X numbering DIFFERS
       from ArduPilot's — verify each motor position + spin direction
       individually, props OFF. This is the highest-risk migration step.

## B. Wiring change (power off)
[x] 6. Move the Benewake rangefinder from **TELEM2** (AP SERIAL2, 115200) to
       **TELEM3** — TELEM2 is reserved for the uXRCE-DDS link to the Jetson
       (921600, gate1_bridge). TELEM3 was unused under AP (SERIAL5 and
       SERIAL6 both PROTOCOL=-1); confirm the connector on the silkscreen
       before crimping.
[x] 7. Wire Jetson ↔ TELEM2 per gate1_bridge.launch.py header (pin 8 TX →
       FC RX, pin 10 RX ← FC TX, common GND, 3.3 V).
       HereFlow stays on CAN1 (DroneCAN) — no change.

## C. Parameters (QGC, then reboot FC, then read back and diff)
[x] 8. Apply `src/drone_bringup/config/px4/gate3_ekf2.params`
       (uXRCE on TELEM2, EKF2 external-vision indoor profile, UAVCAN_ENABLE,
       Gate-4 safety envelope).
[x] 9. Apply `src/drone_bringup/config/px4/migration_from_ardupilot.params`
       (flow mounting offsets, battery calibration, hover thrust, TF02 port).
[x] 10. Reboot; re-download params and diff against both files (params can
        silently fail to save pre-reboot).

## D. Verification handoff (agent takes over from here)
[x] 11. `listener sensor_optical_flow` and `listener distance_sensor` in the
        QGC MAVLink console show live plausible data (flow + TF02; check the
        VL53-shadowing note in the migration params file).
[x] 12. Battery voltage in QGC matches a multimeter within ~0.2 V.
[x] 13a. Tell the agent (done 2026-09-13).
[ ] 13b. Agent runs `scripts/verify_gate1.sh` (XRCE link up), then proceed down
        the normal gate sequence. **Pending.**

## Carried forward (not closed by this record)
- **Gate 1:** `scripts/verify_gate1.sh` (13b).
- **Before Gate 3, blocking:** confirm which `distance_sensor` instance EKF2
  consumes. The HereFlow's VL53L1X (0.08–0.65 m) and the Benewake (0.55–22 m)
  both point down; if EKF2 uses the VL53, height aiding saturates at 0.65 m.
  See the note in `migration_from_ardupilot.params`.
- **Gate 3:** measure the residual `EKF2_EV_DELAY` from logs once the LiDAR is installed.
- **Gate 4 prep:** hover-thrust sanity — `MPC_THR_HOVER 0.62` came from AP's
  learned value; watch the first tethered spool-up.

## What does NOT transfer from ArduPilot
Attitude-rate PID tune (ATC_*), EK3 config, compass calibration offsets,
RC option assignments. PX4's X500 preset + our params replace them; expect
Gate 4 tether time for tune verification, not a tuned-vehicle shortcut.
