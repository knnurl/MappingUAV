<!-- filepath: docs/px4_reflash_runbook.md -->
# Operator Runbook: ArduPilot 4.5.4 → PX4 v1.15 Reflash (Pixhawk 6C)

Decision 2026-07-06: FC found running ArduPilot 4.5.4 (contradicting the
Gate-1 premise); operator chose reflash to PX4 v1.15 so the built stack
stays valid. HUMAN-EXECUTED — the agent never touches the vehicle.
Pre-reflash state archived: docs/ardupilot_4.5.4_params_pre_reflash.txt
(3.2 h flight time, learned hover thrust, calibrations — keep it forever).

## A. Flash + base setup (bench, props OFF throughout)
[ ] 1. QGC → Firmware: flash **PX4 v1.15.x stable** (matches px4_msgs pin).
[ ] 2. Airframe: **Holybro X500 V2** preset → reboot.
[ ] 3. Sensor calibration: accel, level horizon, compass (yes, even though
       indoor profile sets EKF2_MAG_TYPE=5 — bench sanity + outdoor fallback).
[ ] 4. RC calibration; reassign:
       - kill switch (RC_MAP_KILL_SW) — AP had ArmDisarm on ch5, note the
         channels differ per your transmitter setup, TEST DISARMED;
       - flight mode switch (POSITION primary; AP used ch7).
[ ] 5. **Motor mapping**: QGC Actuators tab. PX4 Quad-X numbering DIFFERS
       from ArduPilot's — verify each motor position + spin direction
       individually, props OFF. This is the highest-risk migration step.

## B. Wiring change (power off)
[ ] 6. Move the Benewake rangefinder from **TELEM2** (AP SERIAL2) to the
       **GPS2/UART4** port — TELEM2 is reserved for the uXRCE-DDS link to
       the Jetson (921600, gate1_bridge).
[ ] 7. Wire Jetson ↔ TELEM2 per gate1_bridge.launch.py header (pin 8 TX →
       FC RX, pin 10 RX ← FC TX, common GND, 3.3 V).
       HereFlow stays on CAN1 (DroneCAN) — no change.

## C. Parameters (QGC, then reboot FC, then read back and diff)
[ ] 8. Apply `src/drone_bringup/config/px4/gate3_ekf2.params`
       (uXRCE on TELEM2, EKF2 external-vision indoor profile, UAVCAN_ENABLE,
       Gate-4 safety envelope).
[ ] 9. Apply `src/drone_bringup/config/px4/migration_from_ardupilot.params`
       (flow mounting offsets, battery calibration, hover thrust, TF02 port).
[ ] 10. Reboot; re-download params and diff against both files (params can
        silently fail to save pre-reboot).

## D. Verification handoff (agent takes over from here)
[ ] 11. `listener sensor_optical_flow` and `listener distance_sensor` in the
        QGC MAVLink console show live plausible data (flow + TF02; check the
        VL53-shadowing note in the migration params file).
[ ] 12. Battery voltage in QGC matches a multimeter within ~0.2 V.
[ ] 13. Tell the agent → run `scripts/verify_gate1.sh` (XRCE link up), then
        proceed down the normal gate sequence. Gate 4 prep checklist gains
        one item: hover-thrust sanity (MPC_THR_HOVER 0.62 came from AP's
        learned value; watch first tethered spool-up).

## What does NOT transfer from ArduPilot
Attitude-rate PID tune (ATC_*), EK3 config, compass calibration offsets,
RC option assignments. PX4's X500 preset + our params replace them; expect
Gate 4 tether time for tune verification, not a tuned-vehicle shortcut.
