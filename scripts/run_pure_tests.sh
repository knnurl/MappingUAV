#!/usr/bin/env bash
# filepath: scripts/run_pure_tests.sh
# Every test that needs no ROS installation. Used by CI (.github/workflows/tests.yml)
# and runnable anywhere:  bash scripts/run_pure_tests.sh
set -euo pipefail
cd "$(dirname "$0")/.."
export PYTHONNOUSERSITE="${PYTHONNOUSERSITE:-1}"
run() {  # run <PYTHONPATH> <pytest args...>
  local path="$1"; shift
  echo "== $*"
  PYTHONPATH="$path" python3 -m pytest -p no:cacheprovider -q "$@"
}
P=src/planner_shim
run src/px4_odom_bridge \
    src/px4_odom_bridge/test/test_ev_guard.py src/px4_odom_bridge/test/test_ev_guard_replay.py \
    src/px4_odom_bridge/test/test_step_budget_and_pairing.py
run src/geofence_watchdog src/geofence_watchdog/test/test_geofence_core.py \
    src/geofence_watchdog/test/test_land_rules.py
run src/lio_health_guard src/lio_health_guard/test
run $P $P/test/test_shim_core.py $P/test/test_path_core.py $P/test/test_contract.py
run src/frontier_explorer:$P src/frontier_explorer/test --ignore src/frontier_explorer/test/test_closed_loop.py
run src/sim_bringup src/sim_bringup/test
run src/map_export src/map_export/test
run . tools/contract_tests scripts/test_param_check.py
echo "ALL PURE TESTS PASSED"
