# filepath: tools/contract_tests/test_ownership.py
"""Who may publish what (docs/safety_ownership.md). Static scan of the
flight packages; the rig's fakes are excluded.

Run: python3 -m pytest -p no:cacheprovider tools/contract_tests
"""
import pathlib
import re

SRC = pathlib.Path(__file__).resolve().parents[2] / 'src'
OURS = ['px4_odom_bridge', 'geofence_watchdog', 'lio_health_guard', 'planner_shim',
        'frontier_explorer', 'map_interface', 'map_export']

# topic or name fragment -> the one package allowed to mention it in a publisher / client
OWNERS = {
    r"create_publisher\([^)]*vehicle_visual_odometry|EV_TOPIC, 10\)": 'px4_odom_bridge',
    r"create_publisher\(\s*VehicleCommand": 'geofence_watchdog',
    r"create_publisher\([^)]*'/lio_health/": 'lio_health_guard',
    r"create_publisher\([^)]*'/geofence/": 'geofence_watchdog',
    r"create_publisher\([^)]*'/px4_odom_bridge/|(STATUS|STATE)_TOPIC, 10\)": 'px4_odom_bridge',
    r"ActionClient\(": 'planner_shim',
}


def sources():
    for pkg in OURS:
        for path in (SRC / pkg).rglob('*'):
            if path.suffix in ('.py', '.cpp', '.hpp') and '/test/' not in str(path):
                yield pkg, path


def test_each_safety_output_has_exactly_one_owner():
    wrong = []
    seen = {owner: False for owner in OWNERS.values()}
    for pkg, path in sources():
        text = path.read_text()
        for pattern, owner in OWNERS.items():
            if re.search(pattern, text):
                seen[owner] = True
                if pkg != owner:
                    wrong.append(f'{path.relative_to(SRC)} matches {pattern!r}, owner is {owner}')
    assert wrong == []
    assert all(seen.values()), seen


def test_only_the_watchdog_and_the_bridge_touch_px4():
    hits = sorted({pkg for pkg, path in sources() if re.search(r"/fmu/|px4_msgs", path.read_text())})
    assert hits == ['geofence_watchdog', 'px4_odom_bridge'], hits


def test_nothing_on_the_vehicle_reads_the_map_frame():
    """Loop-closure corrections live in map -> odom; no flight node may consume them."""
    hits = [str(path.relative_to(SRC)) for _, path in sources()
            if re.search(r"lookupTransform|lookup_transform|TransformListener", path.read_text())]
    assert hits == []


def test_no_log_call_site_changes_severity():
    """rclpy raises when one call site logs at different severities; this killed the
    watchdog on a soft-then-hard breach (2026-11-16)."""
    bad = [str(path.relative_to(SRC)) for _, path in sources() if path.suffix == '.py'
           and re.search(r"^\s*log\s*=\s*\(?\s*self\.get_logger\(\)\.\w+\s+if\b", path.read_text(), re.M)]
    assert bad == []
