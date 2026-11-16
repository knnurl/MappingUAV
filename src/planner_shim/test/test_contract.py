# filepath: src/planner_shim/test/test_contract.py
"""One command path (masterplan §6 [FIXED]): this package must never talk to
PX4 directly. Static scan of the package source."""
import pathlib
import re

SRC = pathlib.Path(__file__).resolve().parents[1] / 'planner_shim'
FORBIDDEN = [r'/fmu', r'px4_msgs', r'VehicleCommand', r'TrajectorySetpoint',
             r'set_arming_state', r'set_offboard_mode', r'Takeoff', r'\bLand\b']


def test_no_direct_px4_or_arming_interface():
    hits = []
    for path in sorted(SRC.glob('*.py')):
        text = path.read_text()
        for pat in FORBIDDEN:
            if re.search(pat, text):
                hits.append(f'{path.name}: {pat}')
    assert hits == []


def test_pure_modules_import_no_ros():
    for name in ('path_core.py', 'shim_core.py'):
        text = (SRC / name).read_text()
        assert not re.search(r'^\s*(import|from)\s+(rclpy|\w+_msgs)', text, re.M), name
