# filepath: src/frontier_explorer/test/test_purity_contract.py
"""T-U01 (core purity, VR-1) and T-U02 (static contract, SR-2 / IR-2)."""
import pathlib
import re

SRC = pathlib.Path(__file__).resolve().parents[1] / 'frontier_explorer'
CORE = ['config.py', 'grid.py', 'frontier.py', 'planner.py', 'lifecycle.py', 'fsm.py']
FORBIDDEN = [r'/fmu', r'px4_msgs', r'VehicleCommand', r'set_arming_state',
             r'set_offboard_mode', r'TakeoffBehavior', r'LandBehavior',
             r'motion_reference', r'actuator_command', r'cmd_vel']


def test_core_modules_import_no_ros():
    for name in CORE:
        text = (SRC / name).read_text()
        assert not re.search(r'^\s*(import|from)\s+(rclpy|\w+_msgs|\w+_srvs)\b', text, re.M), name


def test_no_px4_arming_or_motion_interface_anywhere_in_the_package():
    hits = [f'{p.name}: {pat}' for p in sorted(SRC.glob('*.py'))
            for pat in FORBIDDEN if re.search(pat, p.read_text())]
    assert hits == []


def test_node_publishes_only_the_topics_of_spec_4_2():
    text = (SRC / 'explorer_node.py').read_text()
    topics = set(re.findall(r"create_publisher\(\s*\w+,\s*'([^']+)'", text))
    assert topics == {'/go_to_with_avoidance/goal', '/go_to_with_avoidance/hover_request',
                      '/frontier_explorer/state', '/frontier_explorer/status',
                      '/frontier_explorer/alert'}
    assert 'ActionClient' not in text
