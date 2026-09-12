# filepath: tools/frontier_prototype/fact_check.py
"""Pass 1a: machine-check every interface / default / quote the spec relies on."""
import glob
import os
import re
import subprocess
import sys

WS = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
M, S = re.MULTILINE, re.DOTALL


def git(branch, path):
    r = subprocess.run(['git', '-C', WS, 'show', f'{branch}:{path}'],
                       capture_output=True, text=True)
    return r.stdout if r.returncode == 0 else None


def fs(path):
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return None


SHIM = ('git', 'wp-d-planner', 'src/planner_shim/planner_shim/go_to_with_avoidance_node.py')
CORE = ('git', 'wp-d-planner', 'src/planner_shim/planner_shim/shim_core.py')
GUARD = ('git', 'wp-d-planner', 'src/lio_health_guard/lio_health_guard/guard_node.py')
HEALTH = ('git', 'wp-d-planner', 'src/lio_health_guard/lio_health_guard/health_core.py')
EGO = ('git', 'wp-d-planner', 'src/drone_bringup/config/planning/ego_v2.yaml')
WDN = ('git', 'wp-e-geofence-watchdog', 'src/geofence_watchdog/geofence_watchdog/watchdog_node.py')
WDC = ('git', 'wp-e-geofence-watchdog', 'src/geofence_watchdog/geofence_watchdog/geofence_core.py')
MSN = ('git', 'wp-c-mapping', 'src/map_interface/src/map_server_node.cpp')
MCORE = ('git', 'wp-c-mapping', 'src/map_interface/include/map_interface/map_core.hpp')
SRV = ('git', 'wp-c-mapping', 'src/map_interface/srv/QueryMap.srv')
MYAML = ('git', 'wp-c-mapping', 'src/drone_bringup/config/map_interface.yaml')
SCENE = ('git', 'wp-c-mapping', 'tools/replay_tests/synthetic_scene.py')
BRIDGE = ('git', 'main', 'src/px4_odom_bridge/px4_odom_bridge/odom_bridge_node.py')
TOPICS = ('fs', '/opt/ros/humble/include/as2_core/names/topics.hpp')
PSTAT = ('fs', '/opt/ros/humble/share/as2_msgs/msg/PlatformStatus.msg')
PINFO = ('fs', '/opt/ros/humble/share/as2_msgs/msg/PlatformInfo.msg')
RAWOD = ('fs', '/opt/ros/humble/include/raw_odometry.hpp')
MPLAN = ('fs', f'{WS}/docs/ProgressReport/phases3-7_masterplan_consolidated.md')
BRINGUP_D = ('git', 'wp-d-planner', 'src/drone_bringup/setup.py')

CLAIMS = [
    # planner_shim
    ('F01', 'shim node name go_to_with_avoidance', SHIM, r"super\(\).__init__\('go_to_with_avoidance'\)", 0),
    ('F02', 'shim default x_min -4.5', SHIM, r"declare_parameter\('x_min', -4\.5\)", 0),
    ('F03', 'shim default x_max 4.5', SHIM, r"declare_parameter\('x_max', 4\.5\)", 0),
    ('F04', 'shim default y_min -4.5 / y_max 4.5', SHIM, r"'y_min', -4\.5\).*'y_max', 4\.5\)", S),
    ('F05', 'shim default z_min 0.2 / z_max 2.3', SHIM, r"'z_min', 0\.2\).*'z_max', 2\.3\)", S),
    ('F06', 'shim obstacle_inflation 0.4', SHIM, r"declare_parameter\('obstacle_inflation', 0\.4\)", 0),
    ('F07', 'shim goal sub PoseStamped /go_to_with_avoidance/goal depth 10', SHIM,
     r"create_subscription\(PoseStamped, '/go_to_with_avoidance/goal',\s*self\.on_goal, 10\)", S),
    ('F08', 'shim alert String /go_to_with_avoidance/alert depth 10', SHIM,
     r"String, '/go_to_with_avoidance/alert', 10\)", 0),
    ('F09', 'shim hover Empty /go_to_with_avoidance/hover_request', SHIM,
     r"Empty, '/go_to_with_avoidance/hover_request', 10\)", 0),
    ('F10', "shim LIO rejection string 'goal REJECTED: LIO degraded'", SHIM,
     r"String\(data='goal REJECTED: LIO degraded'\)", 0),
    ('F11', "shim generic rejection f'goal REJECTED: {why}'", SHIM, r"f'goal REJECTED: \{why\}'", 0),
    ('F12', 'shim has no goal status / reached publisher', SHIM, r"reached|status|GoalStatus", 'ABSENT'),
    ('F13', 'check_goal(goal, box, inflation) -> (ok, reason)', CORE,
     r"def check_goal\(goal, box: EnuBox, inflation: float\):.*return True, ''", S),
    ('F14', 'inflation applied as box-face margin only', CORE, r"box\.contains\(x, y, z, margin=inflation\)", 0),
    ('F15', 'EnuBox.contains inclusive with margin', CORE,
     r"self\.x_min \+ margin <= x <= self\.x_max - margin", 0),
    # lio_health_guard
    ('F16', 'LIO state UInt8 /lio_health/state (default QoS 10)', GUARD,
     r"UInt8, '/lio_health/state', 10\)", 0),
    ('F17', 'LIO timer 1.0 s publishes state every tick', GUARD,
     r"create_timer\(1\.0, self\.on_timer\).*def on_timer.*\n\s{8}self\.state_pub\.publish", S),
    ('F18', 'LioHealth OK=0 DEGRADED=1', HEALTH, r"OK = 0\s*\n\s*DEGRADED = 1", 0),
    # watchdog
    ('F19', 'watchdog node name geofence_watchdog', WDN, r"super\(\).__init__\('geofence_watchdog'\)", 0),
    ('F20', 'watchdog /geofence/state UInt8', WDN, r"UInt8, '/geofence/state', 10\)", 0),
    ('F21', 'watchdog heartbeat timer 0.5 s (2 Hz)', WDN, r"create_timer\(0\.5, self\._heartbeat\)", 0),
    ('F22', 'FenceState OK=0 SOFT=1 HARD=2', WDC, r"OK = 0\s*\n\s*SOFT = 1.*\n\s*HARD = 2", 0),
    ('F23', 'watchdog defaults x/y +-1.5, z -2.5/0.3, soft_margin 0.5', WDN,
     r"'x_min', -1\.5\).*'x_max', 1\.5\).*'y_min', -1\.5\).*'y_max', 1\.5\).*'z_min', -2\.5\).*'z_max', 0\.3\).*'soft_margin', 0\.5\)", S),
    # map_interface
    ('F24', 'map server node name map_interface', MSN, r"Node\(\s*\"map_interface\"", 0),
    ('F25', 'service /map_interface/query', MSN, r"create_service<srv::QueryMap>\(\s*\"/map_interface/query\"", S),
    ('F26', 'onQuery updates dirty EDT synchronously', MSN,
     r"void onQuery.*?if \(core_->distanceMapDirty\(\)\) \{\s*core_->updateDistanceMap\(\);", S),
    ('F27', 'map params declared: resolution, bounds, clearance_cap, max_insert_range', MSN,
     r"(?=.*declare_parameter<double>\(\"resolution\", 0\.2\))(?=.*\"x_min\", -10\.0)(?=.*\"z_max\", 3\.0)"
     r"(?=.*\"clearance_cap\", 4\.0)(?=.*\"max_insert_range\", 8\.0)", S),
    ('F28', 'srv occupancy 0 free 1 occupied 2 unknown; clearance -1 outside', SRV,
     r"0 = free, 1 = occupied, 2 = unknown.*-1\.0 = point outside", S),
    ('F29', 'outside bounds -> UNKNOWN', MCORE, r"if \(!bounds_\.contains\(x, y, z\)\) \{\s*return Occ::UNKNOWN;", S),
    ('F30', 'clearance -1 outside bounds / EDT error', MCORE, r"return -1\.0f;.*return -1\.0f;", S),
    ('F31', 'map yaml resolution 0.2, insert 0.5 s, range 8.0, edt 2.0 s, unknown_as_occupied false', MYAML,
     r"resolution: 0\.2.*unknown_as_occupied: false.*insert_period_s: 0\.5.*max_insert_range: 8\.0.*edt_period_s: 2\.0", S),
    ('F32', 'map yaml bounds x/y +-10, z -0.5..3.0', MYAML,
     r"x_min: -10\.0.*x_max: 10\.0.*y_min: -10\.0.*y_max: 10\.0.*z_min: -0\.5.*z_max: 3\.0", S),
    ('F33', 'synthetic scene mid-voxel surface comment', SCENE, r"MID-VOXEL", 0),
    # EGO
    ('F34', 'EGO obstacle_inflation 0.4, max_vel 1.0', EGO, r"max_vel: 1\.0.*obstacle_inflation: 0\.4", S),
    # bridge axis convention
    ('F35', 'bridge ENU->NED (y, x, -z): startup forward = East', BRIDGE,
     r"return \[float\(y\), float\(x\), float\(-z\)\]", 0),
    # Aerostack2
    ('F36', 'AS2 self_localization/pose with SensorDataQoS', TOPICS,
     r"namespace self_localization\s*\{\s*const rclcpp::QoS qos = rclcpp::SensorDataQoS\(\);.*pose\[\] = \"self_localization/pose\"", S),
    ('F37', 'AS2 platform/info with QoS(10)', TOPICS,
     r"namespace platform\s*\{\s*const rclcpp::QoS qos = rclcpp::QoS\(10\);\s*const char info\[\] = \"platform/info\"", S),
    ('F38', 'PlatformStatus FLYING = 3', PSTAT, r"FLYING\s*=\s*3", 0),
    ('F39', 'PlatformInfo has connected, armed, offboard, status', PINFO,
     r"bool connected.*bool armed.*bool offboard.*PlatformStatus status", S),
    ('F40', 'raw_odometry pose frame = earth frame', RAWOD, r"pose\.header\.frame_id = get_earth_frame\(\);", 0),
    ('F41', 'raw_odometry set_map_to_odom default true', RAWOD, r"bool set_map_to_odom_ = true;", 0),
    # masterplan quotes
    ('F42', 'masterplan: exploration planner never talks to PX4 directly', MPLAN,
     r"The exploration planner never talks to PX4 directly", 0),
    ('F43', 'masterplan: planners never consume NED', MPLAN, r"Planners never consume NED", 0),
    ('F44', 'masterplan: trip test is Gate 7 entry criterion', MPLAN,
     r"trip test is the Gate 7 entry criterion", 0),
    ('F45', 'masterplan: 2.0 CPU cores and 2.5 GB', MPLAN, r"2\.0 CPU cores and 2\.5 GB RAM", 0),
    ('F46', 'masterplan §8 WP-F build precondition map_interface merged', MPLAN,
     r"WP-F.*\*\*Build precondition:\*\* WP-C `map_interface` merged", S),
    ('F47', 'masterplan: geofence native to planner AND WP-E', MPLAN,
     r"geofence must be native to the chosen planner AND enforced independently by WP-E", 0),
    ('F48', 'DEFECT CONFIRMED: wp-d setup.py does not install config/planning (ego_v2.yaml)', BRINGUP_D,
     r"'planning'", 'ABSENT'),
]


def main():
    fails = 0
    cache = {}
    for cid, desc, src, pat, flags in CLAIMS:
        key = src
        if key not in cache:
            cache[key] = git(src[1], src[2]) if src[0] == 'git' else fs(src[1])
        text = cache[key]
        if text is None:
            status = 'NOSRC'
        elif flags == 'ABSENT':
            status = 'PASS' if not re.search(pat, text) else 'FAIL'
        else:
            status = 'PASS' if re.search(pat, text, flags) else 'FAIL'
        if status != 'PASS':
            fails += 1
        print(f'{status:5s} {cid} {desc}')

    # VD ids unused on every branch
    branches = subprocess.run(['git', '-C', WS, 'for-each-ref', '--format=%(refname:short)', 'refs/heads'],
                              capture_output=True, text=True).stdout.split()
    used = set()
    for b in branches:
        t = git(b, 'docs/verification_debt.yaml') or ''
        used |= set(re.findall(r'VD-\d{3}', t))
    clash = sorted({'VD-007', 'VD-008', 'VD-009'} & used)
    st = 'PASS' if not clash else 'FAIL'
    fails += st != 'PASS'
    print(f'{st:5s} F49 VD-007..009 unused on all branches (used: {sorted(used)})')

    # python / ROS runtime deps importable
    probe = ("import scipy.ndimage as nd; [getattr(nd, n) for n in "
             "('label','binary_dilation','distance_transform_edt','generate_binary_structure')];"
             "import hypothesis; from std_srvs.srv import Trigger; from rcl_interfaces.srv import GetParameters;"
             "from as2_msgs.msg import PlatformInfo; from visualization_msgs.msg import MarkerArray; print('ok')")
    r = subprocess.run(['bash', '-lc', f'source /opt/ros/humble/setup.bash && python3 -c "{probe}"'],
                       capture_output=True, text=True)
    st = 'PASS' if r.stdout.strip().endswith('ok') else 'FAIL'
    fails += st != 'PASS'
    print(f'{st:5s} F50 runtime deps importable (scipy.ndimage fns, hypothesis, std_srvs, rcl_interfaces, as2_msgs, visualization_msgs) {r.stderr.strip()[-120:]}')
    for tool in ('ament_flake8', 'ament_pep257'):
        ok = bool(glob.glob(f'/opt/ros/humble/bin/{tool}'))
        fails += not ok
        print(f"{'PASS' if ok else 'FAIL':5s} F5x {tool} installed")
    print(f'\n{len(CLAIMS) + 4} claims, {fails} not PASS')
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()
