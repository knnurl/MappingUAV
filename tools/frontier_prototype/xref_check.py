# filepath: tools/frontier_prototype/xref_check.py
"""Pass 2a (rev 2): internal consistency of the spec (cross-references, IDs, tables, defaults).
Rev 2: IDs embedded in test names (T-C01, T-I01, T-P01) are no longer read as C/I/P IDs;
asserts the pass-2 fixes are present."""
import json
import math
import os
import re
import sys

import yaml

PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'docs', 'wp-f_frontier_explorer_spec.md')
spec = open(PATH).read()
results = []


def check(name, ok, detail=''):
    results.append((bool(ok), name, detail))


def section(start_pat, end_pat=r'^## '):
    m = re.search(start_pat, spec, re.M)
    if not m:
        return ''
    rest = spec[m.end():]
    e = re.search(end_pat, rest, re.M)
    return rest[:e.start()] if e else rest


def table_rows(text):
    return [ln for ln in text.splitlines() if ln.startswith('|') and not re.match(r'^\|[-| ]+\|$', ln)]


def cells(row):
    return [c.strip() for c in row.strip().strip('|').split('|')]


def expand_ranges(text, prefix_pat):
    out = set()
    for a, b in re.findall(rf'({prefix_pat})\s*[–…-]\s*({prefix_pat})', text):
        pa = re.match(r'^(.*?)(\d+)$', a)
        pb = re.match(r'^(.*?)(\d+)$', b)
        if pa and pb and pa.group(1) == pb.group(1):
            w = len(pa.group(2))
            for n in range(int(pa.group(2)), int(pb.group(2)) + 1):
                out.add(f'{pa.group(1)}{n:0{w}d}')
    return out


def ids(pat):
    """Standalone IDs not embedded in a test name (T-C01 etc.)."""
    return set(re.findall(rf'(?<![A-Z]-)\b{pat}\b', spec)) | expand_ranges(spec, rf'(?<![A-Z]-){pat}')


# ---------------------------------------------------------------- headings & § refs
heads = {m.group(1) for m in re.finditer(r'^#{2,3} (\d+(?:\.\d+)?)\.? ', spec, re.M)}
bad_refs = []
for m in re.finditer(r'§(\d+(?:\.\d+)?)', spec):
    before = spec[max(0, m.start() - 60):m.start()]
    if re.search(r'masterplan[^|\n]*$', before) and not re.search(r'\)\s*[^|\n]*$', before.split('masterplan')[-1]):
        continue
    if 'masterplan' in before[-25:]:
        continue
    if m.group(1) not in heads:
        bad_refs.append(m.group(1))
check('every internal § reference resolves to a heading', not bad_refs, f'unresolved: {sorted(set(bad_refs))}')

# ---------------------------------------------------------------- parameters
p5 = section(r'^## 5\. Parameters')
params = {}
for row in table_rows(p5):
    c = cells(row)
    names = re.findall(r'`([a-z][a-z0-9_]*)`', c[0])
    if not names or c[0] == 'Name':
        continue
    for k, n in enumerate(names):
        parts = [x.strip() for x in c[2].split(',')]
        params[n] = parts[k] if len(parts) == len(names) else c[2]
check('parameter table parsed', len(params) >= 50, f'{len(params)} parameters')
outside = spec.replace(p5, '')
tokens = set(re.findall(r'`([a-z][a-z0-9_]*)`', outside)) | set(re.findall(r'\b([a-z]+(?:_[a-z0-9]+)+)\b', outside))
check('every parameter is referenced outside §5', not [n for n in params if n not in tokens],
      f'unused: {sorted(n for n in params if n not in tokens)}')
param_like = re.compile(r'^[a-z]+(_[a-z0-9]+)*_(m|s|m_s|cells|factor|entries|radius_m|min|max)$')
EXTERNAL = {'soft_margin', 'max_insert_range', 'clearance_cap', 'insert_period_s', 'edt_period_s',
            'map_publish_period_s', 'lio_timeout_s', 'estimate_timeout_s', 'max_vel', 'last_query_ms',
            'last_plan_ms', 'mission_elapsed_s', 'age_s', 'known_free_m2', 'kx_min', 'kx_max', 'ky_min', 'ky_max',
            'x_min_n', 'x_max_n', 'y_min_n', 'y_max_n', 'z_min_n', 'z_max_n'}
undefined = sorted(t for t in set(re.findall(r'`([a-z][a-z0-9_]*)`', outside))
                   if param_like.match(t) and t not in params and t not in EXTERNAL)
check('no undefined parameter-like identifiers', not undefined, f'undefined: {undefined}')

# ---------------------------------------------------------------- requirements & tests
appB = section(r'^## Appendix B\.', end_pat=r'^## ZZZ')
req_rows = {cells(r)[0]: cells(r)[2] for r in table_rows(appB) if re.match(r'^(FR|SR|IR|PR|VR)-\d+$', cells(r)[0])}
refs_req = set(re.findall(r'\b((?:FR|SR|IR|PR|VR)-\d+)\b', spec)) | expand_ranges(spec, r'(?:FR|SR|IR|PR|VR)-\d+')
check('every referenced requirement is defined in Appendix B', refs_req <= set(req_rows), f'{sorted(refs_req - set(req_rows))}')
check('every requirement maps to at least one test', all(re.search(r'[TSB]-', t) for t in req_rows.values()))
s13 = section(r'^## 13\. Test plan')
test_pat = r'(T-[A-Z]\d{2}|S-\d{2}b?|B-\d{2})'
test_ids = {cells(r)[0] for r in table_rows(s13) if re.match(rf'^{test_pat}$', cells(r)[0])}
check('test tables parsed', len(test_ids) >= 60, f'{len(test_ids)} tests')
refs_tests = set(re.findall(rf'\b{test_pat}\b', spec))
check('every referenced test exists in §13', refs_tests <= test_ids, f'missing: {sorted(refs_tests - test_ids)}')
traced = set(re.findall(rf'\b{test_pat}\b', appB))
check('every §13 test is traced by a requirement (Appendix B)', test_ids <= traced, f'untraced: {sorted(test_ids - traced)}')
cover_bad = []
for row in table_rows(section(r'^### 13\.1 Unit tests', end_pat=r'^### ')):
    c = cells(row)
    if re.match(r'^T-[A-Z]\d{2}$', c[0]):
        for r in re.findall(r'\b((?:FR|SR|IR|PR|VR)-\d+)\b', c[-1]):
            if c[0] not in re.findall(rf'\b{test_pat}\b', req_rows.get(r, '')):
                cover_bad.append(f'{c[0]}->{r}')
check("§13.1 'Covers' agrees with Appendix B", not cover_bad, f'mismatch: {cover_bad}')

# ---------------------------------------------------------------- states & transitions
states = {cells(r)[1].strip('`'): int(cells(r)[0]) for r in table_rows(section(r'^### 9\.1 States', end_pat=r'^### '))
          if cells(r)[0].isdigit()}
check('state codes contiguous from 0', sorted(states.values()) == list(range(len(states))), f'{states}')
ACTIVE = {'WAIT_MAP', 'SELECTING', 'NAVIGATING'}
s92 = section(r'^### 9\.2 Transitions', end_pat=r'^### ')
tids, adj, bad_names = [], {s: set() for s in states}, []
for row in table_rows(s92):
    c = cells(row)
    if not re.match(r'^T\d+$', c[0]):
        continue
    tids.append(int(c[0][1:]))
    frm, to = c[1], c[3]
    named = set(re.findall(r'`([A-Z_]+)`', frm))
    if frm.startswith('all except'):
        fnames = set(states) - named
    elif frm == 'any':
        fnames = set(states)
    else:
        fnames = named | (ACTIVE if 'active' in frm else set())
    tnames = named if to == 'same' else set(re.findall(r'`([A-Z_]+)`', to))
    bad_names += [n for n in fnames | tnames if n not in states]
    for f in fnames:
        adj.setdefault(f, set()).update(tnames)
check('transition IDs contiguous T1..Tn', tids == list(range(1, len(tids) + 1)), f'{tids}')
check('transition state names exist in §9.1', not bad_names, f'{sorted(set(bad_names))}')
incoming = set().union(*adj.values())
check('every state except IDLE has an incoming transition', (set(states) - {'IDLE'}) <= incoming)
check('every state except FAULT has an outgoing transition to another state',
      all(adj[s] - {s} for s in set(states) - {'FAULT'}), f'{ {s: adj[s] for s in states} }')
seen, stack = {'IDLE'}, ['IDLE']
while stack:
    for n in adj[stack.pop()]:
        if n not in seen:
            seen.add(n)
            stack.append(n)
check('all states reachable from IDLE', seen == set(states), f'unreachable: {sorted(set(states) - seen)}')
check('FAULT is absorbing', adj['FAULT'] <= {'FAULT'}, f'{adj["FAULT"]}')
check('T-range references within defined transitions', all(int(x[1:]) <= len(tids) for x in expand_ranges(spec, r'T\d+')))

# ---------------------------------------------------------------- interlocks
il = {cells(r)[0]: dict(target=cells(r)[2], hold=cells(r)[3].lower(), hover=cells(r)[4].lower())
      for r in table_rows(section(r'^### 9\.3 Interlocks', end_pat=r'^### ')) if re.match(r'^I\d+$', cells(r)[0])}
check('interlock IDs contiguous I1..In', sorted(int(k[1:]) for k in il) == list(range(1, len(il) + 1)))
check('every referenced interlock exists', ids(r'I\d+') <= set(il), f'{sorted(ids(r"I[0-9]+") - set(il))}')
check('SR-10: I3 and I4 rows publish neither hold goal nor hover',
      all(il[i]['hold'].startswith('no') and il[i]['hover'].startswith('no') for i in ('I3', 'I4')))
check('SR-4: I2 row publishes no hold goal', il['I2']['hold'].startswith('no'))
check('I1 (pose unknown) row publishes no hold goal', il['I1']['hold'].startswith('no'))
check('FAULT interlocks are exactly I7 and I8', {k for k, v in il.items() if 'FAULT' in v['target']} == {'I7', 'I8'})
s86 = section(r'^### 8\.6 Hold goal', end_pat=r'^### |^## ')
check('pass-2 fix present: §8.6 suppresses hold goals while I3 or I4 is raised',
      re.search(r'neither I3 nor I4 is raised', s86) and all(t in s86 for t in ('T17', 'T20', 'on_done')))
s93 = section(r'^### 9\.3 Interlocks', end_pat=r'^### ')
check('pass-2 fix present: interlock conditions evaluated in every state; tick order defined',
      'in every state' in s93 and re.search(r'T21 → interlocks \(T15\) → mission timeout \(T16\)', s93))
check('pass-2 fix present: status cap EVIDENCE_ONLY until DP-2 decision',
      re.search(r'\*\*EVIDENCE_ONLY\*\* until `decision_dp2_explore`', spec) is not None)

# ---------------------------------------------------------------- C, P, H, OI, VD, scenarios
cids = [cells(r)[0] for r in table_rows(section(r'^## 6\. Configuration')) if re.match(r'^C\d+$', cells(r)[0])]
check('checks C1..Cn contiguous', [int(x[1:]) for x in cids] == list(range(1, len(cids) + 1)), f'{cids}')
check('every referenced check exists', ids(r'C\d+') <= set(cids), f'{sorted(ids(r"C[0-9]+") - set(cids))}')
pids = [cells(r)[0] for r in table_rows(section(r'^### 9\.4 START', end_pat=r'^### ')) if re.match(r'^P\d+$', cells(r)[0])]
check('preconditions P1..Pn contiguous', [int(x[1:]) for x in pids] == list(range(1, len(pids) + 1)), f'{pids}')
check('every referenced precondition exists', ids(r'P\d+') <= set(pids), f'{sorted(ids(r"P[0-9]+") - set(pids))}')
hz = [cells(r) for r in table_rows(section(r'^## 10\. Hazard')) if re.match(r'^H-\d{2}$', cells(r)[0])]
check('hazards H-01..H-n contiguous', [int(h[0][2:]) for h in hz] == list(range(1, len(hz) + 1)))
check('every hazard has a test, except the declared undetectable H-11', [h[0] for h in hz if h[-1] in ('', '—')] == ['H-11'])
check('every referenced hazard exists', set(re.findall(r'\bH-\d{2}\b', spec)) <= {h[0] for h in hz})
oi = [cells(r)[0] for r in table_rows(section(r'^## 16\. Open items')) if re.match(r'^OI-\d+$', cells(r)[0])]
check('open items OI-1..n contiguous', [int(x[3:]) for x in oi] == list(range(1, len(oi) + 1)))
check('every referenced OI exists', set(re.findall(r'\bOI-\d+\b', spec)) <= set(oi))
vd = yaml.safe_load(re.search(r'```yaml\n(.*?)```', spec, re.S).group(1))
check('VD yaml block parses with required keys',
      all(set(d) >= {'id', 'package', 'claim_untested', 'original_criterion', 'discharge_path', 'status'} for d in vd))
check('every referenced VD-007+ exists in §14', {v for v in re.findall(r'VD-0\d\d', spec) if int(v[3:]) >= 7} <= {d['id'] for d in vd})
check('VD-007 carries the human-runbook clause', 'agent-written runbook' in vd[0]['discharge_path'])
sc = [cells(r)[0] for r in table_rows(section(r'^### 13\.2 Scenario', end_pat=r'^### ')) if re.match(r'^S-\d{2}b?$', cells(r)[0])]
check('scenario rows S-01..S-09 and S-06b present', {f'S-{n:02d}' for n in range(1, 10)} | {'S-06b'} <= set(sc))

# ---------------------------------------------------------------- JSON & numeric consistency
js = json.loads(re.search(r'```json\n(.*?)```', spec, re.S).group(1))
check('status JSON parses and has goal.tight', 'tight' in js['goal'])


def num(n):
    return float(re.findall(r'-?\d+(?:\.\d+)?', params[n].replace('−', '-'))[0])


res, zf, zmin, zmax, infl = num('resolution'), num('z_fly'), num('z_min'), num('z_max'), num('obstacle_inflation')
zL = (math.floor(zf / res + 1e-6) + 0.5) * res
check('C5 holds at defaults', zmin + infl <= zL <= zmax - infl, f'z_L={zL}')
for a, op, b in [('goal_clearance_m', '>=', 'traverse_clearance_m'), ('unknown_standoff_m', '>=', 'traverse_clearance_m'),
                 ('query_timeout_s', '<', 'cycle_period_s'), ('progress_window_s', '>', 'cycle_period_s'),
                 ('visited_radius_m', '>', 'reach_tolerance_xy_m'), ('goal_timeout_min_s', '>=', 'start_motion_timeout_s')]:
    check(f'{a} {op} {b}', eval(f'{num(a)} {op} {num(b)}'))
check('view_range_m ≤ map max_insert_range 8.0', num('view_range_m') <= 8.0)
check('lio_heartbeat_timeout_s > 1.0 s', num('lio_heartbeat_timeout_s') > 1.0)
check('fence_heartbeat_timeout_s > 0.5 s', num('fence_heartbeat_timeout_s') > 0.5)
check('nominal_speed_m_s ≤ EGO max_vel 1.0', num('nominal_speed_m_s') <= 1.0)
check('max_grid_cells ≥ 46×46', num('max_grid_cells') >= 46 * 46)
fence_enu, sm = (-1.5, 1.5, -1.5, 1.5, -0.3, 2.5), 0.5
box = (num('x_min'), num('x_max'), num('y_min'), num('y_max'), zmin, zmax)
inside = (box[0] >= fence_enu[0] + sm and box[1] <= fence_enu[1] - sm and box[2] >= fence_enu[2] + sm
          and box[3] <= fence_enu[3] - sm and box[4] >= fence_enu[4] + sm and box[5] <= fence_enu[5] - sm)
check("OI-2 claim: C4 fails on today's defaults", not inside)

fails = [r for r in results if not r[0]]
for ok, name, detail in results:
    print(f"{'PASS' if ok else 'FAIL'}  {name}" + (f'   [{detail}]' if detail and (not ok or 'parsed' in name) else ''))
print(f'\n{len(results)} checks, {len(fails)} failed')
sys.exit(1 if fails else 0)
