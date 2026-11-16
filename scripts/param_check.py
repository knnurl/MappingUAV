#!/usr/bin/env python3
# filepath: scripts/param_check.py
"""Compare a QGroundControl parameter dump with the values this repo expects.

  scripts/param_check.py docs/px4_params_<date>.params
  scripts/param_check.py dump.params --expected a.params b.params

Reads only files. It never talks to the flight controller.
Expected files hold "NAME value  # comment" lines; later files override
earlier ones. Default: config/px4/gate3_ekf2.params then gate4_expected.params.
Exit code: 0 all match, 1 a mismatch or a missing parameter, 2 unreadable input.
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PX4 = os.path.join(HERE, '..', 'src', 'drone_bringup', 'config', 'px4')
DEFAULT_EXPECTED = [os.path.join(PX4, 'gate3_ekf2.params'),
                    os.path.join(PX4, 'gate4_expected.params')]


def read_dump(path):
    """QGC dump: tab-separated vehicle, component, name, value, type."""
    out = {}
    with open(path) as fh:
        for line in fh:
            if line.startswith('#') or not line.strip():
                continue
            cols = line.rstrip('\n').split('\t')
            if len(cols) < 4:
                raise ValueError(f'{path}: not a QGC parameter line: {line.strip()!r}')
            out[cols[2]] = float(cols[3])
    if not out:
        raise ValueError(f'{path}: no parameters found')
    return out


def read_expected(paths):
    out = {}
    for path in paths:
        with open(path) as fh:
            for n, line in enumerate(fh, 1):
                body = line.split('#', 1)[0].split()
                if not body:
                    continue
                if len(body) != 2:
                    raise ValueError(f'{path}:{n}: expected "NAME value"')
                out[body[0]] = (float(body[1]), os.path.basename(path))
    return out


def compare(dump, expected):
    """[(name, expected, actual or None, source)] for every difference."""
    bad = []
    for name, (want, source) in sorted(expected.items()):
        got = dump.get(name)
        if got is None or abs(got - want) > 1e-6 * max(1.0, abs(want)):
            bad.append((name, want, got, source))
    return bad


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('dump')
    ap.add_argument('--expected', nargs='+', default=DEFAULT_EXPECTED)
    args = ap.parse_args(argv)
    try:
        dump = read_dump(args.dump)
        expected = read_expected(args.expected)
    except (OSError, ValueError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 2
    bad = compare(dump, expected)
    print(f'{len(expected)} expected parameters, {len(dump)} in the dump, '
          f'{len(bad)} differ')
    for name, want, got, source in bad:
        shown = 'MISSING from the dump' if got is None else f'FC has {got:g}'
        print(f'  {name:18s} expected {want:g}  {shown}  ({source})')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
