# filepath: scripts/test_param_check.py
# Run: python3 -m pytest -p no:cacheprovider scripts/test_param_check.py
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import param_check as pc  # noqa: E402


def write(tmp_path, name, text):
    p = tmp_path / name
    p.write_text(text)
    return str(p)


DUMP = '# Onboard parameters\n1\t1\tEKF2_EV_CTRL\t11\t6\n1\t1\tMPC_XY_VEL_MAX\t1.000000000000000000\t9\n'


def test_match(tmp_path):
    d = write(tmp_path, 'd.params', DUMP)
    e = write(tmp_path, 'e.params', 'EKF2_EV_CTRL 11  # c\nMPC_XY_VEL_MAX 1.0\n')
    assert pc.main([d, '--expected', e]) == 0


def test_mismatch_and_missing(tmp_path, capsys):
    d = write(tmp_path, 'd.params', DUMP)
    e = write(tmp_path, 'e.params', 'EKF2_EV_CTRL 15\nEKF2_OF_CTRL 0\n')
    assert pc.main([d, '--expected', e]) == 1
    out = capsys.readouterr().out
    assert 'EKF2_EV_CTRL' in out and 'FC has 11' in out
    assert 'EKF2_OF_CTRL' in out and 'MISSING' in out


def test_later_expected_file_wins(tmp_path):
    a = write(tmp_path, 'a.params', 'EKF2_EV_CTRL 15\n')
    b = write(tmp_path, 'b.params', '# note\n  # indented comment\nEKF2_EV_CTRL 11\n')
    assert pc.read_expected([a, b])['EKF2_EV_CTRL'] == (11.0, 'b.params')
    assert pc.main([write(tmp_path, 'd.params', DUMP), '--expected', a, b]) == 0


def test_bad_inputs(tmp_path):
    e = write(tmp_path, 'e.params', 'EKF2_EV_CTRL 11\n')
    assert pc.main([str(tmp_path / 'nope'), '--expected', e]) == 2
    assert pc.main([write(tmp_path, 'x.params', 'garbage line\n'), '--expected', e]) == 2
    assert pc.main([write(tmp_path, 'empty.params', '# only comments\n'), '--expected', e]) == 2
    bad = write(tmp_path, 'bad.params', 'ONLYNAME\n')
    assert pc.main([write(tmp_path, 'd.params', DUMP), '--expected', bad]) == 2


def test_repo_expected_files_parse():
    assert len(pc.read_expected(pc.DEFAULT_EXPECTED)) > 30
