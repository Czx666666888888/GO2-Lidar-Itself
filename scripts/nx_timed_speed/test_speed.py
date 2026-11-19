"""CLI and controller tests; no ROS or hardware is used."""
import argparse
import importlib
import os
from pathlib import Path
import sys

import pytest

ROOT = Path(os.environ.get('NX_SPEED_PREPARED', '/tmp/nx-speed-prepared'))
sys.path.insert(0, str(ROOT/'scripts'))
sys.path.insert(0, str(ROOT/'scripts/nx_timed_move_only'))
from nx_timed_options import normalize_timed_args, validate_timed_speed
TimedStraight = importlib.import_module('go2_reactive_avoid.timed_straight').TimedStraight


def parse(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument('--timed-straight', type=float)
    parser.add_argument('--timed-speed', type=float)
    args = parser.parse_args(normalize_timed_args(argv, parser))
    validate_timed_speed(args, parser)
    return args


@pytest.mark.parametrize('argv,seconds,speed', [
    (['--1', '--1.0'], 1., 1.), (['--3', '--0.6'], 3., .6),
    (['--0.6', '--3'], 3., .6), (['--3'], 3., 1.),
    (['--timed-straight', '2.5', '--0.8'], 2.5, .8),
    (['--timed-straight=2.5', '--timed-speed=.7'], 2.5, .7)])
def test_duration_and_speed_are_distinct(argv, seconds, speed):
    args = parse(argv)
    assert (args.timed_straight, args.timed_speed) == (seconds, speed)


@pytest.mark.parametrize('argv', [
    ['--1', '--2'], ['--3', '--1.0', '--0.8'], ['--3', '--1.1'],
    ['--3', '--0.0'], ['--1.0'], ['--3', '--timed-speed=nan'],
    ['--3', '--timed-speed=inf'], ['--3', '--1.0', '--timed-speed=.8'],
    ['--3', '--timed-straight=2'], ['--3', '--timed-speed=-.1']])
def test_ambiguous_or_invalid_values_fail_before_start(argv):
    with pytest.raises(SystemExit):
        parse(argv)


@pytest.mark.parametrize('speed', [.1, .6, .8, 1.])
def test_selected_speed_with_confirmation_heading_and_duration(speed, tmp_path):
    timed = TimedStraight(2., tmp_path/'ready', speed)
    assert timed.update(0., 0., None, .1, True, True) == (0., 0.)
    timed.confirmed_at = 0.
    assert timed.update(.99, .99, None, .1, True, True) == (0., 0.)
    assert timed.update(1., 1., None, .1, True, True) == (speed, 0.)
    vx, wz = timed.update(1.1, 1.1, None, .15, False, True)
    assert vx == speed and wz < 0.
    assert timed.update(3., 3., None, .15, False, True) == (0., 0.)
    assert timed.phase == 'SETTLE'
    timed.update(3.1, 3.1, None, .15, True, True)
    timed.update(3.7, 3.7, None, .15, True, True)
    assert timed.phase == 'DONE'


@pytest.mark.parametrize('speed', [0., -.1, 1.1, float('nan'), float('inf')])
def test_controller_rejects_invalid_speed(speed, tmp_path):
    with pytest.raises(ValueError):
        TimedStraight(1., tmp_path/'ready', speed)
