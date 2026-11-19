"""Unambiguous NX timed segment CLI: --3 seconds, --1.0 metres/second."""
import math
import re


def normalize_timed_args(argv, parser):
    result = list(argv)
    for pattern, option, label in ((r'--[0-9]+', '--timed-straight', '时长'),
                                    (r'--[0-9]+\.[0-9]+', '--timed-speed', '速度')):
        short = [value for value in result if re.fullmatch(pattern, value)]
        if short:
            if len(short) != 1 or any(value == option or value.startswith(option+'=')
                                      for value in result):
                parser.error('只能指定一个计时直行'+label)
            result = [option+'='+value[2:] if value == short[0] else value for value in result]
    return result


def validate_timed_speed(args, parser):
    if args.timed_speed is not None:
        if args.timed_straight is None:
            parser.error('速度须同时指定计时段，如 --3 --1.0')
        if not math.isfinite(args.timed_speed) or not .1 <= args.timed_speed <= 1.0:
            parser.error('计时直行速度范围为 0.10–1.00 m/s')
    else:
        args.timed_speed = 1.0
