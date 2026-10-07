"""Command line: list limits from a model, or pre-flight a sequence file. (c) 2026 Ali Amini"""
from __future__ import annotations

import argparse
import sys

from .author import notice
from .errors import SafeMotionError
from .limits import parse_mjcf_limits
from .sequence import load_sequence


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="safemotion", description="Pre-flight tools for robot motion. Simulation first.")
    sub = p.add_subparsers(dest="cmd", required=True)
    a = sub.add_parser("limits", help="list joint limits found in a MuJoCo model")
    a.add_argument("model")
    a.add_argument("--exclude", default="", help="comma-separated joint names or patterns to skip, e.g. '*hip*,*knee*,*ankle*'")
    a.add_argument("--only", default="", help="comma-separated joint names or patterns to keep, e.g. '*shoulder*,*elbow*,*wrist*,waist*'")
    b = sub.add_parser("preflight", help="check a sequence file against the model's limits")
    b.add_argument("model")
    b.add_argument("sequence")
    b.add_argument("--exclude", default="")
    b.add_argument("--only", default="")
    b.add_argument("--speed", type=float, default=0.25, help="max fraction of a joint's range per second")
    b.add_argument("--margin", type=float, default=0.02)
    args = p.parse_args(argv)
    print(notice())
    try:
        with open(args.model, encoding="utf-8") as f:
            limits = parse_mjcf_limits(f.read(), exclude=[x for x in args.exclude.split(",") if x],
                                       include=[x for x in args.only.split(",") if x] or None)
        if args.cmd == "limits":
            for n, l in limits.items():
                print(f"{n:32s} {l.lo:9.4f} {l.hi:9.4f}")
            print(f"{len(limits)} joints")
            return 0
        with open(args.sequence, encoding="utf-8") as f:
            seq = load_sequence(f.read())
        problems = seq.preflight(limits, args.speed, args.margin)
    except (SafeMotionError, OSError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    if problems:
        print(f"{len(problems)} problem(s):")
        for x in problems:
            print(" -", x)
        return 1
    print("pre-flight OK: the sequence respects the range and speed policy (this does not make it safe on a real robot).")
    return 0
