"""Runs a sequence through the safety filter against any robot that offers read_state() and send().
No real robot adapter is shipped: write one against your simulator first.
(c) 2026 Ali Amini"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Protocol

from .filter import SafetyFilter
from .sequence import Sequence


@dataclass(frozen=True)
class State:
    values: dict
    stamp: float


class RobotIO(Protocol):
    def read_state(self) -> State: ...
    def send(self, command: dict) -> None: ...


@dataclass
class RunResult:
    completed: bool
    steps: int
    reason: str | None = None


@dataclass
class Recorder:
    rows: list = field(default_factory=list)

    def add(self, step: int, t: float, target: dict, command: dict, state: dict, flags: str) -> None:
        for n in command:
            self.rows.append((step, round(t, 6), n, target.get(n), command[n], state.get(n), flags))

    def to_csv(self) -> str:
        def cell(v, text=False):
            s = "" if v is None else str(v)
            if text and s[:1] in ("=", "+", "-", "@", "\t", "\r"):
                s = "'" + s
            if any(c in s for c in ',"\n\r'):
                s = '"' + s.replace('"', '""') + '"'
            return s
        lines = ["step,t,joint,target,command,state,flags"]
        for step, t, joint, target, command, state, flags in self.rows:
            lines.append(",".join([cell(step), cell(t), cell(joint, True), cell(target), cell(command), cell(state), cell(flags, True)]))
        return "\n".join(lines) + "\n"


def realtime(dt: float, monotonic: Callable[[], float] | None = None, sleep: Callable[[float], None] | None = None):
    """Return (clock, pace) for a real-time loop: clock(i) is seconds since the start, pace(i) sleeps until the next tick."""
    import time
    mono = monotonic or time.monotonic
    nap = sleep or time.sleep
    t0 = mono()

    def clock(i: int) -> float:
        return mono() - t0

    def pace(i: int) -> None:
        delay = t0 + (i + 1) * dt - mono()
        if delay > 0:
            nap(delay)

    return clock, pace


def run_sequence(sequence: Sequence, robot: RobotIO, filt: SafetyFilter, dt: float, recorder: Recorder | None = None,
                 dry_run: bool = False, stale_after: float | None = None, clock: Callable[[int], float] | None = None,
                 pace: Callable[[int], None] | None = None, on_estop: Callable[[str], None] | None = None) -> RunResult:
    """dry_run computes and records every command but never calls robot.send().
    stale_after (seconds) arms the watchdog: if the state feed stops updating, the filter latches an emergency stop.
    clock(i) gives the time used for the filter's speed limits; for a real robot pass the clock from realtime(dt).
    pace(i) is called after every step (use it to sleep until the next tick).
    on_estop(reason) is called once when an emergency stop latches; use it to put the robot into its own damping mode."""
    clock = clock or (lambda i: i * dt)
    n = math.ceil(sequence.duration / dt - 1e-9)
    steps = n + 1                                    # the last step lands exactly on the final keyframe
    for i in range(steps):
        t = sequence.start + min(i * dt, sequence.duration)
        now = clock(i)
        state = robot.read_state()
        if stale_after is not None and not dry_run and now - state.stamp > stale_after:
            filt.estop("stale_state")
        target = sequence.sample(t)
        command, report = filt.apply(target, state.values, check_tracking=not dry_run, now=now)
        flags = ";".join(x for x in [
            f"clamped:{','.join(report.clamped)}" if report.clamped else "",
            f"rate_limited:{','.join(report.rate_limited)}" if report.rate_limited else "",
            f"fault:{report.fault}" if report.fault else "",
            f"estop:{report.estop_reason}" if report.estopped else "",
        ] if x)
        if recorder is not None:
            recorder.add(i, t, target, command, state.values, flags)
        if report.estopped:
            if on_estop is not None:
                on_estop(filt.estop_reason or "estop")
            return RunResult(False, i, filt.estop_reason)
        if not dry_run:
            robot.send(command)
        if pace is not None:
            pace(i)
    return RunResult(True, steps)


class FakeRobot:
    """A tiny stand-in for tests: joints follow commands with a first-order lag. Not a physics model."""

    def __init__(self, joints: dict, dt: float, lag: float = 0.05, blocked: tuple = ()):
        self.q = {k: float(v) for k, v in joints.items()}
        self.dt, self.lag, self.blocked = dt, lag, set(blocked)
        self.stamp = 0.0
        self.frozen = False
        self.sent: list = []

    def read_state(self) -> State:
        return State(dict(self.q), self.stamp)

    def send(self, command: dict) -> None:
        self.sent.append(dict(command))
        for n, c in command.items():
            if n not in self.blocked:
                self.q[n] += (c - self.q[n]) * min(1.0, self.dt / self.lag)
        if not self.frozen:
            self.stamp += self.dt
