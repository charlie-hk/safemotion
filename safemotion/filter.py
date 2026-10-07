"""The safety filter: sits between whatever plans a motion and whatever sends it to the robot.
Keeps commands inside the joint range (with a margin), limits speed AND acceleration against real elapsed time,
refuses bad numbers, notices a robot that stops following, and latches an emergency stop that only an explicit reset clears.
(c) 2026 Ali Amini"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .errors import SafeMotionError
from .limits import JointLimit


@dataclass(frozen=True)
class Report:
    clamped: tuple[str, ...] = ()
    rate_limited: tuple[str, ...] = ()
    fault: str | None = None
    estopped: bool = False
    estop_reason: str | None = None


class SafetyFilter:
    def __init__(self, limits: dict[str, JointLimit], dt: float, max_speed_fraction: float = 0.25,
                 max_accel_fraction: float | None = 4.0, margin_fraction: float = 0.02,
                 max_tracking_error_fraction: float = 0.2, max_faults: int = 3):
        """Speed and acceleration limits are fractions of each joint's range (per second, per second squared)."""
        if not limits:
            raise SafeMotionError("SM-E30", "no joints to protect")
        if not (math.isfinite(dt) and dt > 0):
            raise SafeMotionError("SM-E31", "dt must be a positive number")
        if not (0 < max_speed_fraction <= 1):
            raise SafeMotionError("SM-E32", "max_speed_fraction must be in (0, 1]")
        if not (0 <= margin_fraction <= 0.25):
            raise SafeMotionError("SM-E33", "margin_fraction must be in [0, 0.25]")
        if not (0 < max_tracking_error_fraction <= 1):
            raise SafeMotionError("SM-E34", "max_tracking_error_fraction must be in (0, 1]")
        if max_faults < 1:
            raise SafeMotionError("SM-E35", "max_faults must be at least 1")
        if max_accel_fraction is not None and not (math.isfinite(max_accel_fraction) and max_accel_fraction > 0):
            raise SafeMotionError("SM-E36", "max_accel_fraction must be a positive number or None")
        self.limits = dict(limits)
        self.dt = dt
        self.max_speed = {n: l.span * max_speed_fraction for n, l in self.limits.items()}
        self.max_accel = {n: (l.span * max_accel_fraction if max_accel_fraction is not None else math.inf) for n, l in self.limits.items()}
        self.max_step = {n: v * dt for n, v in self.max_speed.items()}        # at the nominal tick
        self.margin = {n: l.span * margin_fraction for n, l in self.limits.items()}
        self.max_tracking_error = {n: l.span * max_tracking_error_fraction for n, l in self.limits.items()}
        self.max_faults = max_faults
        self._last: dict[str, float] | None = None
        self._vel: dict[str, float] = {n: 0.0 for n in self.limits}
        self._last_time: float | None = None
        self._faults = 0
        self.estopped = False
        self.estop_reason: str | None = None

    # ---- emergency stop ----
    def estop(self, reason: str) -> None:
        """Latch the stop. The filter then only repeats the last command; use the runner's on_estop hook to put the
        robot into its own damping or passive mode, because only your adapter knows how to do that."""
        self.estopped = True
        self.estop_reason = reason

    def reset(self, state: dict[str, float]) -> None:
        """Clear the emergency stop. The next command starts from where the robot actually is, so nothing jumps."""
        self._check_state(state)
        self._last = {n: float(state[n]) for n in self.limits}
        self._vel = {n: 0.0 for n in self.limits}
        self._last_time = None
        self._faults = 0
        self.estopped = False
        self.estop_reason = None

    def _check_state(self, state: dict[str, float]) -> None:
        missing = [n for n in self.limits if n not in state]
        if missing:
            raise SafeMotionError("SM-E21", f"state is missing joints: {missing[:5]}")
        if not all(math.isfinite(state[n]) for n in self.limits):
            raise SafeMotionError("SM-E22", "state contains a non-finite number")

    def _hold(self, state: dict[str, float]) -> dict[str, float]:
        return dict(self._last) if self._last is not None else {n: float(state[n]) for n in self.limits}

    # ---- main entry ----
    def apply(self, target: dict[str, float], state: dict[str, float], check_tracking: bool = True, now: float | None = None):
        """Return (command for every protected joint, Report). Joints missing from `target` are held in place.

        Pass `now` (seconds, a clock that never goes backwards) in every real-time loop: the speed and acceleration limits
        are then enforced against the REAL time since the previous command, even if the loop runs fast or stalls.
        Without `now` the filter assumes exactly one `dt` per call."""
        unknown = [n for n in target if n not in self.limits]
        if unknown:
            raise SafeMotionError("SM-E20", f"target names unknown joints: {unknown[:5]}")
        if now is not None and not (isinstance(now, (int, float)) and math.isfinite(now)):
            raise SafeMotionError("SM-E23", "now must be a finite number")
        try:
            self._check_state(state)
        except SafeMotionError:
            self.estop("bad_state")          # a broken state feed is a fault, not something to guess around
            if self._last is None:
                raise
            return dict(self._last), Report(fault="bad_state", estopped=True, estop_reason=self.estop_reason)

        if self.estopped:
            return self._hold(state), Report(estopped=True, estop_reason=self.estop_reason)

        if check_tracking and self._last is not None:
            for n in self.limits:
                if abs(self._last[n] - state[n]) > self.max_tracking_error[n]:
                    self.estop(f"tracking_error:{n}")
                    return dict(self._last), Report(fault="tracking_error", estopped=True, estop_reason=self.estop_reason)

        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in target.values()):
            self._faults += 1
            if self._faults >= self.max_faults:
                self.estop("repeated_bad_targets")
            return self._hold(state), Report(fault="non_finite_target", estopped=self.estopped, estop_reason=self.estop_reason)
        self._faults = 0

        if now is None or self._last_time is None:
            elapsed = self.dt
        else:
            elapsed = now - self._last_time
        if elapsed <= 0:                       # same or earlier timestamp: no time has passed, so nothing may move
            return self._hold(state), Report(fault="no_time_elapsed")
        e = min(elapsed, 2 * self.dt)          # a stall never earns a bigger step

        ref = self._last if self._last is not None else {n: float(state[n]) for n in self.limits}
        command: dict[str, float] = {}
        vel: dict[str, float] = {}
        clamped: list[str] = []
        limited: list[str] = []
        for n, lim in self.limits.items():
            lo, hi = lim.lo + self.margin[n], lim.hi - self.margin[n]
            if n in target:
                want = float(target[n])
                if want < lo or want > hi:
                    clamped.append(n)
                    want = min(max(want, lo), hi)
            else:
                want = ref[n]
            v_des = (want - ref[n]) / e
            v_hi = min(self.max_speed[n], self._vel[n] + self.max_accel[n] * e)
            v_lo = max(-self.max_speed[n], self._vel[n] - self.max_accel[n] * e)
            if math.isfinite(self.max_accel[n]):
                # Braking-aware: never approach a band edge faster than the joint could still stop in the distance left.
                v_hi = min(v_hi, math.sqrt(2 * self.max_accel[n] * max(hi - ref[n], 0.0)))
                v_lo = max(v_lo, -math.sqrt(2 * self.max_accel[n] * max(ref[n] - lo, 0.0)))
            v = min(max(v_des, v_lo), v_hi)
            if abs(v - v_des) > 1e-12:
                limited.append(n)
            new = ref[n] + v * e
            new = min(max(new, min(lo, ref[n])), max(hi, ref[n]))      # never leave the band; never be dragged further out
            command[n] = new
            vel[n] = (new - ref[n]) / e
        self._last = dict(command)
        self._vel = vel
        if now is not None:
            self._last_time = now
        return command, Report(clamped=tuple(clamped), rate_limited=tuple(limited))
