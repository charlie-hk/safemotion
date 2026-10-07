"""Keyframe sequences with interpolation and a pre-flight check you can run before anything moves.
(c) 2026 Ali Amini"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass

from .errors import SafeMotionError
from .limits import JointLimit

MAX_JSON_BYTES = 1_000_000


@dataclass(frozen=True)
class Keyframe:
    t: float
    pose: dict


class Sequence:
    def __init__(self, keyframes: list[Keyframe], interpolation: str = "smoothstep"):
        if interpolation not in ("linear", "smoothstep"):
            raise SafeMotionError("SM-E40", "interpolation must be 'linear' or 'smoothstep'")
        if len(keyframes) < 2:
            raise SafeMotionError("SM-E41", "a sequence needs at least two keyframes")
        names = set(keyframes[0].pose)
        if not names:
            raise SafeMotionError("SM-E42", "keyframes must name at least one joint")
        prev = -math.inf
        for k in keyframes:
            if not (isinstance(k.t, (int, float)) and math.isfinite(k.t)) or k.t <= prev:
                raise SafeMotionError("SM-E43", "keyframe times must be finite and strictly increasing")
            prev = k.t
            if set(k.pose) != names:
                raise SafeMotionError("SM-E44", "every keyframe must name the same joints")
            if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in k.pose.values()):
                raise SafeMotionError("SM-E45", "joint values must be finite numbers")
        self.keyframes = list(keyframes)
        self.interpolation = interpolation

    @property
    def start(self) -> float:
        return self.keyframes[0].t

    @property
    def duration(self) -> float:
        return self.keyframes[-1].t - self.keyframes[0].t

    def sample(self, t: float) -> dict:
        ks = self.keyframes
        t = min(max(t, ks[0].t), ks[-1].t)
        for a, b in zip(ks, ks[1:]):
            if t <= b.t:
                alpha = (t - a.t) / (b.t - a.t)
                if self.interpolation == "smoothstep":
                    alpha = alpha * alpha * (3 - 2 * alpha)
                return {n: a.pose[n] + (b.pose[n] - a.pose[n]) * alpha for n in a.pose}
        return dict(ks[-1].pose)

    def preflight(self, limits: dict[str, JointLimit], max_speed_fraction: float = 0.25, margin_fraction: float = 0.02) -> list[str]:
        """Return human-readable problems. An empty list means the sequence respects the range and speed policy."""
        problems: list[str] = []
        names = list(self.keyframes[0].pose)
        for n in names:
            if n not in limits:
                problems.append(f"unknown joint {n!r}")
        peak = 1.5 if self.interpolation == "smoothstep" else 1.0
        for n in names:
            lim = limits.get(n)
            if lim is None:
                continue
            lo, hi = lim.lo + lim.span * margin_fraction, lim.hi - lim.span * margin_fraction
            for k in self.keyframes:
                v = k.pose[n]
                if v < lo or v > hi:
                    problems.append(f"{n}: value {v:.4f} at t={k.t} is outside the safe band [{lo:.4f}, {hi:.4f}]")
            allowed = lim.span * max_speed_fraction
            for a, b in zip(self.keyframes, self.keyframes[1:]):
                speed = peak * abs(b.pose[n] - a.pose[n]) / (b.t - a.t)
                if speed > allowed + 1e-12:
                    problems.append(f"{n}: peak speed {speed:.4f}/s between t={a.t} and t={b.t} is above the allowed {allowed:.4f}/s")
        return problems


def load_sequence(text: str) -> Sequence:
    if len(text.encode("utf-8")) > MAX_JSON_BYTES:
        raise SafeMotionError("SM-E46", "sequence file is too large")
    try:
        data = json.loads(text)
    except json.JSONDecodeError as e:
        raise SafeMotionError("SM-E47", f"sequence file is not valid JSON ({e})") from None
    if not isinstance(data, dict) or not isinstance(data.get("keyframes"), list):
        raise SafeMotionError("SM-E47", "expected an object with a 'keyframes' list")
    frames = []
    for item in data["keyframes"]:
        if not isinstance(item, dict) or not isinstance(item.get("pose"), dict) or "t" not in item:
            raise SafeMotionError("SM-E47", "each keyframe needs 't' and 'pose'")
        frames.append(Keyframe(item["t"], item["pose"]))
    return Sequence(frames, data.get("interpolation", "smoothstep"))
