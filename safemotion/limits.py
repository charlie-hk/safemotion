"""Joint limits read from a MuJoCo (MJCF) model, so numbers come from the robot model and are never typed in by hand.
Fails closed: a joint without a clear range is an error, not a guess.
(c) 2026 Ali Amini"""
from __future__ import annotations

import fnmatch
import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass

from .errors import SafeMotionError


@dataclass(frozen=True)
class JointLimit:
    name: str
    lo: float
    hi: float

    @property
    def span(self) -> float:
        return self.hi - self.lo


_UNSUPPORTED = frozenset({"include", "frame", "replicate", "attach", "model"})


def _two_floats(text: str, what: str) -> tuple[float, float]:
    try:
        vals = [float(p) for p in text.split()]
    except ValueError:
        raise SafeMotionError("SM-E10", f"{what}: range is not made of numbers: {text!r}") from None
    if len(vals) != 2 or not all(math.isfinite(v) for v in vals):
        raise SafeMotionError("SM-E10", f"{what}: range needs exactly two finite numbers: {text!r}")
    return vals[0], vals[1]


def parse_mjcf_limits(xml_text: str, exclude: frozenset[str] | set[str] | list[str] = frozenset(),
                      include: list[str] | None = None) -> dict[str, JointLimit]:
    """Return {joint name: JointLimit} in radians (hinge) or model units (slide).

    Supported: hinge and slide joints, ranges set directly or through <default> classes (and childclass),
    and the <compiler angle> setting. Free joints are skipped. Anything else that would need a guess raises.
    `exclude` and `include` take joint names or shell-style patterns such as "*hip*". Excluded joints are skipped entirely
    (they are not even checked for a range). If `include` is given, only joints matching it are kept.
    """
    if re.search(r"<!\s*(DOCTYPE|ENTITY)", xml_text, re.I):
        raise SafeMotionError("SM-E11", "DOCTYPE and ENTITY are not allowed in the model file")
    try:
        root = ET.fromstring(xml_text)
    except ET.ParseError as e:
        raise SafeMotionError("SM-E12", f"the model file is not valid XML ({e})") from None
    if root.tag != "mujoco":
        raise SafeMotionError("SM-E12", "this is not a MuJoCo model (root element is not <mujoco>)")

    for el in root.iter():
        if el.tag in _UNSUPPORTED:
            raise SafeMotionError("SM-E19", f"the model uses <{el.tag}>, which this reader does not follow, so joints could be missed silently. "
                                           "Pass the single file that defines all the robot's joints (not a scene file that includes it).")

    compiler = root.find("compiler")
    angle = (compiler.get("angle", "degree") if compiler is not None else "degree").lower()
    if angle not in ("degree", "radian"):
        raise SafeMotionError("SM-E12", f"unknown compiler angle unit {angle!r}")
    to_rad = math.pi / 180.0 if angle == "degree" else 1.0

    classes: dict[str, dict[str, str]] = {"main": {}}

    def walk_default(el: ET.Element, inherited: dict[str, str], name: str | None) -> None:
        attrs = dict(inherited)
        j = el.find("joint")
        if j is not None:
            attrs.update(j.attrib)
        if name:
            classes[name] = attrs
        for child in el.findall("default"):
            walk_default(child, attrs, child.get("class"))

    top = root.find("default")
    if top is not None:
        walk_default(top, {}, "main")

    out: dict[str, JointLimit] = {}

    def add(j: ET.Element, cls: str) -> None:
        cname = j.get("class", cls)
        if cname not in classes:
            raise SafeMotionError("SM-E13", f"joint refers to an unknown default class {cname!r}")
        attrs = dict(classes[cname])
        attrs.update(j.attrib)
        jtype = attrs.get("type", "hinge")
        if jtype == "free":
            return
        name = j.get("name")
        if not name:
            raise SafeMotionError("SM-E15", "found a joint without a name; it cannot be commanded by name")
        if any(fnmatch.fnmatchcase(name, pat) for pat in exclude):
            return
        if include is not None and not any(fnmatch.fnmatchcase(name, pat) for pat in include):
            return
        if jtype not in ("hinge", "slide"):
            raise SafeMotionError("SM-E14", f"joint {name!r} has unsupported type {jtype!r} (exclude it if you do not command it)")
        rng = attrs.get("range")
        if rng is None or attrs.get("limited", "auto").lower() == "false":
            raise SafeMotionError("SM-E16", f"joint {name!r} has no range; refusing to guess a limit (exclude it if you do not command it)")
        lo, hi = _two_floats(rng, f"joint {name!r}")
        if not lo < hi:
            raise SafeMotionError("SM-E10", f"joint {name!r}: range must have lo < hi, got {lo} {hi}")
        if jtype == "hinge":
            lo, hi = lo * to_rad, hi * to_rad
        if name in out:
            raise SafeMotionError("SM-E17", f"joint name {name!r} appears twice")
        out[name] = JointLimit(name, lo, hi)

    def visit(body: ET.Element, cls: str) -> None:
        cls = body.get("childclass", cls)
        for j in body.findall("joint"):
            add(j, cls)
        for sub in body.findall("body"):
            visit(sub, cls)

    wb = root.find("worldbody")
    if wb is None:
        raise SafeMotionError("SM-E12", "the model has no <worldbody>")
    visit(wb, "main")
    if not out:
        raise SafeMotionError("SM-E18", "no commandable joints were found")
    return out
