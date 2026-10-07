import os
import sys
import unittest
sys.path.insert(0, os.path.dirname(__file__))

from safemotion import Keyframe, SafeMotionError, Sequence, load_sequence
from helpers import LIMITS


def code(fn):
    try:
        fn()
    except SafeMotionError as e:
        return e.code
    return None


def seq(**kw):
    return Sequence([Keyframe(0.0, {"j1": 0.0, "j2": 1.0}), Keyframe(4.0, {"j1": 0.4, "j2": 1.0})], **kw)


class SequenceTest(unittest.TestCase):
    def test_interpolation_endpoints_and_middle(self):
        for mode in ("linear", "smoothstep"):
            s = seq(interpolation=mode)
            self.assertEqual(s.sample(0.0)["j1"], 0.0)
            self.assertAlmostEqual(s.sample(2.0)["j1"], 0.2)
            self.assertAlmostEqual(s.sample(4.0)["j1"], 0.4)
            self.assertAlmostEqual(s.sample(-5)["j1"], 0.0)
            self.assertAlmostEqual(s.sample(99)["j1"], 0.4)
        self.assertLess(seq().sample(1.0)["j1"], seq(interpolation="linear").sample(1.0)["j1"])   # smoothstep starts slower
        self.assertEqual(seq().duration, 4.0)

    def test_validation(self):
        a, b = {"j1": 0.0}, {"j1": 1.0}
        self.assertEqual(code(lambda: Sequence([Keyframe(0, a)])), "SM-E41")
        self.assertEqual(code(lambda: Sequence([Keyframe(0, a), Keyframe(0, b)])), "SM-E43")
        self.assertEqual(code(lambda: Sequence([Keyframe(1, a), Keyframe(0, b)])), "SM-E43")
        self.assertEqual(code(lambda: Sequence([Keyframe(0, a), Keyframe(float("nan"), b)])), "SM-E43")
        self.assertEqual(code(lambda: Sequence([Keyframe(0, a), Keyframe(1, {"j2": 0.0})])), "SM-E44")
        self.assertEqual(code(lambda: Sequence([Keyframe(0, a), Keyframe(1, {"j1": float("inf")})])), "SM-E45")
        self.assertEqual(code(lambda: Sequence([Keyframe(0, {}), Keyframe(1, {})])), "SM-E42")
        self.assertEqual(code(lambda: Sequence([Keyframe(0, a), Keyframe(1, b)], interpolation="cubic")), "SM-E40")

    def test_preflight_accepts_a_gentle_sequence(self):
        self.assertEqual(seq().preflight(LIMITS), [])

    def test_preflight_reports_range_speed_and_unknown_joints(self):
        out_of_range = Sequence([Keyframe(0, {"j1": 0.0}), Keyframe(10, {"j1": 1.0})])
        self.assertTrue(any("outside the safe band" in p for p in out_of_range.preflight(LIMITS)))
        too_fast = Sequence([Keyframe(0, {"j1": 0.0}), Keyframe(0.5, {"j1": 0.5})])
        self.assertTrue(any("peak speed" in p for p in too_fast.preflight(LIMITS)))
        unknown = Sequence([Keyframe(0, {"zz": 0.0}), Keyframe(10, {"zz": 0.1})])
        self.assertEqual(unknown.preflight(LIMITS), ["unknown joint 'zz'"])

    def test_smoothstep_peak_is_counted(self):
        # average speed 0.1/s is under the 0.5/s allowance, but the smoothstep peak 0.15/s still is; use a tighter policy
        s = seq()
        self.assertEqual(s.preflight(LIMITS, max_speed_fraction=0.25), [])
        self.assertTrue(s.preflight(LIMITS, max_speed_fraction=0.06))
        lin = seq(interpolation="linear")
        self.assertEqual(lin.preflight(LIMITS, max_speed_fraction=0.06), [])   # 0.1/s average, 0.12/s allowed

    def test_load_sequence(self):
        ok = '{"keyframes":[{"t":0,"pose":{"j1":0}},{"t":1,"pose":{"j1":0.1}}]}'
        self.assertEqual(load_sequence(ok).duration, 1)
        for text in ["not json", "[]", '{"keyframes":{}}', '{"keyframes":[{"pose":{}}]}', '{"keyframes":[1,2]}']:
            self.assertEqual(code(lambda: load_sequence(text)), "SM-E47", text)
        self.assertEqual(code(lambda: load_sequence("x" * 1_100_000)), "SM-E46")


if __name__ == "__main__":
    unittest.main()
