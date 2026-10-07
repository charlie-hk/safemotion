"""A regression test built from the ranges the tool printed for the real G1 29-DoF model file on 2026-10-07
(rounded to 4 decimals, radians). It is a fixture of those printed numbers, not the model file itself."""
import json
import os
import unittest

from safemotion import load_sequence, parse_mjcf_limits

HERE = os.path.dirname(os.path.dirname(__file__))

PRINTED = """left_hip_pitch_joint -2.5307 2.8798
left_hip_roll_joint -0.5236 2.9671
left_hip_yaw_joint -2.7576 2.7576
left_knee_joint -0.0873 2.8798
left_ankle_pitch_joint -0.8727 0.5236
left_ankle_roll_joint -0.2618 0.2618
right_hip_pitch_joint -2.5307 2.8798
right_hip_roll_joint -2.9671 0.5236
right_hip_yaw_joint -2.7576 2.7576
right_knee_joint -0.0873 2.8798
right_ankle_pitch_joint -0.8727 0.5236
right_ankle_roll_joint -0.2618 0.2618
waist_yaw_joint -2.6180 2.6180
waist_roll_joint -0.5200 0.5200
waist_pitch_joint -0.5200 0.5200
left_shoulder_pitch_joint -3.0892 2.6704
left_shoulder_roll_joint -1.5882 2.2515
left_shoulder_yaw_joint -2.6180 2.6180
left_elbow_joint -1.0472 2.0944
left_wrist_roll_joint -1.9722 1.9722
left_wrist_pitch_joint -1.6144 1.6144
left_wrist_yaw_joint -1.6144 1.6144
right_shoulder_pitch_joint -3.0892 2.6704
right_shoulder_roll_joint -2.2515 1.5882
right_shoulder_yaw_joint -2.6180 2.6180
right_elbow_joint -1.0472 2.0944
right_wrist_roll_joint -1.9722 1.9722
right_wrist_pitch_joint -1.6144 1.6144
right_wrist_yaw_joint -1.6144 1.6144"""


def xml():
    joints = "".join(f"<joint name='{n}' range='{lo} {hi}'/>" for n, lo, hi in (l.split() for l in PRINTED.splitlines()))
    return f"<mujoco><compiler angle='radian'/><worldbody><body><joint type='free' name='floating_base_joint'/><body>{joints}</body></body></worldbody></mujoco>"


class G1FixtureTest(unittest.TestCase):
    def test_all_29_joints_and_the_mirror_symmetry(self):
        lim = parse_mjcf_limits(xml())
        self.assertEqual(len(lim), 29)
        self.assertAlmostEqual(lim["left_hip_roll_joint"].lo, -lim["right_hip_roll_joint"].hi)
        self.assertAlmostEqual(lim["left_shoulder_roll_joint"].hi, -lim["right_shoulder_roll_joint"].lo)

    def test_patterns_select_the_body_parts(self):
        legs = ["*hip*", "*knee*", "*ankle*"]
        upper = parse_mjcf_limits(xml(), exclude=legs)
        self.assertEqual(len(upper), 17)                                    # 3 waist + 14 arm joints
        self.assertTrue(all(("shoulder" in n or "elbow" in n or "wrist" in n or n.startswith("waist")) for n in upper))
        arms = parse_mjcf_limits(xml(), include=["*shoulder*", "*elbow*", "*wrist*"])
        self.assertEqual(len(arms), 14)
        left = parse_mjcf_limits(xml(), include=["left_*"], exclude=["*hip*", "*knee*", "*ankle*"])
        self.assertEqual(len(left), 7)
        self.assertEqual(len(parse_mjcf_limits(xml(), include=["left_elbow_joint"])), 1)     # exact names still work
        self.assertEqual(len(parse_mjcf_limits(xml(), exclude=["left_elbow_joint"])), 28)

    def test_excluded_joints_are_not_checked_for_a_range(self):
        bad = "<mujoco><worldbody><body><joint name='hand_x'/><joint name='arm' range='0 1'/></body></worldbody></mujoco>"
        self.assertEqual(list(parse_mjcf_limits(bad, exclude=["hand_*"])), ["arm"])
        self.assertEqual(list(parse_mjcf_limits(bad, include=["arm"])), ["arm"])

    def test_the_left_arm_example_passes_preflight_on_the_real_ranges(self):
        lim = parse_mjcf_limits(xml(), include=["left_*"], exclude=["*hip*", "*knee*", "*ankle*"])
        with open(os.path.join(HERE, "examples", "g1_left_arm_example.json"), encoding="utf-8") as f:
            text = f.read()
        self.assertIn("_note", json.loads(text))
        seq = load_sequence(text)
        self.assertEqual(seq.preflight(lim), [])
        self.assertTrue(seq.preflight(lim, max_speed_fraction=0.02))        # a much stricter policy does flag it


if __name__ == "__main__":
    unittest.main()
