"""A regression test built from the ranges the tool printed for the real Unitree H1 model file (unitree_robots/h1/h1.xml
from the unitree_mujoco repository) on 2026-10-08, run by the author on Windows. It is a fixture of those printed numbers
and of the structure that matters (a free joint, a default class given by childclass, and one joint with no range), not
the model file itself. The author has not run the H1 on a simulator or a robot."""
import unittest

from safemotion import SafeMotionError, parse_mjcf_limits

PRINTED = """left_hip_yaw_joint -0.43 0.43
left_hip_roll_joint -0.43 0.43
left_hip_pitch_joint -3.14 2.53
left_knee_joint -0.26 2.05
left_ankle_joint -0.87 0.52
right_hip_yaw_joint -0.43 0.43
right_hip_roll_joint -0.43 0.43
right_hip_pitch_joint -3.14 2.53
right_knee_joint -0.26 2.05
right_ankle_joint -0.87 0.52
torso_joint -2.35 2.35
left_shoulder_pitch_joint -2.87 2.87
left_shoulder_roll_joint -0.34 3.11
left_shoulder_yaw_joint -1.3 4.45
left_elbow_joint -1.25 2.61
right_shoulder_pitch_joint -2.87 2.87
right_shoulder_roll_joint -3.11 0.34
right_shoulder_yaw_joint -4.45 1.3
right_elbow_joint -1.25 2.61"""


def xml():
    joints = "".join(f"<body><joint name='{n}' range='{lo} {hi}'/>" for n, lo, hi in (l.split() for l in PRINTED.splitlines()))
    closing = "</body>" * len(PRINTED.splitlines())
    return ("<mujoco model='h1'><compiler angle='radian' autolimits='true'/>"
            "<default><default class='h1'><joint damping='1' armature='0.1'/></default></default>"
            "<worldbody><body name='pelvis' childclass='h1'><freejoint/>"
            f"{joints}{closing}"
            "<body name='unused'><joint name='not_use_joint'/></body></body></worldbody></mujoco>")


class H1FixtureTest(unittest.TestCase):
    def test_a_joint_without_a_range_is_refused_by_name(self):
        with self.assertRaises(SafeMotionError) as cm:
            parse_mjcf_limits(xml())
        self.assertEqual(cm.exception.code, "SM-E16")
        self.assertIn("not_use_joint", str(cm.exception))

    def test_excluding_it_gives_the_19_joints_and_the_mirror_symmetry(self):
        lim = parse_mjcf_limits(xml(), exclude=["not_use_joint"])
        self.assertEqual(len(lim), 19)
        self.assertNotIn("not_use_joint", lim)
        self.assertAlmostEqual(lim["left_shoulder_roll_joint"].hi, -lim["right_shoulder_roll_joint"].lo)
        self.assertAlmostEqual(lim["left_shoulder_yaw_joint"].hi, -lim["right_shoulder_yaw_joint"].lo)
        self.assertAlmostEqual(lim["left_elbow_joint"].lo, lim["right_elbow_joint"].lo)

    def test_upper_body_selection(self):
        upper = parse_mjcf_limits(xml(), exclude=["not_use_joint", "*hip*", "*knee*", "*ankle*"])
        self.assertEqual(len(upper), 9)                                     # torso + 8 arm joints
        arms = parse_mjcf_limits(xml(), include=["*shoulder*", "*elbow*"])
        self.assertEqual(len(arms), 8)                                      # only-selected joints are not checked beyond themselves


if __name__ == "__main__":
    unittest.main()
