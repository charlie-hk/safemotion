import math
import unittest

from safemotion import SafeMotionError, parse_mjcf_limits

MJCF = """<mujoco model='t'>
 <compiler angle='radian'/>
 <default><joint range='-1 1'/><default class='arm'><joint range='-2 2'/></default></default>
 <worldbody>
  <body name='base'><joint type='free' name='root'/>
   <body name='a'><joint name='j1'/>
    <body name='b' childclass='arm'><joint name='j2'/><joint name='j3' class='main' range='0 0.5'/></body>
   </body>
  </body>
 </worldbody>
</mujoco>"""


def code(fn):
    try:
        fn()
    except SafeMotionError as e:
        return e.code
    return None


class LimitsTest(unittest.TestCase):
    def test_defaults_classes_childclass_and_free_joint(self):
        lim = parse_mjcf_limits(MJCF)
        self.assertEqual(list(lim), ["j1", "j2", "j3"])
        self.assertEqual((lim["j1"].lo, lim["j1"].hi), (-1.0, 1.0))
        self.assertEqual((lim["j2"].lo, lim["j2"].hi), (-2.0, 2.0))
        self.assertEqual((lim["j3"].lo, lim["j3"].hi), (0.0, 0.5))
        self.assertEqual(lim["j2"].span, 4.0)

    def test_degrees_are_converted_but_slide_joints_are_not(self):
        xml = "<mujoco><worldbody><body><joint name='h' range='-90 90'/><joint name='s' type='slide' range='0 0.3'/></body></worldbody></mujoco>"
        lim = parse_mjcf_limits(xml)
        self.assertAlmostEqual(lim["h"].hi, math.pi / 2)
        self.assertEqual((lim["s"].lo, lim["s"].hi), (0.0, 0.3))
        rad = "<mujoco><compiler angle='radian'/><worldbody><body><joint name='h' range='-90 90'/></body></worldbody></mujoco>"
        self.assertEqual(parse_mjcf_limits(rad)["h"].hi, 90.0)

    def test_fails_closed_instead_of_guessing(self):
        w = "<mujoco><worldbody><body>{}</body></worldbody></mujoco>"
        cases = {
            "SM-E16": w.format("<joint name='x'/>"),
            "SM-E14": w.format("<joint name='x' type='ball' range='0 1'/>"),
            "SM-E15": w.format("<joint range='0 1'/>"),
            "SM-E13": w.format("<joint name='x' class='nope' range='0 1'/>"),
            "SM-E10": w.format("<joint name='x' range='1 0'/>"),
            "SM-E17": w.format("<joint name='x' range='0 1'/><joint name='x' range='0 1'/>"),
            "SM-E18": w.format("<joint type='free' name='r'/>"),
        }
        for expected, xml in cases.items():
            self.assertEqual(code(lambda: parse_mjcf_limits(xml)), expected, expected)
        self.assertEqual(code(lambda: parse_mjcf_limits(w.format("<joint name='x' range='0 1' limited='false'/>"))), "SM-E16")
        self.assertEqual(code(lambda: parse_mjcf_limits(w.format("<joint name='x' range='0 nan'/>"))), "SM-E10")
        self.assertEqual(code(lambda: parse_mjcf_limits(w.format("<joint name='x' range='0 inf'/>"))), "SM-E10")
        self.assertEqual(code(lambda: parse_mjcf_limits(w.format("<joint name='x' range='0'/>"))), "SM-E10")

    def test_exclude_skips_joints_you_do_not_command(self):
        xml = "<mujoco><worldbody><body><joint name='a' range='0 1'/><joint name='hand'/></body></worldbody></mujoco>"
        self.assertEqual(list(parse_mjcf_limits(xml, exclude={"hand"})), ["a"])

    def test_bad_files(self):
        self.assertEqual(code(lambda: parse_mjcf_limits("<!DOCTYPE x><mujoco/>")), "SM-E11")
        self.assertEqual(code(lambda: parse_mjcf_limits("<!ENTITY a 'b'>")), "SM-E11")
        self.assertEqual(code(lambda: parse_mjcf_limits("<mujoco>")), "SM-E12")
        self.assertEqual(code(lambda: parse_mjcf_limits("<robot/>")), "SM-E12")
        self.assertEqual(code(lambda: parse_mjcf_limits("<mujoco><compiler angle='turns'/><worldbody/></mujoco>")), "SM-E12")
        self.assertEqual(code(lambda: parse_mjcf_limits("<mujoco/>")), "SM-E12")

    def test_constructs_that_could_hide_joints_are_refused_not_ignored(self):
        for tag, body in [("include", "<include file='more.xml'/>"), ("frame", "<frame pos='0 0 1'><body><joint name='h' range='0 1'/></body></frame>"),
                          ("replicate", "<replicate count='2'><body><joint name='r' range='0 1'/></body></replicate>"),
                          ("attach", "<attach model='m' body='b' prefix='x'/>")]:
            xml = f"<mujoco><worldbody><body><joint name='a' range='0 1'/></body>{body}</worldbody></mujoco>"
            self.assertEqual(code(lambda: parse_mjcf_limits(xml)), "SM-E19", tag)
        asset = "<mujoco><asset><model name='m' file='m.xml'/></asset><worldbody><body><joint name='a' range='0 1'/></body></worldbody></mujoco>"
        self.assertEqual(code(lambda: parse_mjcf_limits(asset)), "SM-E19")


if __name__ == "__main__":
    unittest.main()
