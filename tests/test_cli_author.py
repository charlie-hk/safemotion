import contextlib
import io
import os
import tempfile
import unittest

import safemotion
from safemotion import author
from safemotion.cli import main

MJCF = "<mujoco><compiler angle='radian'/><worldbody><body><joint name='j1' range='-1 1'/><joint name='j2' range='0 2'/></body></worldbody></mujoco>"
GOOD = '{"keyframes":[{"t":0,"pose":{"j1":0.0,"j2":1.0}},{"t":4,"pose":{"j1":0.4,"j2":1.0}}]}'
BAD = '{"keyframes":[{"t":0,"pose":{"j1":0.0,"j2":1.0}},{"t":0.2,"pose":{"j1":0.9,"j2":1.0}}]}'


def run(args):
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        rc = main(args)
    return rc, out.getvalue(), err.getvalue()


class CliTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.model = os.path.join(self.dir.name, "m.xml")
        with open(self.model, "w") as f:
            f.write(MJCF)

    def tearDown(self):
        self.dir.cleanup()

    def write(self, name, text):
        p = os.path.join(self.dir.name, name)
        with open(p, "w") as f:
            f.write(text)
        return p

    def test_limits_command_lists_joints(self):
        rc, out, _ = run(["limits", self.model])
        self.assertEqual(rc, 0)
        self.assertIn("j1", out)
        self.assertIn("2 joints", out)

    def test_preflight_ok_and_failing(self):
        self.assertEqual(run(["preflight", self.model, self.write("g.json", GOOD)])[0], 0)
        rc, out, _ = run(["preflight", self.model, self.write("b.json", BAD)])
        self.assertEqual(rc, 1)
        self.assertIn("problem(s)", out)

    def test_errors_return_2_with_a_message(self):
        rc, _, err = run(["limits", os.path.join(self.dir.name, "missing.xml")])
        self.assertEqual(rc, 2)
        self.assertIn("error", err)
        rc, _, err = run(["preflight", self.model, self.write("x.json", "not json")])
        self.assertEqual((rc, "SM-E47" in err), (2, True))

    def test_patterns_work_on_the_command_line(self):
        model = self.write("many.xml", "<mujoco><worldbody><body><joint name='left_hip' range='0 1'/><joint name='left_elbow' range='0 1'/><joint name='right_elbow' range='0 1'/></body></worldbody></mujoco>")
        rc, out, _ = run(["limits", model, "--exclude", "*hip*"])
        self.assertEqual((rc, "2 joints" in out, "left_hip" in out), (0, True, False))
        rc, out, _ = run(["limits", model, "--only", "left_*", "--exclude", "*hip*"])
        self.assertEqual((rc, "1 joints" in out, "left_elbow" in out), (0, True, True))
        rc, out, _ = run(["limits", model, "--only", "nothing_*"])
        self.assertEqual(rc, 2)


class AuthorTest(unittest.TestCase):
    def test_notice_verifies_and_detects_edits(self):
        self.assertTrue(safemotion.verify())
        self.assertIn("Ali Amini", safemotion.notice())
        edited = dict(author.AUTHOR, name="Someone Else")
        self.assertFalse(author.verify(edited))
        self.assertIn("modified copy", author.notice(edited))
        self.assertNotIn("Ali Amini", author.notice(edited))


if __name__ == "__main__":
    unittest.main()
