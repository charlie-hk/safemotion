import os
import unittest

from safemotion import FakeRobot, SafetyFilter, load_sequence, parse_mjcf_limits, run_sequence

HERE = os.path.dirname(os.path.dirname(__file__))


class ExamplesTest(unittest.TestCase):
    def test_toy_example_files_pass_preflight_and_run_end_to_end(self):
        with open(os.path.join(HERE, "examples", "toy_arm.xml"), encoding="utf-8") as f:
            limits = parse_mjcf_limits(f.read())
        with open(os.path.join(HERE, "examples", "toy_wave.json"), encoding="utf-8") as f:
            seq = load_sequence(f.read())
        self.assertEqual(list(limits), ["shoulder", "elbow", "wrist"])
        self.assertEqual(seq.preflight(limits), [])
        dt = 0.01
        robot = FakeRobot({"shoulder": 0.0, "elbow": 0.5, "wrist": 0.0}, dt)
        res = run_sequence(seq, robot, SafetyFilter(limits, dt), dt, stale_after=0.05)
        self.assertTrue(res.completed)


if __name__ == "__main__":
    unittest.main()
