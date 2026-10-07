import os
import sys
import unittest
sys.path.insert(0, os.path.dirname(__file__))

from safemotion import FakeRobot, Keyframe, Recorder, SafetyFilter, Sequence, realtime, run_sequence
from helpers import LIMITS

DT = 0.01


def slow():
    return Sequence([Keyframe(0, {"j1": 0.0, "j2": 1.0}), Keyframe(2, {"j1": 0.4, "j2": 1.2}), Keyframe(4, {"j1": 0.0, "j2": 1.0})])


def setup(**robot_kw):
    return FakeRobot({"j1": 0.0, "j2": 1.0}, DT, **robot_kw), SafetyFilter(LIMITS, DT)


class RunnerTest(unittest.TestCase):
    def test_a_gentle_sequence_runs_to_the_end_and_the_robot_follows(self):
        robot, filt = setup()
        res = run_sequence(slow(), robot, filt, DT)
        self.assertTrue(res.completed)
        self.assertEqual(res.steps, 401)
        self.assertEqual(len(robot.sent), 401)
        self.assertAlmostEqual(robot.q["j1"], 0.0, delta=0.02)

    def test_everything_sent_respects_range_and_speed(self):
        robot, filt = setup()
        fast = Sequence([Keyframe(0, {"j1": 0.0, "j2": 1.0}), Keyframe(0.2, {"j1": 3.0, "j2": -3.0})])
        run_sequence(fast, robot, filt, DT, recorder=Recorder())
        prev = {"j1": 0.0, "j2": 1.0}
        for cmd in robot.sent:
            for n, lim in LIMITS.items():
                self.assertTrue(lim.lo <= cmd[n] <= lim.hi)
                self.assertLessEqual(abs(cmd[n] - prev[n]), filt.max_step[n] + 1e-12)
            prev = cmd

    def test_dry_run_computes_and_records_but_never_sends(self):
        robot, filt = setup()
        rec = Recorder()
        res = run_sequence(slow(), robot, filt, DT, recorder=rec, dry_run=True)
        self.assertTrue(res.completed)
        self.assertEqual(robot.sent, [])
        self.assertEqual(len(rec.rows), 401 * 2)

    def test_a_frozen_state_feed_stops_the_run(self):
        robot, filt = setup()
        robot.frozen = True
        res = run_sequence(slow(), robot, filt, DT, stale_after=0.05)
        self.assertFalse(res.completed)
        self.assertEqual(res.reason, "stale_state")
        self.assertLess(len(robot.sent), 10)

    def test_a_blocked_joint_stops_the_run_with_a_tracking_error(self):
        robot, filt = setup(blocked=("j1",))
        wide = Sequence([Keyframe(0, {"j1": 0.0, "j2": 1.0}), Keyframe(4, {"j1": 0.8, "j2": 1.0})])
        res = run_sequence(wide, robot, filt, DT)
        self.assertFalse(res.completed)
        self.assertTrue(res.reason.startswith("tracking_error:j1"))

    def test_the_recording_flags_what_the_filter_did(self):
        robot, filt = setup()
        rec = Recorder()
        fast = Sequence([Keyframe(0, {"j1": 0.0}), Keyframe(0.1, {"j1": 3.0})])
        run_sequence(fast, robot, filt, DT, recorder=rec)
        self.assertTrue(any("clamped:j1" in r[6] for r in rec.rows))
        self.assertTrue(any("rate_limited:j1" in r[6] for r in rec.rows))
        csv = rec.to_csv().splitlines()
        self.assertEqual(csv[0], "step,t,joint,target,command,state,flags")
        self.assertEqual(len(csv), 1 + len(rec.rows))

    def test_csv_cells_are_safe_for_spreadsheets(self):
        rec = Recorder()
        rec.add(0, 0.0, {"=cmd": 1.0}, {"=cmd": 1.0, "a,b": -0.5}, {"=cmd": 0.0}, "x")
        text = rec.to_csv()
        self.assertIn("'=cmd", text)
        self.assertIn('"a,b"', text)
        self.assertIn(",-0.5,", text)            # numbers keep their minus sign

    # ---- found in the audit ----
    def test_the_final_keyframe_is_reached_even_when_the_duration_is_not_a_multiple_of_dt(self):
        robot, filt = setup()
        seq = Sequence([Keyframe(0, {"j1": 0.0, "j2": 1.0}), Keyframe(1.005, {"j1": 0.3, "j2": 1.0})])
        res = run_sequence(seq, robot, filt, DT)
        self.assertTrue(res.completed)
        self.assertAlmostEqual(robot.sent[-1]["j1"], 0.3, places=9)
        exact = run_sequence(Sequence([Keyframe(0, {"j1": 0.0, "j2": 1.0}), Keyframe(1.0, {"j1": 0.3, "j2": 1.0})]), *setup(), DT)
        self.assertEqual(exact.steps, 101)

    def test_the_filter_gets_the_real_clock_so_a_fast_loop_cannot_beat_the_speed_limit(self):
        robot, filt = setup()
        fast = Sequence([Keyframe(0, {"j1": 0.0, "j2": 1.0}), Keyframe(0.05, {"j1": 0.9, "j2": 1.0})])
        run_sequence(fast, robot, filt, DT, clock=lambda i: i * 0.001)       # the loop really runs 10x faster than dt
        steps = len(robot.sent)
        moved = robot.sent[-1]["j1"]
        self.assertLessEqual(moved, 0.5 * steps * 0.001 + 0.5 * 0.001 + 1e-9)

    def test_on_estop_is_called_once_with_the_reason_and_pace_every_step(self):
        robot, filt = setup(blocked=("j1",))
        wide = Sequence([Keyframe(0, {"j1": 0.0, "j2": 1.0}), Keyframe(4, {"j1": 0.8, "j2": 1.0})])
        reasons, paced = [], []
        res = run_sequence(wide, robot, filt, DT, on_estop=reasons.append, pace=paced.append)
        self.assertFalse(res.completed)
        self.assertEqual(len(reasons), 1)
        self.assertTrue(reasons[0].startswith("tracking_error:j1"))
        self.assertEqual(paced, list(range(len(paced))))
        self.assertEqual(len(paced), res.steps)

    def test_on_estop_is_not_called_on_a_normal_run(self):
        robot, filt = setup()
        called = []
        run_sequence(slow(), robot, filt, DT, on_estop=called.append)
        self.assertEqual(called, [])

    def test_realtime_helper_paces_to_the_tick_grid(self):
        now = [100.0]
        naps = []
        clock, pace = realtime(0.01, monotonic=lambda: now[0], sleep=lambda s: (naps.append(round(s, 6)), now.__setitem__(0, now[0] + s)))
        now[0] += 0.004
        self.assertAlmostEqual(clock(0), 0.004)
        pace(0)                                   # next tick is at 0.010 -> sleep 0.006
        pace(1)                                   # now exactly 0.010; next tick at 0.020 -> sleep 0.010
        self.assertEqual(naps, [0.006, 0.01])
        now[0] += 1.0                             # a long stall: no negative sleep
        pace(2)
        self.assertEqual(len(naps), 2)


if __name__ == "__main__":
    unittest.main()
