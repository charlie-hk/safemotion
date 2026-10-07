import math
import random
import os
import sys
import unittest
sys.path.insert(0, os.path.dirname(__file__))

from safemotion import SafeMotionError, SafetyFilter
from helpers import LIMITS

DT = 0.01


def mk(**kw):
    return SafetyFilter(LIMITS, DT, **kw)


def code(fn):
    try:
        fn()
    except SafeMotionError as e:
        return e.code
    return None


class FilterTest(unittest.TestCase):
    def test_clamps_and_rate_limits_a_huge_target(self):
        f = mk()
        state = {"j1": 0.0, "j2": 1.0}
        cmd, rep = f.apply({"j1": 50.0}, state)
        self.assertAlmostEqual(cmd["j1"], 0.0008)          # from rest: acceleration 8 rad/s2 * dt * dt
        self.assertEqual(rep.clamped, ("j1",))
        self.assertEqual(rep.rate_limited, ("j1",))
        self.assertEqual(cmd["j2"], 1.0)                    # joint not in target is held

    def test_never_leaves_the_safe_band_or_the_speed_limit(self):
        f = mk()
        q = {"j1": 0.0, "j2": 1.0}
        for _ in range(400):
            before = dict(q)
            cmd, _ = f.apply({"j1": 99.0, "j2": -99.0}, q)
            for n, lim in LIMITS.items():
                self.assertLessEqual(cmd[n], lim.hi - lim.span * 0.02 + 1e-12)
                self.assertGreaterEqual(cmd[n], lim.lo + lim.span * 0.02 - 1e-12)
                self.assertLessEqual(abs(cmd[n] - before[n]), f.max_step[n] + 1e-12)
            q = dict(cmd)
        self.assertAlmostEqual(q["j1"], 0.96)
        self.assertAlmostEqual(q["j2"], 0.04)

    def test_a_state_outside_the_band_is_walked_back_slowly(self):
        f = mk()
        cmd, _ = f.apply({"j1": 0.0}, {"j1": 1.0, "j2": 1.0})
        self.assertAlmostEqual(cmd["j1"], 0.9992)

    def test_input_checks(self):
        f = mk()
        s = {"j1": 0.0, "j2": 1.0}
        self.assertEqual(code(lambda: f.apply({"zz": 0.0}, s)), "SM-E20")
        self.assertEqual(code(lambda: f.apply({"j1": 0.0}, {"j1": 0.0})), "SM-E21")
        self.assertEqual(code(lambda: f.apply({"j1": 0.0}, {"j1": math.nan, "j2": 1.0})), "SM-E22")

    def test_bad_targets_hold_position_and_repeat_offenders_latch_estop(self):
        f = mk(max_faults=3)
        s = {"j1": 0.2, "j2": 1.0}
        cmd0, _ = f.apply({"j1": 0.2}, s)
        for i in range(2):
            cmd, rep = f.apply({"j1": math.nan}, s)
            self.assertEqual(cmd, cmd0)
            self.assertEqual(rep.fault, "non_finite_target")
            self.assertFalse(rep.estopped)
        _, rep = f.apply({"j1": math.inf}, s)
        self.assertTrue(rep.estopped)
        self.assertEqual(f.estop_reason, "repeated_bad_targets")

    def test_a_good_tick_resets_the_fault_counter(self):
        f = mk(max_faults=2)
        s = {"j1": 0.0, "j2": 1.0}
        f.apply({"j1": math.nan}, s)
        f.apply({"j1": 0.0}, s)
        _, rep = f.apply({"j1": math.nan}, s)
        self.assertFalse(rep.estopped)

    def test_estop_latches_until_an_explicit_reset(self):
        f = mk()
        s = {"j1": 0.0, "j2": 1.0}
        f.apply({"j1": 0.0}, s)
        f.estop("operator")
        cmd, rep = f.apply({"j1": 0.5}, s)
        self.assertTrue(rep.estopped)
        self.assertEqual(cmd["j1"], 0.0)
        f.apply({"j1": 0.5}, s)
        self.assertTrue(f.estopped)
        f.reset({"j1": 0.3, "j2": 1.1})
        self.assertFalse(f.estopped)
        cmd, _ = f.apply({"j1": 0.3}, {"j1": 0.3, "j2": 1.1})
        self.assertEqual(cmd["j1"], 0.3)
        self.assertEqual(cmd["j2"], 1.1)           # starts from where the robot is: no jump

    def test_a_robot_that_stops_following_triggers_estop(self):
        f = mk()
        s = {"j1": 0.0, "j2": 1.0}
        f.apply({"j1": 0.5}, s)
        far = {"j1": 0.0 + 0.5, "j2": 1.0}
        for _ in range(300):
            f.apply({"j1": 0.9}, {"j1": 0.0, "j2": 1.0})
            if f.estopped:
                break
        self.assertTrue(f.estopped)
        self.assertTrue(f.estop_reason.startswith("tracking_error"))
        self.assertEqual(far["j1"], 0.5)

    def test_a_broken_state_feed_latches_estop(self):
        f = mk()
        f.apply({"j1": 0.0}, {"j1": 0.0, "j2": 1.0})
        cmd, rep = f.apply({"j1": 0.1}, {"j1": math.nan, "j2": 1.0})
        self.assertTrue(rep.estopped)
        self.assertEqual(f.estop_reason, "bad_state")
        self.assertEqual(cmd["j1"], 0.0)

    def test_constructor_validation(self):
        for kw, c in [({"dt": 0}, "SM-E31"), ({"dt": math.nan}, "SM-E31"), ({"max_speed_fraction": 0}, "SM-E32"),
                      ({"max_speed_fraction": 1.5}, "SM-E32"), ({"margin_fraction": 0.5}, "SM-E33"),
                      ({"max_tracking_error_fraction": 0}, "SM-E34"), ({"max_faults": 0}, "SM-E35"),
                      ({"max_accel_fraction": 0}, "SM-E36"), ({"max_accel_fraction": math.inf}, "SM-E36")]:
            base = {"limits": LIMITS, "dt": DT}
            base.update(kw)
            self.assertEqual(code(lambda: SafetyFilter(**base)), c, kw)
        self.assertEqual(code(lambda: SafetyFilter({}, DT)), "SM-E30")

    def test_random_session_stays_inside_policy(self):
        rnd = random.Random(2026)
        f = mk()
        q = {"j1": 0.0, "j2": 1.0}
        for _ in range(3000):
            target = {n: rnd.choice([rnd.uniform(-5, 5), math.nan, 1e12, -1e12, 0.0]) for n in rnd.sample(["j1", "j2"], rnd.randint(0, 2))}
            before = dict(q)
            cmd, rep = f.apply(target, q)
            if rep.estopped:
                f.reset(q)
                continue
            for n, lim in LIMITS.items():
                self.assertTrue(lim.lo <= cmd[n] <= lim.hi)
                self.assertLessEqual(abs(cmd[n] - before[n]), f.max_step[n] + 1e-12)
            q = dict(cmd)

    # ---- real elapsed time and acceleration (found in the audit) ----
    def test_a_loop_that_runs_faster_than_dt_is_still_held_to_the_speed_limit(self):
        f = mk(max_accel_fraction=None)                    # pure speed limit: 0.5 rad/s for a span of 2
        q = {"j1": 0.0, "j2": 1.0}
        for k in range(1, 101):                            # 100 calls, 1 ms apart = 0.1 s of real time
            cmd, _ = f.apply({"j1": 1.9}, q, now=k * 0.001)
            q = dict(cmd)
        self.assertLessEqual(q["j1"], 0.5 * (0.099 + DT) + 1e-9)        # speed * real time, plus the first call's assumed tick

    def test_without_now_the_filter_assumes_one_dt_per_call(self):
        f = mk(max_accel_fraction=None)
        cmd, _ = f.apply({"j1": 1.9}, {"j1": 0.0, "j2": 1.0})
        self.assertAlmostEqual(cmd["j1"], 0.005)

    def test_a_stall_never_earns_a_bigger_step(self):
        f = mk(max_accel_fraction=None)
        q = {"j1": 0.0, "j2": 1.0}
        f.apply({"j1": 1.9}, q, now=0.0)
        cmd, _ = f.apply({"j1": 1.9}, q, now=5.0)          # five seconds later
        self.assertLessEqual(cmd["j1"] - 0.005, 2 * DT * 0.5 + 1e-9)

    def test_same_or_earlier_timestamps_hold_position(self):
        f = mk()
        q = {"j1": 0.2, "j2": 1.0}
        c0, _ = f.apply({"j1": 0.8}, q, now=1.0)
        for t in (1.0, 0.5):
            cmd, rep = f.apply({"j1": 0.8}, q, now=t)
            self.assertEqual(cmd, c0)
            self.assertEqual(rep.fault, "no_time_elapsed")
            self.assertFalse(rep.estopped)
        self.assertEqual(code(lambda: f.apply({"j1": 0.0}, q, now=math.nan)), "SM-E23")
        self.assertEqual(code(lambda: f.apply({"j1": 0.0}, q, now="x")), "SM-E23")

    def test_acceleration_is_limited_and_the_band_still_holds(self):
        f = mk()                                            # accel 8 rad/s2, speed 0.5 rad/s
        q = {"j1": 0.0, "j2": 1.0}
        prev_v = 0.0
        worst_decel = 0.0
        reached_top_speed = False
        for k in range(1, 400):
            before = q["j1"]
            cmd, _ = f.apply({"j1": 99.0}, q, now=k * DT)
            v = (cmd["j1"] - before) / DT
            self.assertLessEqual(v - prev_v, 8.0 * DT + 1e-9, f"speeding up too fast at tick {k}")
            worst_decel = max(worst_decel, prev_v - v)
            self.assertLessEqual(abs(v), 0.5 + 1e-9)
            self.assertLessEqual(cmd["j1"], 0.96 + 1e-12)
            reached_top_speed = reached_top_speed or abs(v - 0.5) < 1e-9
            prev_v, q = v, dict(cmd)
        self.assertTrue(reached_top_speed)
        self.assertAlmostEqual(q["j1"], 0.96, places=6)
        # Braking is planned ahead, so there is no dead stop at the band edge. Because time moves in ticks, the very last
        # tick can need up to about twice the configured deceleration; that is the honest bound, and it is still far gentler
        # than a stop from full speed in one tick (which would be 0.5 rad/s, about 6x the bound below).
        self.assertLessEqual(worst_decel, 8.0 * DT * 2.2)
        self.assertLess(worst_decel, 0.5 / 2.5)

    def test_a_joint_without_a_target_slows_down_smoothly_instead_of_stopping_dead(self):
        f = mk()
        q = {"j1": 0.0, "j2": 1.0}
        for k in range(1, 30):
            cmd, _ = f.apply({"j1": 99.0}, q, now=k * DT)
            q = dict(cmd)
        v_before = (q["j1"] - f._last["j1"]) / DT
        c1, _ = f.apply({}, q, now=30 * DT)
        step1 = c1["j1"] - q["j1"]
        self.assertGreater(step1, 0)                        # still coasting a little
        self.assertLess(step1 / DT, 0.5)                    # but slower than top speed

    def test_random_session_with_jittery_time_stays_inside_policy(self):
        rnd = random.Random(7)
        f = mk()
        q = {"j1": 0.0, "j2": 1.0}
        t = 0.0
        for _ in range(3000):
            t += rnd.choice([0.0005, DT, DT, 3 * DT, 0.0])
            target = {n: rnd.choice([rnd.uniform(-5, 5), math.nan, 1e12]) for n in rnd.sample(["j1", "j2"], rnd.randint(0, 2))}
            before = dict(q)
            cmd, rep = f.apply(target, q, now=t)
            if rep.estopped:
                f.reset(q)
                continue
            for n, lim in LIMITS.items():
                self.assertTrue(lim.lo <= cmd[n] <= lim.hi)
                self.assertLessEqual(abs(cmd[n] - before[n]), f.max_speed[n] * 2 * DT + 1e-9)
            q = dict(cmd)

    def test_reset_starts_from_rest_not_from_the_speed_the_joint_had_before_the_stop(self):
        f = mk()
        q = {"j1": 0.0, "j2": 1.0}
        for k in range(1, 40):                                # build up to top speed
            cmd, _ = f.apply({"j1": 99.0}, q, now=k * DT)
            q = dict(cmd)
        self.assertGreater(f._vel["j1"], 0.4)
        f.estop("operator")
        f.reset(q)
        cmd, _ = f.apply({"j1": 99.0}, q, now=100.0)          # a long time later
        self.assertAlmostEqual(cmd["j1"] - q["j1"], 8.0 * DT * DT)    # first step from rest, not a continuation


if __name__ == "__main__":
    unittest.main()
