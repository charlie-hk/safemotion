[README.md](https://github.com/user-attachments/files/33192916/README.md)

# safemotion: safety layer for Unitree G1/H1 humanoid joint motion (unofficial)

[![tests](https://github.com/charlie-hk/safemotion/actions/workflows/tests.yml/badge.svg)](https://github.com/charlie-hk/safemotion/actions/workflows/tests.yml)

(c) 2026 Ali Amini. Unofficial and **not affiliated with Unitree Robotics**. Unitree, G1 and H1 are trademarks of their owners.

Pre-flight and safety tools for humanoid robot motion (oriented to Unitree G1/H1), built **simulation first**. Pure Python, no dependencies. Needs Python 3.10 or newer (developed on 3.12 and run by the author on Windows; the included GitHub Actions workflow also runs the tests on Linux and Windows with 3.10 and 3.12).

What it gives you:

- **Joint limits read from the robot model** (a MuJoCo/MJCF file). It fails closed: a joint without a clear range is an error, not a guess,
  and constructs that could hide joints (`<include>`, `<frame>`, `<replicate>`, `<attach>`, `<model>`) are refused, not ignored.
  Pass the single file that defines all the robot's joints. Note: a model's ranges are what the *model* says; they are not
  necessarily the real hardware's safe limits. Check them against the manufacturer's documentation.
- **Safety filter.** Sits between whatever plans a motion and whatever sends it. It keeps every command inside the range (with a margin),
  limits **speed and acceleration against real elapsed time** (pass the clock, so a fast or stalled loop cannot beat the limits),
  plans braking before band edges, refuses non-finite numbers, notices when the robot stops following (tracking error),
  and latches an emergency stop that only an explicit `reset()` clears.
- **Keyframe sequences** with smooth interpolation and a **pre-flight check** (range, peak speed) you can run before anything moves.
- **Runner** with dry-run mode, a stale-feed watchdog, real-time pacing (`realtime(dt)`), an `on_estop` hook and a CSV recorder (formula-safe).
- **Command line:** `python -m safemotion limits MODEL.xml` and `python -m safemotion preflight MODEL.xml SEQUENCE.json`.
  Select joints with names or patterns: `--exclude "*hip*,*knee*,*ankle*"` or `--only "left_*"` (in code: `parse_mjcf_limits(xml, exclude=[...], include=[...])`).

## Try it with a real G1 model file

Take the robot file (not a `scene*.xml` file) from the `unitree_mujoco` repository:

```
python -m safemotion limits path\to\unitree_robots\g1\g1_29dof.xml --exclude "*hip*,*knee*,*ankle*"
python -m safemotion preflight path\to\unitree_robots\g1\g1_29dof.xml examples\g1_left_arm_example.json
```

The example sequence only proves the tools run on real joint names. Its poses are arbitrary, inside the ranges, and have never been run on a simulator or a robot.

### H1

The H1 model file (`unitree_robots/h1/h1.xml`) contains one joint, `not_use_joint`, that has no range. By design the reader refuses it
instead of guessing, so exclude it by name:

```
python -m safemotion limits path\to\unitree_robots\h1\h1.xml --exclude "not_use_joint"
```

This prints the 19 joints with their ranges. Without `--exclude` you get error `SM-E16` naming that joint.

## Try it (no robot, no simulator needed)

```
python -m safemotion limits examples/toy_arm.xml
python -m safemotion preflight examples/toy_arm.xml examples/toy_wave.json
python -m unittest discover -s tests
```

```python
from safemotion import FakeRobot, SafetyFilter, load_sequence, parse_mjcf_limits, run_sequence

limits = parse_mjcf_limits(open("examples/toy_arm.xml").read())
seq = load_sequence(open("examples/toy_wave.json").read())
assert seq.preflight(limits) == []

dt = 0.01
robot = FakeRobot({"shoulder": 0.0, "elbow": 0.5, "wrist": 0.0}, dt)      # a stand-in, not physics
result = run_sequence(seq, robot, SafetyFilter(limits, dt), dt, stale_after=0.05)   # on a real loop also pass clock/pace from realtime(dt)
print(result)
```

## Validation status (read this before relying on it)

| Checked | How |
|---|---|
| The logic (limits, filter, sequences, runner) | 56 automated tests. 30 deliberate code breakages (limit removed, check skipped, and so on) were each caught by a test, checked when there were 53 tests |
| Real Unitree model files | **G1 29-DoF: the model file from the unitree_mujoco repository parses (29 joints, left/right ranges mirror each other), checked by the author on 2026-10-07. H1: parses with `--exclude "not_use_joint"` (19 joints), checked by the author on 2026-10-08.** The 23-DoF G1 and other models: not yet reported. Run `python -m safemotion limits path/to/robot.xml` on your own model and report problems |
| Unitree firmware | **not tested** against any firmware version (including 1.5.x). Ranges come from the published model file and may differ from the limits the robot's firmware enforces |
| A physics simulator | **not run** by the author |
| A real robot | **never** |

The tests use a toy first-order stand-in (`FakeRobot`), not a physics model.

## What is NOT here

- **No adapter for any real Unitree interface.** Write one against your simulator first (it only needs `read_state()` and `send(command)`),
  using the manufacturer's SDK and simulator documentation.
- **The emergency stop only repeats the last command.** Putting a real robot into its own damping or passive mode is robot-specific,
  so your adapter must do it through the `on_estop` hook.
- No handling of coupled or parallel joints (for example some ankle or waist mechanisms), balance, or whole-body dynamics.
  Intended scope: arms and other joints you command while the robot's own controller stays in charge of balance.
- The example model is invented. Use the model file that ships with your simulator.

## Safety

Commanding joints of a real humanoid can damage it or hurt people. Start in simulation, use the manufacturer's guidance
(they recommend suspending the robot on a protective bracket during development, and their teleoperation examples tell you to put the robot into
debug mode first so the built-in motion controller does not fight your commands), keep an emergency stop within reach, and treat
the defaults here as conservative starting points, not as a guarantee. The tracking-error threshold (20 percent of a joint's range by default)
is deliberately loose; tighten it for your robot. A passing pre-flight means the sequence respects a policy; it does not make a motion safe.

## Related projects

Other open-source projects touch the same area, for example `inspect-robots-unitree-g1` (preflight checks and per-joint speed caps for G1 arms),
`unitree_launcher` (a deployment stack with a safety controller and emergency stop) and `unitree-sdk2-mcp` (joint-limit checks before commands).
The author has not compared them in depth. This library's focus: no dependencies, joint limits read straight from the model file with ambiguous
constructs refused, speed and acceleration limits against real elapsed time, and a latched emergency stop.

## Tests

`python -m unittest discover -s tests`: 56 tests. They include a seeded random session that checks no command ever leaves the allowed band or exceeds the speed limit.

## Licence

MIT, see `LICENSE`.
