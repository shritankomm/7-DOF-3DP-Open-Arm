# Humanoid Arms — ROS 2 Control & Simulation Software

A **ROS 2 (Jazzy)** software stack for an open-source **7-DOF humanoid
arms**: robot description (URDF/meshes), MoveIt motion planning, RViz/Gazebo
simulation, and a set of Python tools for **kinesthetic capture, replay, hardware
control, and real-time bilateral teleoperation** of the physical arm.

This folder is a self-contained **ROS 2 workspace** built with `colcon` — it holds
two ROS 2 packages (`humanoid_arms` and `humanoid_arms_moveit_config`, each with a
standard `package.xml`). Clone it, build it, and run.

---

## Requirements

- **Ubuntu 24.04**
- **ROS 2 Jazzy**
- **Gazebo Harmonic** (for the physics sim; optional if you only want RViz + MoveIt)
- **Python 3** with the **Feetech servo SDK** (`scservo_sdk`) for any hardware scripts

---

## Packages

| Package | What it is |
|---------|-----------|
| `humanoid_arms` | Robot description (URDF + STL meshes), Gazebo launch, and all hardware/teleop Python scripts (`scripts/`). |
| `humanoid_arms_moveit_config` | MoveIt configuration (SRDF, kinematics, OMPL, controllers, joint limits) and the `demo.launch.py` that brings up RViz + MoveIt with mock controllers. |

> **Two URDFs, by design.** Gazebo uses `humanoid_arms/urdf/humanoid_arms_final.urdf`
> (with the Gazebo control plugin); MoveIt uses
> `humanoid_arms_moveit_config/config/humanoid_arms.urdf` (with the mock-components
> plugin). They describe the same robot but carry different `<plugin>` blocks.

---

## Build

```bash
# from the workspace root (this folder)
rosdep install --from-paths src -y --ignore-src     # install ROS dependencies
colcon build
source install/setup.bash
```

The Feetech SDK is a **pip** package, not a ROS one, so install it separately:

```bash
pip install feetech-servo-sdk        # provides `scservo_sdk`
```

---

## Wayland workaround (important for GUIs)

On Wayland, RViz and Gazebo can crash or render black. Export these **before any
GUI launch** (put them in your shell profile or prefix each launch):

```bash
export QT_QPA_PLATFORM=xcb
export LIBGL_ALWAYS_SOFTWARE=true
```

---

## Run the simulation

```bash
ros2 launch humanoid_arms_moveit_config demo.launch.py
```

This brings up RViz with the MoveIt MotionPlanning panel. Use **Plan & Execute**
(you must Execute, not just Plan) to pose the arm. The transparent/ghost arm is the
real robot state; the solid arm is the planning goal.

---

## Capture & replay (simulation)

Pose the arm in RViz, then:

```bash
# in the scripts/ directory, with the sim running
python3 capture_keyframes.py left        # ENTER snapshots each pose; s save, q quit
python3 replay_keyframes.py FILE.json --dt 1.5
```

---

## Hardware bring-up

The physical arm uses **Feetech STS3215/STS3250** servos on a TTL bus via a
Waveshare adapter (default `/dev/ttyACM0` @ 1 Mbaud) with a separate 12 V supply.

**Serial port permissions:** add your user to the `dialout` group once
(`sudo usermod -aG dialout $USER`, then log out/in), or per-session
`sudo chmod a+rw /dev/ttyACM0`.

### 1. Assign servo IDs
Left arm = IDs 2–8, left claw = 9, right arm = 10–16, right claw = 17.
```bash
python3 assign_ids_right_and_claws.py    # one fresh (ID 1) servo at a time
python3 ping_all_ids.py                  # confirm 2–17 respond
```

### 2. Calibrate
`servo_calibration.json` and `mirror_calibration.json` **in this repo are the
author's calibration** for a specific physical arm. **You must recalibrate for your
own hardware** — the zero ticks and signs will not match your build.

```bash
python3 capture_zero.py        # record each joint's zero tick (arm at zero pose)
python3 sign_calibrate.py      # find each joint's sign
python3 rezero_servo.py <id>   # re-center a servo to mid-range (fixes tick wraparound)
```

### 3. Drive the arm from captured motion
```bash
python3 replay_hardware.py FILE.json     # keyframes -> servo ticks
python3 teleop_record.py                 # go limp, record motion by hand (50 Hz)
python3 teleop_replay.py FILE.json       # stream a recording back onto the servos
```

---

## Real-time bilateral teleoperation

Move the **left** arm by hand (limp) and the **right** arm mirrors it live.

```bash
# 1. discover the reference pose + per-joint mirror signs (one time, watch the arm)
python3 mirror_calibrate.py              # writes mirror_calibration.json (8 pairs)

# 2. real-time mirror
python3 teleop_mirror.py                 # left limp -> right follows, incl. grippers
```

Just the grippers, in isolation:
```bash
python3 gripper_mirror.py
```

**How the mirror works.** Both arms use identical direct-drive servos, so mirroring
is done in raw tick space: `R = R0 + sign*(L - L0)`. `L0/R0` are the ticks at a
shared reference pose; `sign` (±1) encodes which joints are physically mirrored and
is found empirically during calibration. The teleop unwraps the 0–4095 tick seam so
yaw joints don't fling a full turn, and applies EMA smoothing + a per-loop slew
limit for safety.

---

## Script reference

All scripts live in `src/humanoid_arms/scripts/`. Sim scripts need `demo.launch.py`
running; hardware scripts need the servo bus powered and readable on `/dev/ttyACM0`.

**Servo bus setup**
| Script | What it does |
|--------|-------------|
| `assign_servo_ids.py` | Assign IDs 2–8 to fresh servos (left arm), one at a time. |
| `assign_ids_right_and_claws.py` | Assign IDs 9–17 (left claw, right arm, right claw). |
| `ping_all_ids.py` | Ping the expected IDs and report which servos respond. |
| `map_servos.py` | Wiggle each servo to discover which one drives which joint. |

**Calibration**
| Script | What it does |
|--------|-------------|
| `capture_zero.py` | Read-only: record each joint's zero tick with the arm at its URDF-zero pose. |
| `sign_calibrate.py` | Sim-vs-hardware jog to find each joint's ±1 sign. |
| `rezero_servo.py <id>` | Re-center any servo to mid-range and auto-update the calibration files (fixes tick-wraparound). |
| `rezero_elbow.py` | Elbow-only re-center — **superseded by `rezero_servo.py 5`** (kept for reference). |
| `read_live.py <id>` | Torque-off live tick readout — move a joint by hand and watch its ticks. |

**Simulation / IK**
| Script | What it does |
|--------|-------------|
| `ik_trajectory.py` | FK-anchored straight-line Cartesian sweep; solves IK per waypoint, writes a CSV. |
| `ik_explorer.py` | Builds RobotModel from URDF+SRDF; solves single-point and straight-line Cartesian IK offline. |
| `pose_and_capture.py` | Interactive-marker poser + capture — **superseded** by MoveIt Plan&Execute + `capture_keyframes.py`. |

**Keyframe capture & replay**
| Script | What it does |
|--------|-------------|
| `capture_keyframes.py` | Snapshot the current joint states as keyframes (sim). |
| `replay_keyframes.py` | Replay a keyframe JSON through the controllers (sim). |
| `replay_hardware.py` | Replay a keyframe JSON on the physical servos (rad→tick). |
| `hw_goto.py` | Drive the real arm to one pose/keyframe and optionally mirror it in the sim, to validate calibration. |

**Kinesthetic teleop (record by hand → replay)**
| Script | What it does |
|--------|-------------|
| `teleop_record.py` | Go limp, record dense motion (50 Hz) as you move the arm by hand. |
| `teleop_replay.py` | Stream a teleop recording back onto the servos. |

**Real-time bilateral (mirror) teleop**
| Script | What it does |
|--------|-------------|
| `mirror_calibrate.py` | Discover the reference pose + per-joint mirror signs → writes `mirror_calibration.json`. |
| `teleop_mirror.py` | Real-time: left arm limp leads, right arm mirrors it (all 7 joints + gripper). |
| `gripper_mirror.py` | Standalone gripper-only mirror (left claw → right claw). |

**Basic bring-up tests**
| Script | What it does |
|--------|-------------|
| `test_wiggle.py` | Gently wiggle a single servo around its current position. |
| `test_wiggle_dual.py` | Sync-wiggle two servos at once (group sync-write demo). |

---

## Safety notes

- **Read servo positions before enabling torque** so the arm doesn't snap to a stale
  command.
- Generated IK angles can exceed a servo's physical range — sanity-check before
  driving hardware. The scripts clamp to each joint's `[tick_min, tick_max]`.
- The arm goes **limp** when torque is released (kinesthetic capture, teleop leader) —
  support it so it doesn't fall.
- Confirm the servo bus responds (`ping_all_ids.py`) with 12 V on *before* recording;
  a powered-off bus silently logs frozen fallback values.

---

## License

MIT (see `LICENSE`).
