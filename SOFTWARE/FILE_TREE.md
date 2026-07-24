# File Tree — `SOFTWARE/`

A guide to what's in this folder and where to find things. This is a self-contained
**ROS 2 (Jazzy) workspace**: build it with `colcon build` from the `SOFTWARE/` root.

```
SOFTWARE/
├── README.md                     # start here: requirements, build, run, script reference
├── .gitignore                    # keeps build/ install/ log/, caches, recordings out of git
└── src/                          # colcon looks here for ROS 2 packages
    │
    ├── humanoid_arms/                     # robot description + all Python tooling
    │   ├── package.xml                    # ROS 2 package manifest (name, deps, license)
    │   ├── CMakeLists.txt                 # build/install rules
    │   │
    │   ├── config/
    │   │   └── controllers.yaml           # ros2_control controller definitions
    │   │
    │   ├── launch/
    │   │   ├── display.launch.py          # robot in RViz (no physics)
    │   │   └── gazebo.launch.py           # robot in Gazebo (physics sim)
    │   │
    │   ├── urdf/                          # robot geometry/kinematics description
    │   │   ├── humanoid_arms_final.urdf   # ★ the Gazebo robot model (source of truth)
    │   │   ├── humanoid_arms_collision.urdf     # variant w/ collision geometry
    │   │   ├── humanoid_arms_fixed.urdf         # intermediate build step
    │   │   ├── humanoid_arms_ros2_control.urdf  # intermediate build step
    │   │   ├── add_gazebo_plugin.py       # URDF build helper: injects the Gazebo plugin
    │   │   ├── add_ros2_control.py        # URDF build helper: injects ros2_control block
    │   │   └── add_collisions.py          # URDF build helper: adds collision meshes
    │   │
    │   ├── meshes/                        # 56 .stl CAD meshes the URDF references
    │   │                                  #   (chest_plate, shoulder/elbow/wrist parts,
    │   │                                  #    servo bodies, claw arms, frame extrusions)
    │   │
    │   └── scripts/                       # Python tools (see README "Script reference")
    │       │  ── servo bus setup ──
    │       ├── assign_servo_ids.py        # assign IDs 2–8 (left arm)
    │       ├── assign_ids_right_and_claws.py  # assign IDs 9–17 (claws + right arm)
    │       ├── ping_all_ids.py            # check which servos respond
    │       ├── map_servos.py              # discover which servo drives which joint
    │       │  ── calibration ──
    │       ├── capture_zero.py            # record each joint's zero tick (read-only)
    │       ├── sign_calibrate.py          # find each joint's ±1 sign
    │       ├── rezero_servo.py            # re-center a servo to mid-range (fixes wraparound)
    │       ├── rezero_elbow.py            # elbow-only re-center (superseded by rezero_servo)
    │       ├── read_live.py               # live tick readout, joint moved by hand
    │       ├── servo_calibration.json     # ⚙ per-joint hw calibration (RECALIBRATE for your arm)
    │       ├── mirror_calibration.json    # ⚙ left↔right mirror reference + signs
    │       │  ── simulation / IK ──
    │       ├── ik_trajectory.py           # straight-line Cartesian IK sweep → CSV
    │       ├── ik_explorer.py             # offline IK from URDF+SRDF
    │       ├── pose_and_capture.py        # marker poser (superseded by MoveIt + capture)
    │       │  ── keyframe capture / replay ──
    │       ├── capture_keyframes.py       # snapshot joint poses as keyframes (sim)
    │       ├── replay_keyframes.py        # replay keyframes through controllers (sim)
    │       ├── replay_hardware.py         # replay keyframes on the real servos
    │       ├── hw_goto.py                 # drive real arm to one pose; validate calibration
    │       │  ── kinesthetic teleop ──
    │       ├── teleop_record.py           # go limp, record motion by hand at 50 Hz
    │       ├── teleop_replay.py           # stream a recording back onto the servos
    │       │  ── real-time bilateral (mirror) teleop ──
    │       ├── mirror_calibrate.py        # find reference + mirror signs → mirror_calibration.json
    │       ├── teleop_mirror.py           # ★ left arm limp → right arm mirrors live (+ gripper)
    │       ├── gripper_mirror.py          # standalone gripper-only mirror
    │       │  ── basic bring-up tests ──
    │       ├── test_wiggle.py             # wiggle one servo around its current position
    │       └── test_wiggle_dual.py        # sync-wiggle two servos at once
    │
    └── humanoid_arms_moveit_config/       # MoveIt motion-planning configuration
        ├── package.xml                    # ROS 2 package manifest
        ├── CMakeLists.txt                 # build/install rules
        ├── .setup_assistant               # lets MoveIt Setup Assistant re-open/regenerate this
        │
        ├── config/
        │   ├── humanoid_arms.urdf         # robot model MoveIt uses (mock-controllers variant)
        │   ├── humanoid_arms.srdf         # semantic model: groups, tips, disabled collisions
        │   ├── kinematics.yaml            # IK solver (pick_ik) settings per arm
        │   ├── joint_limits.yaml          # velocity/acceleration limits
        │   ├── ompl_planning.yaml         # OMPL planner config
        │   ├── moveit_controllers.yaml    # maps MoveIt to the ros2_control controllers
        │   ├── ros2_controllers.yaml      # controller manager + controller definitions
        │   ├── pilz_cartesian_limits.yaml # Pilz planner limits
        │   └── moveit.rviz                # saved RViz layout (MotionPlanning panel)
        │
        └── launch/
            ├── demo.launch.py             # ★ full bring-up: MoveIt + RViz + mock controllers
            ├── move_group.launch.py       # the MoveIt planning node
            ├── moveit_rviz.launch.py      # RViz with the MotionPlanning plugin
            ├── rsp.launch.py              # robot_state_publisher
            ├── spawn_controllers.launch.py     # start the ros2_control controllers
            ├── static_virtual_joint_tfs.launch.py  # world→base static TF
            ├── setup_assistant.launch.py  # re-open the MoveIt Setup Assistant
            └── warehouse_db.launch.py     # (optional) motion warehouse database
```

**★ = the files most people start with.**
**⚙ = calibration files specific to the author's physical arm — recalibrate for yours.**

## Quick "where do I go for…"
- **Just run the sim** → `README.md` → *Run the simulation* (`demo.launch.py`)
- **Understand the robot** → `src/humanoid_arms/urdf/humanoid_arms_final.urdf` + `meshes/`
- **Capture & replay a pose** → `scripts/capture_keyframes.py`, `scripts/replay_keyframes.py`
- **Drive the real arm** → `README.md` → *Hardware bring-up* + `scripts/` (assign IDs → calibrate → replay)
- **Bilateral teleop** → `scripts/mirror_calibrate.py` then `scripts/teleop_mirror.py`
