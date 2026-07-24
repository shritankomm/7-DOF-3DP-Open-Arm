#!/usr/bin/env python3
"""
IK explorer for the humanoid_arms bilateral 7-DOF arm.

Builds RobotModel directly from URDF + SRDF file paths (no MoveItPy, no planning
pipelines). Solves single-point IK and straight-line Cartesian IK with
warm-started seeding, collision-checked against the self-collision model.
"""

import csv
import os

import numpy as np
from geometry_msgs.msg import Pose
from moveit.core.robot_model import RobotModel
from moveit.core.robot_state import RobotState
from moveit.core.planning_scene import PlanningScene

# ---- config -----------------------------------------------------------------
CONFIG_DIR = os.path.expanduser("~/ros2_ws/src/humanoid_arms_moveit_config/config")
URDF_PATH = os.path.join(CONFIG_DIR, "humanoid_arms.urdf")
SRDF_PATH = os.path.join(CONFIG_DIR, "humanoid_arms.srdf")
GROUP_NAME = "right_arm"
TIP_LINK = "part_51"
IK_TIMEOUT = 0.1
# -----------------------------------------------------------------------------


def solve_point_ik(scene: PlanningScene, robot_state: RobotState, pose: Pose, verbose_collision: bool = False):
    ok = robot_state.set_from_ik(
        joint_model_group_name=GROUP_NAME,
        geometry_pose=pose,
        tip_name=TIP_LINK,
        timeout=IK_TIMEOUT,
    )
    if not ok:
        return None
    robot_state.update(force=True)

    colliding = scene.is_state_colliding(
        robot_state=robot_state,
        joint_model_group_name=GROUP_NAME,
        verbose=verbose_collision,
    )
    if colliding:
        return None

    return list(robot_state.get_joint_group_positions(GROUP_NAME))


def lerp_pose(pose_a: Pose, pose_b: Pose, t: float) -> Pose:
    p = Pose()
    p.position.x = pose_a.position.x + t * (pose_b.position.x - pose_a.position.x)
    p.position.y = pose_a.position.y + t * (pose_b.position.y - pose_a.position.y)
    p.position.z = pose_a.position.z + t * (pose_b.position.z - pose_a.position.z)

    qa = np.array([pose_a.orientation.x, pose_a.orientation.y, pose_a.orientation.z, pose_a.orientation.w])
    qb = np.array([pose_b.orientation.x, pose_b.orientation.y, pose_b.orientation.z, pose_b.orientation.w])
    if np.dot(qa, qb) < 0.0:
        qb = -qb
    dot = np.clip(np.dot(qa, qb), -1.0, 1.0)
    theta = np.arccos(dot)
    if theta < 1e-6:
        q = qa
    else:
        q = (np.sin((1 - t) * theta) * qa + np.sin(t * theta) * qb) / np.sin(theta)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
    return p


def solve_straight_line(scene, robot_state, pose_start, pose_end, n_waypoints=20):
    results = []
    for i in range(n_waypoints):
        t = i / (n_waypoints - 1)
        pose = lerp_pose(pose_start, pose_end, t)
        joints = solve_point_ik(scene, robot_state, pose)
        results.append((i, joints))
        status = "ok" if joints else "FAILED (unreachable or self-colliding)"
        print(f"  waypoint {i:02d}  t={t:.2f}  {status}")
    return results


def main():
    print(f"Loading RobotModel from:\n  URDF: {URDF_PATH}\n  SRDF: {SRDF_PATH}")
    robot_model = RobotModel(urdf_xml_path=URDF_PATH, srdf_xml_path=SRDF_PATH)
    print("RobotModel loaded. Groups:", robot_model.joint_model_group_names)

    scene = PlanningScene(robot_model)
    robot_state = scene.current_state
    robot_state.set_to_default_values()
    robot_state.update(force=True)

    # --- single-point IK ---
    target = Pose()
    target.position.x, target.position.y, target.position.z = 0.30, -0.15, 0.40
    target.orientation.w = 1.0
    joints = solve_point_ik(scene, robot_state, target)
    print("\nSingle-point IK result:", joints)

    # --- straight-line path ---
    start_pose = Pose()
    start_pose.position.x, start_pose.position.y, start_pose.position.z = 0.30, -0.15, 0.40
    start_pose.orientation.w = 1.0

    end_pose = Pose()
    end_pose.position.x, end_pose.position.y, end_pose.position.z = 0.30, 0.15, 0.40
    end_pose.orientation.w = 1.0

    robot_state.set_to_default_values()
    robot_state.update(force=True)
    print("\nSolving straight-line path...")
    trajectory = solve_straight_line(scene, robot_state, start_pose, end_pose, n_waypoints=20)

    with open("straight_line_ik.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["waypoint"] + [f"joint_{i}" for i in range(7)])
        for i, joints in trajectory:
            writer.writerow([i] + (joints if joints else ["FAILED"] * 7))

    print("\nWrote straight_line_ik.csv")


if __name__ == "__main__":
    main()
