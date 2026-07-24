#!/usr/bin/env python3
"""
Straight-line IK trajectory generator for the humanoid_arms 7-DOF arms.
FK-anchored (sidesteps the broken world/root TF tree).

Usage:
    python3 ik_trajectory.py           # left arm (default)
    python3 ik_trajectory.py left
    python3 ik_trajectory.py right

Requires demo.launch.py running (provides /compute_fk and /compute_ik).
Writes ik_trajectory_<arm>.csv : joint angles (radians) per waypoint along a
straight Cartesian line, warm-started so the redundant 7th DOF stays continuous.
"""

import csv
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

BASE_LINK = "chest_plate"      # shared base for both arms
AVOID_COLLISIONS = True
IK_TIMEOUT = 0.1
SWEEP_HALF = 0.06
N_WAYPOINTS = 20
SEED_POSE = [0.3, 0.2, 0.0, -0.6, 0.0, 0.3, 0.0]   # mild bend, radians

ARM_CONFIG = {
    "left": {
        "group": "left_arm",
        "tip": "wrist_holder",
        "joints": ["leftshoulderpitch", "leftshoulderroll", "leftarmyaw",
                   "leftelbowpitch", "leftforearmyaw", "leftwristpitch", "leftwristroll"],
    },
    "right": {
        "group": "right_arm",
        "tip": "part_51",
        "joints": ["rightshoulderpitch", "rightshoulderroll", "rightarmyaw",
                   "rightelbowpitch", "rightforearmyaw", "rightwristpitch", "rightwristroll"],
    },
}


class IKFKClient(Node):
    def __init__(self, cfg):
        super().__init__("ik_trajectory_client")
        self.group = cfg["group"]
        self.tip = cfg["tip"]
        self.joint_names = cfg["joints"]
        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.create_client(GetPositionFK, "/compute_fk")
        for cli, name in [(self.ik, "/compute_ik"), (self.fk, "/compute_fk")]:
            if not cli.wait_for_service(timeout_sec=10.0):
                self.get_logger().error(f"{name} not available -- is demo.launch.py running?")
                sys.exit(1)

    def robot_state(self, positions):
        rs = RobotState()
        js = JointState()
        js.name = list(self.joint_names)
        js.position = list(positions)
        rs.joint_state = js
        return rs

    def fk_tip(self, joint_positions):
        req = GetPositionFK.Request()
        req.header.frame_id = BASE_LINK
        req.fk_link_names = [self.tip]
        req.robot_state = self.robot_state(joint_positions)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        resp = fut.result()
        if resp is None or resp.error_code.val != 1 or not resp.pose_stamped:
            return None
        return resp.pose_stamped[0]

    def solve_ik(self, pose_stamped, seed=None):
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group
        req.ik_request.ik_link_name = self.tip
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.avoid_collisions = AVOID_COLLISIONS
        req.ik_request.timeout.nanosec = int(IK_TIMEOUT * 1e9)
        if seed is not None:
            req.ik_request.robot_state = self.robot_state(seed)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        resp = fut.result()
        if resp is None or resp.error_code.val != 1:
            return None
        m = dict(zip(resp.solution.joint_state.name, resp.solution.joint_state.position))
        try:
            return [m[j] for j in self.joint_names]
        except KeyError:
            return None


def main():
    arm = sys.argv[1].lower() if len(sys.argv) > 1 else "left"
    if arm not in ARM_CONFIG:
        print(f"Unknown arm '{arm}'. Use 'left' or 'right'.")
        return
    cfg = ARM_CONFIG[arm]

    rclpy.init()
    node = IKFKClient(cfg)
    print(f"=== Generating trajectory for {arm.upper()} arm (group={cfg['group']}, tip={cfg['tip']}) ===\n")

    anchor = node.fk_tip(SEED_POSE)
    if anchor is None:
        print("FK failed -- couldn't locate the tip. Check joint names / service.")
        rclpy.shutdown()
        return
    ax, ay, az = anchor.pose.position.x, anchor.pose.position.y, anchor.pose.position.z
    frame = anchor.header.frame_id
    ori = anchor.pose.orientation
    print(f"FK anchor: tip at x={ax:.3f} y={ay:.3f} z={az:.3f} in frame '{frame}'")
    print(f"Sweeping straight line +/- {SWEEP_HALF*100:.0f}cm in Y around it.\n")

    def make(x, y, z):
        p = PoseStamped()
        p.header.frame_id = frame
        p.pose.position.x, p.pose.position.y, p.pose.position.z = x, y, z
        p.pose.orientation = ori
        return p

    y0, y1 = ay - SWEEP_HALF, ay + SWEEP_HALF
    rows, seed = [], SEED_POSE
    for i in range(N_WAYPOINTS):
        t = i / (N_WAYPOINTS - 1)
        y = y0 + t * (y1 - y0)
        sol = node.solve_ik(make(ax, y, az), seed=seed)
        if sol is not None:
            seed = sol
            print(f"  wp {i:02d}  y={y:+.3f}  ok")
        else:
            print(f"  wp {i:02d}  y={y:+.3f}  FAILED")
        rows.append((i, sol))

    out = f"ik_trajectory_{arm}.csv"
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["waypoint"] + node.joint_names)
        for i, sol in rows:
            w.writerow([i] + (sol if sol else ["FAILED"] * len(node.joint_names)))

    n_ok = sum(1 for _, s in rows if s)
    print(f"\nWrote {out}  ({n_ok}/{N_WAYPOINTS} solved)")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
