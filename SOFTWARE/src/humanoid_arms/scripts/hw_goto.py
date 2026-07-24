#!/usr/bin/env python3
"""
Drive the physical LEFT arm to a target pose (radians -> ticks via
servo_calibration.json), optionally mirroring the SAME pose in the sim (RViz) so
you can compare them. Use it to VALIDATE the calibration before replaying real
keyframes.

SAFETY:
  - Every joint is clamped to its [tick_min, tick_max] (never past the servo/URDF
    range). Clamped joints are flagged.
  - Reads current servo positions before moving (shown), so nothing snaps blindly.
  - Moderate speed/accel; all joints move together via SyncWrite.
  - Prompts before moving; returns to zero and disables torque at the end / Ctrl-C.

Usage:
    python3 hw_goto.py                     # gentle TEST pose: every joint +0.3 rad
    python3 hw_goto.py --zero              # go to URDF zero
    python3 hw_goto.py FILE.json --index N # go to keyframe N of a capture
    python3 hw_goto.py --no-sim            # hardware only (don't command RViz)
"""

import argparse
import json
import os
import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from scservo_sdk import *   # noqa: F401,F403

HERE = os.path.dirname(os.path.abspath(__file__))
CAL = os.path.join(HERE, "servo_calibration.json")
CONTROLLER = "/left_arm_controller/joint_trajectory"
JOINTS = ["leftshoulderpitch", "leftshoulderroll", "leftarmyaw", "leftelbowpitch",
          "leftforearmyaw", "leftwristpitch", "leftwristroll"]
SPEED = 200      # slow-moderate
ACCEL = 20       # gentle
MOVE_WAIT = 2.5
TEST_POSE = {j: 0.3 for j in JOINTS}


def rad_to_tick(j, rad):
    t = int(round(j["zero_tick"] + j["sign"] * j["ticks_per_rad"] * rad))
    clamped = max(j["tick_min"], min(j["tick_max"], t))
    return clamped, (clamped != t)


class SimCmd(Node):
    def __init__(self):
        super().__init__("hw_goto_sim")
        self.pub = self.create_publisher(JointTrajectory, CONTROLLER, 10)
        # wait for the controller to actually subscribe, else the first msg drops
        t0 = time.time()
        while self.pub.get_subscription_count() == 0 and time.time() - t0 < 5.0:
            time.sleep(0.1)

    def pose(self, posmap, secs=2.0):
        t = JointTrajectory(); t.joint_names = list(JOINTS)
        p = JointTrajectoryPoint()
        p.positions = [float(posmap.get(j, 0.0)) for j in JOINTS]
        p.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        t.points = [p]
        for _ in range(3):          # publish a few times so it lands
            self.pub.publish(t)
            time.sleep(0.1)


def sync_move(servo, id_tick_pairs):
    for sid, tick in id_tick_pairs:
        servo.SyncWritePosEx(sid, tick, SPEED, ACCEL)
    servo.groupSyncWrite.txPacket()
    servo.groupSyncWrite.clearParam()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("file", nargs="?")
    ap.add_argument("--index", type=int, default=0)
    ap.add_argument("--zero", action="store_true")
    ap.add_argument("--no-sim", action="store_true")
    args = ap.parse_args()

    cal = json.load(open(CAL))

    if args.zero:
        pose = {j: 0.0 for j in JOINTS}; label = "ZERO"
    elif args.file:
        data = json.load(open(args.file))
        kf = data["keyframes"][args.index]["positions"]
        pose = dict(zip(data["joint_names"], kf))
        label = f"{os.path.basename(args.file)} keyframe {args.index}"
    else:
        pose = dict(TEST_POSE); label = "TEST pose (+0.3 rad each)"

    ph = PortHandler(cal["port"]); servo = sms_sts(ph)
    if not ph.openPort():
        print(f"!! cannot open {cal['port']} (permission?)"); return
    ph.setBaudRate(cal["baud"])

    print(f"\nTarget: {label}")
    print(f"{'joint':20s} {'rad':>7s} {'->tick':>7s} {'current':>7s}  note")
    print("-" * 55)
    targets = []
    for name in JOINTS:
        j = cal["joints"][name]
        rad = pose.get(name, 0.0)
        tick, clamped = rad_to_tick(j, rad)
        cur, _, res, _ = servo.ReadPosSpeed(j["servo_id"])
        targets.append((j["servo_id"], tick))
        print(f"{name:20s} {rad:7.3f} {tick:7d} {cur:7d}  {'CLAMPED!' if clamped else ''}")

    sim = None
    if not args.no_sim:
        rclpy.init(); sim = SimCmd(); sim.pose(pose)
        print("\n(sim commanded to the same pose -- compare the RViz arm to the real arm)")

    try:
        input("\nReady to MOVE the real arm? [Enter = go, Ctrl-C = abort] ")
        sync_move(servo, targets)
        time.sleep(MOVE_WAIT)
        print("Final positions:")
        for name, (sid, tick) in zip(JOINTS, targets):
            cur, _, _, _ = servo.ReadPosSpeed(sid)
            print(f"  {name:20s} target {tick:6d}  actual {cur:6d}  (err {cur - tick:+d})")
        input("\n[Enter to return to ZERO and relax] ")
    except KeyboardInterrupt:
        print("\naborted -- returning to zero")
    finally:
        # return to zero, then torque off (so the arm rests near zero when limp)
        zero_targets = [(cal["joints"][n]["servo_id"], cal["joints"][n]["zero_tick"]) for n in JOINTS]
        sync_move(servo, zero_targets)
        if sim is not None:
            sim.pose({j: 0.0 for j in JOINTS})
        time.sleep(MOVE_WAIT)
        for name in JOINTS:
            servo.write1ByteTxRx(cal["joints"][name]["servo_id"], SMS_STS_TORQUE_ENABLE, 0)
        ph.closePort()
        if sim is not None:
            rclpy.shutdown()
        print("returned to zero, torque off, done.")


if __name__ == "__main__":
    main()
