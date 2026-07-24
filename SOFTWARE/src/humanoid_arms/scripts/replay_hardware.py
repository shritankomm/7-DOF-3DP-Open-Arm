#!/usr/bin/env python3
"""
Replay a captured keyframe routine on the PHYSICAL left arm.

Reads a keyframes_<arm>_*.json (from capture_keyframes.py), converts each joint
radians->ticks via servo_calibration.json, and plays the sequence through the
servos (SyncWrite, all joints together). Optionally mirrors the same motion in
the sim so you can watch both.

SAFETY:
  - Every joint clamped to its [tick_min, tick_max]; clamped values are flagged.
  - Shows the full converted plan and PAUSES before moving.
  - Moves to the first keyframe slowly, then steps at --dt.
  - Returns to zero and disables torque at the end / on Ctrl-C.

Usage:
    python3 replay_hardware.py FILE.json                 # play once
    python3 replay_hardware.py FILE.json --dt 2.0        # seconds per keyframe
    python3 replay_hardware.py FILE.json --loop 3
    python3 replay_hardware.py FILE.json --no-sim        # hardware only
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
SPEED = 200
ACCEL = 20


def rad_to_tick(j, rad):
    t = int(round(j["zero_tick"] + j["sign"] * j["ticks_per_rad"] * rad))
    clamped = max(j["tick_min"], min(j["tick_max"], t))
    return clamped, (clamped != t)


class SimCmd(Node):
    def __init__(self):
        super().__init__("replay_hw_sim")
        self.pub = self.create_publisher(JointTrajectory, CONTROLLER, 10)
        t0 = time.time()
        while self.pub.get_subscription_count() == 0 and time.time() - t0 < 5.0:
            time.sleep(0.1)

    def pose(self, posmap, secs=1.0):
        t = JointTrajectory(); t.joint_names = list(JOINTS)
        p = JointTrajectoryPoint()
        p.positions = [float(posmap.get(j, 0.0)) for j in JOINTS]
        p.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        t.points = [p]
        for _ in range(2):
            self.pub.publish(t); time.sleep(0.05)


def sync_move(servo, id_tick_pairs):
    for sid, tick in id_tick_pairs:
        servo.SyncWritePosEx(sid, tick, SPEED, ACCEL)
    servo.groupSyncWrite.txPacket()
    servo.groupSyncWrite.clearParam()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("file")
    ap.add_argument("--dt", type=float, default=2.0)
    ap.add_argument("--loop", type=int, default=1)
    ap.add_argument("--no-sim", action="store_true")
    args = ap.parse_args()

    data = json.load(open(args.file))
    cal = json.load(open(CAL))
    names = data["joint_names"]
    frames = [dict(zip(names, kf["positions"])) for kf in data["keyframes"]]
    if not frames:
        print("no keyframes in file"); return

    ph = PortHandler(cal["port"]); servo = sms_sts(ph)
    if not ph.openPort():
        print(f"!! cannot open {cal['port']} (permission?)"); return
    ph.setBaudRate(cal["baud"])

    # build + show the converted plan
    print(f"\nReplaying {os.path.basename(args.file)}: {len(frames)} keyframes\n")
    plans = []   # list over keyframes of [(sid, tick), ...]
    for i, pose in enumerate(frames):
        row = []; flags = []
        for name in JOINTS:
            j = cal["joints"][name]
            tick, clamped = rad_to_tick(j, pose.get(name, 0.0))
            row.append((j["servo_id"], tick))
            if clamped:
                flags.append(name)
        plans.append(row)
        note = f"  CLAMPED: {', '.join(flags)}" if flags else ""
        print(f"  kf{i}: " + " ".join(f"{t:4d}" for _, t in row) + note)

    sim = None
    if not args.no_sim:
        rclpy.init(); sim = SimCmd()

    try:
        input(f"\nReady to move the REAL arm through {len(frames)} keyframes "
              f"({args.dt}s each)? [Enter = go, Ctrl-C = abort] ")
        for rep in range(args.loop):
            if args.loop > 1:
                print(f"--- loop {rep+1}/{args.loop} ---")
            for i, (pose, row) in enumerate(zip(frames, plans)):
                print(f"  -> keyframe {i}")
                sync_move(servo, row)
                if sim is not None:
                    sim.pose(pose)
                time.sleep(args.dt)
        print("done.")
    except (EOFError, KeyboardInterrupt):
        print("\ninterrupted")
    finally:
        zero = [(cal["joints"][n]["servo_id"], cal["joints"][n]["zero_tick"]) for n in JOINTS]
        sync_move(servo, zero)
        if sim is not None:
            sim.pose({j: 0.0 for j in JOINTS});
        time.sleep(2.5)
        for name in JOINTS:
            servo.write1ByteTxRx(cal["joints"][name]["servo_id"], SMS_STS_TORQUE_ENABLE, 0)
        ph.closePort()
        if sim is not None:
            rclpy.shutdown()
        print("returned to zero, torque off.")


if __name__ == "__main__":
    main()
