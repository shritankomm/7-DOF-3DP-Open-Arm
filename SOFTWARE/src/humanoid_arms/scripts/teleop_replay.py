#!/usr/bin/env python3
"""
Replay a kinesthetic recording (from teleop_record.py) on the PHYSICAL left arm.

Streams the recorded servo ticks back at the ORIGINAL timing, so the arm smoothly
retraces the motion you performed by hand. Because the recording is dense (~50 Hz)
this reproduces the whole gesture, not discrete keyframes. Optionally mirrors the
motion in the RViz sim (decimated so RViz stays smooth).

SAFETY:
  - Every tick is clamped to that servo's [tick_min, tick_max]; clamps are counted.
  - Reads current positions, enables torque, then EASES gently to the first frame
    (slow speed) and PAUSES for your confirmation before streaming the full motion.
  - Returns to zero and disables torque at the end / on Ctrl-C.

Usage:
    python3 teleop_replay.py teleop_left_XXXX.json
    python3 teleop_replay.py FILE.json --rate-scale 0.5   # half speed (slower/safer)
    python3 teleop_replay.py FILE.json --loop 3
    python3 teleop_replay.py FILE.json --smooth 5         # moving-avg smoothing window
    python3 teleop_replay.py FILE.json --no-sim
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

STREAM_SPEED = 0      # 0 = servo max speed; deltas per frame are tiny so this is smooth
STREAM_ACCEL = 0      # 0 = max accel; ditto
APPROACH_SPEED = 300  # gentle ease-in to the first frame
APPROACH_ACCEL = 20
SIM_HZ = 15           # decimate sim updates so RViz stays smooth


def clamp(j, tick):
    c = max(j["tick_min"], min(j["tick_max"], tick))
    return c, (c != tick)


def tick_to_rad(j, tick):
    return j["sign"] * (tick - j["zero_tick"]) / j["ticks_per_rad"]


class SimCmd(Node):
    def __init__(self, joints):
        super().__init__("teleop_replay_sim")
        self.joints = joints
        self.pub = self.create_publisher(JointTrajectory, CONTROLLER, 10)
        t0 = time.time()
        while self.pub.get_subscription_count() == 0 and time.time() - t0 < 5.0:
            time.sleep(0.1)

    def send(self, rads, secs):
        t = JointTrajectory(); t.joint_names = list(self.joints)
        p = JointTrajectoryPoint()
        p.positions = [float(r) for r in rads]
        p.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        t.points = [p]
        self.pub.publish(t)


def sync_move(servo, ids, ticks, speed, accel):
    for sid, tk in zip(ids, ticks):
        servo.SyncWritePosEx(sid, tk, speed, accel)
    servo.groupSyncWrite.txPacket()
    servo.groupSyncWrite.clearParam()


def smooth_samples(ticks_list, window):
    if window <= 1:
        return ticks_list
    n = len(ticks_list); w = window // 2
    out = []
    for i in range(n):
        lo, hi = max(0, i - w), min(n, i + w + 1)
        seg = ticks_list[lo:hi]
        out.append([int(round(sum(col) / len(col))) for col in zip(*seg)])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("file")
    ap.add_argument("--rate-scale", type=float, default=1.0,
                    help="<1 slower, >1 faster than the recording")
    ap.add_argument("--loop", type=int, default=1)
    ap.add_argument("--smooth", type=int, default=1, help="moving-avg window (1=off)")
    ap.add_argument("--speed", type=int, default=STREAM_SPEED,
                    help="streaming goal speed (0=max; cap it to slow/soften)")
    ap.add_argument("--accel", type=int, default=STREAM_ACCEL, help="streaming accel (0=max)")
    ap.add_argument("--no-sim", action="store_true")
    args = ap.parse_args()

    rec = json.load(open(args.file))
    if rec.get("type") != "teleop_recording":
        print("!! not a teleop_recording file (use replay_hardware.py for keyframes)")
        return
    cal = json.load(open(CAL))
    joints = rec["joint_names"]
    ids = [cal["joints"][n]["servo_id"] for n in joints]
    jcfg = [cal["joints"][n] for n in joints]

    samples = rec["samples"]
    if not samples:
        print("empty recording"); return
    times = [s["t"] for s in samples]
    raw = [s["ticks"] for s in samples]
    raw = smooth_samples(raw, args.smooth)

    # clamp everything up front; count clamps per joint
    clamps = [0] * len(joints)
    frames = []
    for row in raw:
        out = []
        for k, tk in enumerate(row):
            c, was = clamp(jcfg[k], tk)
            out.append(c)
            if was:
                clamps[k] += 1
        frames.append(out)

    ph = PortHandler(cal["port"]); servo = sms_sts(ph)
    if not ph.openPort():
        print(f"!! cannot open {cal['port']} (permission?)"); return
    ph.setBaudRate(cal["baud"])

    dur = times[-1] / args.rate_scale
    print(f"\nRecording: {os.path.basename(args.file)}")
    print(f"  {len(frames)} frames, {times[-1]:.1f}s recorded "
          f"-> {dur:.1f}s playback (rate-scale {args.rate_scale})")
    print(f"  first frame ticks: {frames[0]}")
    print(f"  last  frame ticks: {frames[-1]}")
    anyc = False
    for k, n in enumerate(joints):
        if clamps[k]:
            pct = 100.0 * clamps[k] / len(frames)
            print(f"  CLAMPED {n}: {clamps[k]}/{len(frames)} frames ({pct:.0f}%)")
            anyc = True
    if not anyc:
        print("  no clamping -- whole motion is within servo limits.")

    print("\n  current servo ticks:", end=" ")
    for sid in ids:
        cur, _, _, _ = servo.ReadPosSpeed(sid)
        print(cur, end=" ")
    print()

    sim = None
    if not args.no_sim:
        rclpy.init(); sim = SimCmd(joints)

    try:
        input("\nEnable torque and EASE to the first frame? [Enter = go, Ctrl-C = abort] ")
        # enable torque + gentle approach to frame 0
        for sid in ids:
            servo.write1ByteTxRx(sid, SMS_STS_TORQUE_ENABLE, 1)
        sync_move(servo, ids, frames[0], APPROACH_SPEED, APPROACH_ACCEL)
        if sim is not None:
            sim.send([tick_to_rad(jcfg[k], frames[0][k]) for k in range(len(joints))], 1.5)
        time.sleep(2.0)

        input("At the start pose. Ready to STREAM the full motion? [Enter = go, Ctrl-C = abort] ")
        sim_period = 1.0 / SIM_HZ
        for rep in range(args.loop):
            if args.loop > 1:
                print(f"--- loop {rep + 1}/{args.loop} ---")
            t_start = time.time()
            last_sim = 0.0
            for i in range(len(frames)):
                # pace to the recording's own timeline (scaled)
                target = t_start + times[i] / args.rate_scale
                slp = target - time.time()
                if slp > 0:
                    time.sleep(slp)
                sync_move(servo, ids, frames[i], args.speed, args.accel)
                if sim is not None and (times[i] - last_sim) >= sim_period:
                    sim.send([tick_to_rad(jcfg[k], frames[i][k]) for k in range(len(joints))],
                             sim_period)
                    last_sim = times[i]
            print(f"  played {len(frames)} frames in {time.time() - t_start:.1f}s")
        print("done.")
    except (EOFError, KeyboardInterrupt):
        print("\ninterrupted")
    finally:
        zero = [jcfg[k]["zero_tick"] for k in range(len(joints))]
        sync_move(servo, ids, zero, APPROACH_SPEED, APPROACH_ACCEL)
        if sim is not None:
            sim.send([0.0] * len(joints), 2.0)
        time.sleep(2.5)
        for sid in ids:
            servo.write1ByteTxRx(sid, SMS_STS_TORQUE_ENABLE, 0)
        ph.closePort()
        if sim is not None:
            rclpy.shutdown()
        print("returned to zero, torque off.")


if __name__ == "__main__":
    main()
