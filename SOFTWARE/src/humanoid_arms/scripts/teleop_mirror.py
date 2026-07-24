#!/usr/bin/env python3
"""
REAL-TIME BILATERAL TELEOP -- move the LEFT arm by hand, the RIGHT arm mirrors it
live.

The LEFT arm goes limp and acts as the "leader": you move it by hand. Every loop
we read its ticks and drive the RIGHT arm (the "follower") to the mirrored ticks,
using the reference + signs from mirror_calibrate.py:

    R_target = R0 + sign * (L_now - L0)      # per joint, in raw ticks

This script is data-driven: it mirrors EVERY pair in mirror_calibration.json. That
now includes the 7 arm joints (left 2-8 -> right 10-16) AND the gripper
(left 9 -> right 17), so calibrating the claw in mirror_calibrate.py automatically
makes it mirror here too -- no code change needed. (gripper_mirror.py can also
drive just the claws in isolation.)

Run mirror_calibrate.py FIRST to produce mirror_calibration.json.

SAFETY:
  - LEFT arm is limp the whole time -- HOLD IT / support it.
  - Right targets are hard-clamped to [0, 4095] and SLEW-LIMITED (--max-step) so a
    single bad read can't make the follower jump. A light EMA (--smooth) removes
    jitter. Start with defaults and small motions; keep a hand near power.
  - Enables right torque and eases to the reference before mirroring; on Ctrl-C it
    eases the right arm back to reference and drops torque on both arms.

Usage:
    python3 teleop_mirror.py
    python3 teleop_mirror.py --rate 60 --smooth 0.4 --max-step 120
    python3 teleop_mirror.py --cal mirror_calibration.json
"""

import argparse
import json
import os
import time

from scservo_sdk import *   # noqa: F401,F403

HERE = os.path.dirname(os.path.abspath(__file__))
MIRROR_CAL = os.path.join(HERE, "mirror_calibration.json")

FOLLOW_SPEED = 0     # 0 = servo max; per-loop deltas are small so motion stays smooth
FOLLOW_ACCEL = 0
APPROACH_SPEED = 300  # gentle ease to the reference at the start / end
APPROACH_ACCEL = 20


def read_tick(servo, sid, fallback):
    pos, _, res, _ = servo.ReadPosSpeed(sid)
    return pos if res == 0 else fallback


def sync_move(servo, ids, ticks, speed, accel):
    for sid, tk in zip(ids, ticks):
        servo.SyncWritePosEx(sid, int(max(0, min(4095, tk))), speed, accel)
    servo.groupSyncWrite.txPacket()
    servo.groupSyncWrite.clearParam()


def torque(servo, ids, on):
    for sid in ids:
        servo.write1ByteTxRx(sid, SMS_STS_TORQUE_ENABLE, 1 if on else 0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cal", default=MIRROR_CAL, help="mirror_calibration.json")
    ap.add_argument("--rate", type=float, default=60.0, help="mirror loop Hz")
    ap.add_argument("--smooth", type=float, default=0.35,
                    help="EMA factor 0..1 (0=no smoothing/most responsive, "
                         "higher=smoother/laggier)")
    ap.add_argument("--max-step", type=int, default=150,
                    help="max ticks the follower may move per loop (slew limit)")
    args = ap.parse_args()

    if not os.path.exists(args.cal):
        print(f"!! {args.cal} not found -- run mirror_calibrate.py first."); return
    mc = json.load(open(args.cal))
    pairs = mc["pairs"]
    left_ids = [p["left_id"] for p in pairs]
    right_ids = [p["right_id"] for p in pairs]
    L0 = [p["L0"] for p in pairs]
    R0 = [p["R0"] for p in pairs]
    sign = [p["sign"] for p in pairs]
    names = [p["joint"] for p in pairs]

    ph = PortHandler(mc["port"]); servo = sms_sts(ph)
    if not ph.openPort():
        print(f"!! cannot open {mc['port']} (permission? power?)"); return
    ph.setBaudRate(mc["baud"])

    print("BILATERAL TELEOP -- left arm leads, right arm mirrors.")
    print(f"  pairs: {[ (names[k], left_ids[k], right_ids[k], sign[k]) for k in range(len(names)) ]}")
    print(f"  loop {args.rate:.0f} Hz  smooth={args.smooth}  max-step={args.max_step}\n")

    # right arm: torque on, ease to reference so it starts matched
    torque(servo, right_ids, True)
    sync_move(servo, right_ids, R0, APPROACH_SPEED, APPROACH_ACCEL)
    time.sleep(1.5)
    input("Right arm is at reference. Press Enter, then the LEFT arm goes LIMP -- HOLD IT. ")

    # left arm limp -> becomes the hand-guided leader
    torque(servo, left_ids, False)
    print("LEFT arm limp. Move it; the right arm follows. Ctrl-C to stop.\n")

    period = 1.0 / args.rate
    a = args.smooth
    filt = [float(r) for r in R0]     # EMA state of the follower target
    cmd = list(R0)                    # last commanded follower ticks (for slew limit)
    # UNWRAP: track a CONTINUOUS leader position per joint. The servos read 0-4095
    # and roll over at that seam; a yaw joint whose zero sits near 0/4095 makes the
    # raw reading jump ~4096 when it crosses, which would otherwise fling the
    # follower a whole turn. Seed from the current reading, then accumulate small
    # per-frame deltas, treating any jump > half-range as a seam crossing.
    l_prev = [read_tick(servo, left_ids[k], L0[k]) for k in range(len(names))]
    l_cont = list(l_prev)
    next_t = time.time()
    try:
        while True:
            for k in range(len(names)):
                l = read_tick(servo, left_ids[k], l_prev[k])
                d = l - l_prev[k]
                if d > 2048:            # rolled 0 -> 4095: unwrap downward
                    d -= 4096
                elif d < -2048:         # rolled 4095 -> 0: unwrap upward
                    d += 4096
                l_cont[k] += d
                l_prev[k] = l
                raw = R0[k] + sign[k] * (l_cont[k] - L0[k])
                raw = max(0, min(4095, raw))
                filt[k] = a * filt[k] + (1 - a) * raw           # EMA smooth
                step = filt[k] - cmd[k]
                step = max(-args.max_step, min(args.max_step, step))  # slew limit
                cmd[k] = cmd[k] + step
            sync_move(servo, right_ids, cmd, FOLLOW_SPEED, FOLLOW_ACCEL)
            next_t += period
            slp = next_t - time.time()
            if slp > 0:
                time.sleep(slp)
            else:
                next_t = time.time()
    except KeyboardInterrupt:
        print("\n\nstopping -- easing right arm back to reference.")
    finally:
        try:
            sync_move(servo, right_ids, R0, APPROACH_SPEED, APPROACH_ACCEL)
            time.sleep(1.5)
        except Exception:
            pass
        torque(servo, right_ids, False)
        torque(servo, left_ids, False)
        ph.closePort()
        print("both arms limp. done.")


if __name__ == "__main__":
    main()
