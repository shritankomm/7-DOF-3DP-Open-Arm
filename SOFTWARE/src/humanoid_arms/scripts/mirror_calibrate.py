#!/usr/bin/env python3
"""
MIRROR CALIBRATION -- find the reference pose and per-joint SIGN so the right arm
can mirror the left arm.

Both arms use identical direct-drive servos, so mirroring is done in raw TICK
space (no radians, no scaling):

    delta   = L_now - L0                 # how far the LEFT joint moved from ref
    R_target = R0 + sign * delta         # mirror onto the RIGHT joint

L0/R0 are the ticks each joint reads at a shared REFERENCE pose (both arms posed
to physically match). `sign` is +1 if the right servo should turn the SAME tick
direction as the left, or -1 if OPPOSITE -- this depends on how each servo is
physically mounted (the "which motors are mirrored" question) and MUST be found
by watching the real arm, which is what this script does.

WORKFLOW:
  1. Both arms go LIMP. Pose them into the SAME (mirrored) configuration, e.g.
     both arms hanging straight at neutral. Press Enter -> captures L0, R0.
  2. The right arm holds that reference (torque on). Then for EACH joint, one at
     a time, you move the LEFT joint by hand and the RIGHT joint mirrors it live.
     Keys while a joint is live:
        f = flip this joint's sign      r = re-capture this joint's reference
        n / Enter = accept, next joint  q = quit without finishing
  3. Saves mirror_calibration.json (L0, R0, sign per pair).

SAFETY:
  - The LEFT arm is limp the whole time -- HOLD IT.
  - Only ONE right joint moves at a time; the rest hold at reference. Start with
    SMALL left movements so you can catch a wrong sign before it drives far.
  - Right targets are hard-clamped to [0, 4095]. The right arm has no mechanical
    limits calibrated yet, so watch for hard stops and keep a hand near power.
  - Leaves both arms limp at the end -- keep holding until set down.

Usage:
    python3 mirror_calibrate.py
    python3 mirror_calibrate.py --out mirror_calibration.json
"""

import argparse
import json
import os
import select
import sys
import termios
import time
import tty

from scservo_sdk import *   # noqa: F401,F403

HERE = os.path.dirname(os.path.abspath(__file__))
CAL = os.path.join(HERE, "servo_calibration.json")

# left joint name -> right servo ID (kinematic order, shoulder -> wrist, then claw)
RIGHT_IDS = {
    "leftshoulderpitch": 10,
    "leftshoulderroll":  11,
    "leftarmyaw":        12,
    "leftelbowpitch":    13,
    "leftforearmyaw":    14,
    "leftwristpitch":    15,
    "leftwristroll":     16,
    "leftgripper":       17,
}
# left IDs normally come from servo_calibration.json, but the gripper (ID 9) isn't
# in that file, so its left ID is supplied explicitly here.
LEFT_ID_OVERRIDE = {"leftgripper": 9}
JOINTS = list(RIGHT_IDS.keys())

# gentle so a wrong sign can't whip the right joint across its range
MIRROR_SPEED = 1000
MIRROR_ACCEL = 40


class KeyPoller:
    """Read single keypresses without waiting for Enter (cbreak mode)."""
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *a):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def get(self):
        dr, _, _ = select.select([sys.stdin], [], [], 0)
        return sys.stdin.read(1) if dr else None


def read_tick(servo, sid, fallback):
    pos, _, res, _ = servo.ReadPosSpeed(sid)
    return pos if res == 0 else fallback


def sync_move(servo, ids, ticks):
    for sid, tk in zip(ids, ticks):
        servo.SyncWritePosEx(sid, int(max(0, min(4095, tk))), MIRROR_SPEED, MIRROR_ACCEL)
    servo.groupSyncWrite.txPacket()
    servo.groupSyncWrite.clearParam()


def torque(servo, ids, on):
    for sid in ids:
        servo.write1ByteTxRx(sid, SMS_STS_TORQUE_ENABLE, 1 if on else 0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(HERE, "mirror_calibration.json"))
    args = ap.parse_args()

    cal = json.load(open(CAL))
    left_ids = [LEFT_ID_OVERRIDE.get(n) or cal["joints"][n]["servo_id"] for n in JOINTS]
    right_ids = [RIGHT_IDS[n] for n in JOINTS]

    ph = PortHandler(cal["port"]); servo = sms_sts(ph)
    if not ph.openPort():
        print(f"!! cannot open {cal['port']} (permission? power?)"); return
    ph.setBaudRate(cal["baud"])

    print("MIRROR CALIBRATION")
    print(f"  left  servos: {left_ids}")
    print(f"  right servos: {right_ids}\n")

    # --- Stage 1: capture the shared reference pose -------------------------
    torque(servo, left_ids, False)
    torque(servo, right_ids, False)
    print("Both arms are LIMP. Pose them into the SAME mirrored configuration")
    print("(e.g. both arms hanging straight down at neutral). HOLD the left arm.")
    input("Press Enter to capture the reference pose... ")

    L0 = [read_tick(servo, s, 0) for s in left_ids]
    R0 = [read_tick(servo, s, 0) for s in right_ids]
    sign = [1] * len(JOINTS)      # discovered below
    print("\nReference captured:")
    for k, n in enumerate(JOINTS):
        print(f"  {n:20s} L0={L0[k]:4d}  R0={R0[k]:4d}")

    # right arm holds the reference; left stays limp
    torque(servo, right_ids, True)
    sync_move(servo, right_ids, R0)
    time.sleep(1.0)

    # --- Stage 2: per-joint live sign discovery ----------------------------
    print("\nNow tune each joint. Move the LEFT joint by hand; the RIGHT mirrors it.")
    print("Keys:  f=flip sign   r=re-capture this joint   n/Enter=next   q=quit\n")

    try:
        with KeyPoller() as kp:
            for k, n in enumerate(JOINTS):
                lid, rid = left_ids[k], right_ids[k]
                print(f"\n--- {n}  (left {lid} -> right {rid}) ---")
                print("    move the LEFT joint; watch the RIGHT. f=flip  r=recapture  n=next  q=quit")
                l_last = L0[k]
                while True:
                    l = read_tick(servo, lid, l_last); l_last = l
                    delta = l - L0[k]
                    r_target = R0[k] + sign[k] * delta
                    r_clamped = int(max(0, min(4095, r_target)))
                    # move only this joint; the rest hold at reference
                    targets = list(R0)
                    targets[k] = r_clamped
                    sync_move(servo, right_ids, targets)

                    flag = "" if r_clamped == r_target else "  <clamped!>"
                    print(f"    L={l:4d} d={delta:+5d}  sign={sign[k]:+d}  "
                          f"R->{r_clamped:4d}{flag}      ", end="\r", flush=True)

                    key = kp.get()
                    if key in ("f", "F"):
                        sign[k] *= -1
                        print(f"\n    sign flipped -> {sign[k]:+d}")
                    elif key in ("r", "R"):
                        print("\n    RE-CAPTURE: match both arms at this joint, then press any key...")
                        torque(servo, [rid], False)      # let you pose the right joint too
                        while kp.get() is None:
                            time.sleep(0.02)
                        L0[k] = read_tick(servo, lid, L0[k])
                        R0[k] = read_tick(servo, rid, R0[k])
                        torque(servo, [rid], True)
                        sync_move(servo, right_ids, R0)
                        print(f"    re-captured: L0={L0[k]} R0={R0[k]}")
                    elif key in ("n", "\n", "\r"):
                        print(f"\n    accepted {n}: sign={sign[k]:+d}")
                        break
                    elif key in ("q", "Q"):
                        raise KeyboardInterrupt
                    time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n\ninterrupted -- saving what we have.")
    finally:
        # ease right back to reference, then everything limp
        try:
            sync_move(servo, right_ids, R0)
            time.sleep(1.0)
        except Exception:
            pass
        torque(servo, right_ids, False)
        torque(servo, left_ids, False)
        ph.closePort()

    out = {
        "type": "mirror_calibration",
        "note": "R_target = R0 + sign*(L_now - L0), in raw ticks. leader=left, follower=right.",
        "port": cal["port"], "baud": cal["baud"],
        "pairs": [
            {"joint": n, "left_id": left_ids[k], "right_id": right_ids[k],
             "L0": L0[k], "R0": R0[k], "sign": sign[k]}
            for k, n in enumerate(JOINTS)
        ],
    }
    with open(args.out, "w") as f:
        json.dump(out, f, indent=2)
    print(f"\nsaved -> {args.out}")
    print("signs:", {n: sign[k] for k, n in enumerate(JOINTS)})
    print("Both arms limp. Next: python3 teleop_mirror.py")


if __name__ == "__main__":
    main()
