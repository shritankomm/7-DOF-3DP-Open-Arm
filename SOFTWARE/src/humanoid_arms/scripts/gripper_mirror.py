#!/usr/bin/env python3
"""
STANDALONE GRIPPER MIRROR -- mirror just the two claws (left ID 9 -> right ID 17)
in isolation, without touching the arm.

Same tick-space model as the arm teleop:
    R = R0 + sign * (L - L0)
The LEFT claw goes LIMP (squeeze/open it by hand); the RIGHT claw mirrors it live.
Press 'f' to flip the sign if the right claw closes when the left opens; 'q' to quit.

Use this to test/tune the gripper pair on its own before folding it into the
full-arm teleop (mirror_calibrate.py + teleop_mirror.py already include it too).

SAFETY:
  - Right target is hard-clamped to [0, 4095] and slew-limited (--max-step).
  - Do NOT over-squeeze the LEFT claw by hand: the right claw mirrors your squeeze,
    and driving a gripper hard against a stop STALLS the servo (high current/heat).
    Stop if the right claw hits its object/stop and keeps pushing.
  - Both claws are left limp at the end.

Usage:
    python3 gripper_mirror.py
    python3 gripper_mirror.py --smooth 0.4 --max-step 80
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

LEFT_GRIPPER = 9
RIGHT_GRIPPER = 17
MIRROR_SPEED = 600     # gentle -- grippers have little travel; don't slam them
MIRROR_ACCEL = 30


class KeyPoller:
    """Single keypresses without waiting for Enter (cbreak mode)."""
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


def torque(servo, sid, on):
    servo.write1ByteTxRx(sid, SMS_STS_TORQUE_ENABLE, 1 if on else 0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--smooth", type=float, default=0.35,
                    help="EMA factor 0..1 (higher = smoother/laggier)")
    ap.add_argument("--max-step", type=int, default=80,
                    help="max ticks the right claw may move per loop (slew limit)")
    ap.add_argument("--rate", type=float, default=50.0)
    args = ap.parse_args()

    cal = json.load(open(CAL))
    port = cal.get("port", "/dev/ttyACM0"); baud = cal.get("baud", 1000000)
    ph = PortHandler(port); servo = sms_sts(ph)
    if not ph.openPort():
        print(f"!! cannot open {port} (permission? power?)"); return
    ph.setBaudRate(baud)

    print("GRIPPER MIRROR -- left claw leads, right claw follows.")
    print(f"  left  claw = ID {LEFT_GRIPPER}   right claw = ID {RIGHT_GRIPPER}\n")

    # capture reference: both claws limp, pose them to the SAME opening
    torque(servo, LEFT_GRIPPER, False)
    torque(servo, RIGHT_GRIPPER, False)
    print("Both claws are LIMP. Set them to the SAME opening (e.g. both fully open).")
    input("Press Enter to capture the reference... ")
    L0 = read_tick(servo, LEFT_GRIPPER, 0)
    R0 = read_tick(servo, RIGHT_GRIPPER, 0)
    print(f"  reference: L0={L0}  R0={R0}")

    # right claw holds reference; left stays limp
    torque(servo, RIGHT_GRIPPER, True)
    servo.WritePosEx(RIGHT_GRIPPER, R0, MIRROR_SPEED, MIRROR_ACCEL)
    time.sleep(0.6)

    sign = 1
    period = 1.0 / args.rate
    a = args.smooth
    filt = float(R0)
    cmd = R0
    l_last = L0
    print("\nSqueeze/open the LEFT claw. f=flip sign  q=quit\n")
    next_t = time.time()
    try:
        with KeyPoller() as kp:
            while True:
                l = read_tick(servo, LEFT_GRIPPER, l_last); l_last = l
                raw = R0 + sign * (l - L0)
                raw = max(0, min(4095, raw))
                filt = a * filt + (1 - a) * raw
                step = max(-args.max_step, min(args.max_step, filt - cmd))
                cmd = cmd + step
                servo.WritePosEx(RIGHT_GRIPPER, int(cmd), MIRROR_SPEED, MIRROR_ACCEL)
                print(f"  L={l:4d}  d={l - L0:+5d}  sign={sign:+d}  R->{int(cmd):4d}   ",
                      end="\r", flush=True)

                key = kp.get()
                if key in ("f", "F"):
                    sign *= -1
                    print(f"\n  sign flipped -> {sign:+d}")
                elif key in ("q", "Q"):
                    break

                next_t += period
                slp = next_t - time.time()
                if slp > 0:
                    time.sleep(slp)
                else:
                    next_t = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        torque(servo, LEFT_GRIPPER, False)
        torque(servo, RIGHT_GRIPPER, False)
        ph.closePort()
        print(f"\n\ndone. final sign for the gripper: {sign:+d}  (L0={L0} R0={R0})")
        print("both claws limp.")


if __name__ == "__main__":
    main()
