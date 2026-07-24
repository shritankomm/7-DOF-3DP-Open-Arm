#!/usr/bin/env python3
"""
Discover which physical servo (IDs 2-8) drives which URDF joint of the LEFT arm.

Wiggles each servo one at a time with a gentle, clearly visible motion, then asks
which joint moved. You just watch the arm and press a number (7 times). Writes
servo_mapping.json for the calibration + hardware-replay steps.

SAFETY:
  - Reads each servo's CURRENT position before moving and wiggles gently AROUND it
    (no snap to a stale target).
  - Slow speed / low acceleration; small wiggle (~13 deg, tune WIGGLE_TICKS).
  - Torque disabled on ALL servos at the end and on Ctrl-C.

Run in YOUR terminal (you must watch the arm):
    python3 map_servos.py
Permission error on /dev/ttyACM0?  ->  sudo usermod -aG dialout $USER  (re-login)
"""

import json
import time

from scservo_sdk import *   # noqa: F401,F403  (matches the other hardware scripts)

PORT = "/dev/ttyACM0"
BAUD = 1000000
ARM_IDS = [2, 3, 4, 5, 6, 7, 8]
WIGGLE_TICKS = 150     # ~13 deg; gentle but visible. Lower if a joint is tight.
SPEED = 200            # slow
ACCEL = 30             # gentle
SETTLE = 0.9           # seconds between moves

# left-arm joints, shoulder -> wrist (from CLAUDE.md)
JOINTS = ["leftshoulderpitch", "leftshoulderroll", "leftarmyaw", "leftelbowpitch",
          "leftforearmyaw", "leftwristpitch", "leftwristroll"]


def wiggle(servo, sid, center):
    lo = max(0, center - WIGGLE_TICKS)
    hi = min(4095, center + WIGGLE_TICKS)
    for target in (hi, center, lo, center):
        servo.WritePosEx(sid, target, SPEED, ACCEL)
        time.sleep(SETTLE)


def main():
    ph = PortHandler(PORT)
    servo = sms_sts(ph)
    if not ph.openPort():
        print(f"!! could not open {PORT} (permission? in the dialout group?)")
        return
    ph.setBaudRate(BAUD)

    def torque_off_all():
        for sid in ARM_IDS:
            servo.write1ByteTxRx(sid, SMS_STS_TORQUE_ENABLE, 0)

    try:
        print("Pinging servos", ARM_IDS, "...")
        missing = []
        for sid in ARM_IDS:
            _, res, _ = servo.ping(sid)
            print(f"  ID {sid}: {'OK' if res == 0 else 'NO RESPONSE'}")
            if res != 0:
                missing.append(sid)
        if missing:
            print(f"!! not responding: {missing}. Check wiring/IDs, then rerun.")
            return

        mapping = {}   # servo_id -> joint_name
        print("\nEach servo wiggles once. Watch the arm; tell me which joint moved.\n")
        for sid in ARM_IDS:
            joint = None
            while True:
                pos, _, res, _ = servo.ReadPosSpeed(sid)
                if res != 0:
                    print(f"  !! can't read ID {sid}, skipping")
                    break
                print(f"--- Servo ID {sid}: wiggling around {pos} (watch the arm) ---")
                wiggle(servo, sid, pos)
                print("  Which joint moved?")
                for i, j in enumerate(JOINTS):
                    used = "  (already mapped)" if j in mapping.values() else ""
                    print(f"    {i}: {j}{used}")
                print("    r: wiggle again   s: skip this servo")
                ans = input("  choice> ").strip().lower()
                if ans == "r":
                    continue
                if ans == "s":
                    break
                if ans.isdigit() and 0 <= int(ans) < len(JOINTS):
                    joint = JOINTS[int(ans)]
                    break
                print("  ? enter a joint number, r, or s")
            if joint:
                mapping[sid] = joint
                print(f"  => ID {sid} = {joint}\n")

        print("\n=== MAPPING ===")
        for sid in ARM_IDS:
            print(f"  ID {sid}: {mapping.get(sid, '(unmapped)')}")
        unmapped = [j for j in JOINTS if j not in mapping.values()]
        if unmapped:
            print("  joints with NO servo yet:", unmapped)

        out = "servo_mapping.json"
        json.dump({
            "port": PORT, "baud": BAUD, "arm": "left",
            "servo_to_joint": {str(k): v for k, v in mapping.items()},
            "joint_to_servo": {v: k for k, v in mapping.items()},
        }, open(out, "w"), indent=2)
        print(f"\nsaved -> {out}")

    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        torque_off_all()
        ph.closePort()
        print("torque disabled on all servos, port closed.")


if __name__ == "__main__":
    main()
