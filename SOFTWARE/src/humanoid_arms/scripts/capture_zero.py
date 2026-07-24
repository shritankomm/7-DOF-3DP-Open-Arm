#!/usr/bin/env python3
"""
Capture the ZERO offset of each left-arm servo.

READ-ONLY: it only reads servo positions, it does NOT move anything. Run it while
the arm is physically in its URDF-zero pose (joints hanging to zero / wrists
centered). Records each servo's current tick as that joint's zero_tick and sets
ticks_per_rad = 651.9 (direct-drive STS3215: 4096 ticks / 2pi rad) into
servo_calibration.json.

Run in YOUR terminal (needs /dev/ttyACM0 access):
    python3 capture_zero.py
"""

import json
import math
import os

from scservo_sdk import *   # noqa: F401,F403

HERE = os.path.dirname(os.path.abspath(__file__))
CAL = os.path.join(HERE, "servo_calibration.json")
TICKS_PER_REV = 4096.0
TICKS_PER_RAD = TICKS_PER_REV / (2.0 * math.pi)   # ~651.9, direct-drive


def main():
    cal = json.load(open(CAL))
    port = cal.get("port", "/dev/ttyACM0")
    baud = cal.get("baud", 1000000)

    ph = PortHandler(port)
    servo = sms_sts(ph)
    if not ph.openPort():
        print(f"!! could not open {port} (permission? in the dialout group?)")
        return
    ph.setBaudRate(baud)

    print("Reading current positions (arm should be in its ZERO pose)...\n")
    ok = True
    for jname, j in cal["joints"].items():
        sid = j["servo_id"]
        pos, _, res, _ = servo.ReadPosSpeed(sid)
        if res != 0:
            print(f"  {jname:20s} ID {sid}: NO RESPONSE")
            ok = False
            continue
        j["zero_tick"] = pos
        j["ticks_per_rad"] = round(TICKS_PER_RAD, 3)
        print(f"  {jname:20s} ID {sid}: zero_tick = {pos}")
    ph.closePort()

    if not ok:
        print("\n!! some servos didn't respond -- not saving. Check wiring and rerun.")
        return

    json.dump(cal, open(CAL, "w"), indent=2)
    print(f"\nsaved zero offsets -> {CAL}")
    print("Next: determine sign (+1/-1) and safe tick limits per joint.")


if __name__ == "__main__":
    main()
