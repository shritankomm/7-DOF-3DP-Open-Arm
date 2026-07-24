#!/usr/bin/env python3
"""
Re-center ANY servo to mid-range (~2048) and AUTO-UPDATE the calibration files so
the change flows straight into the teleop -- no hand-editing. General version of
rezero_elbow.py (use `rezero_servo.py 5` for the elbow now; it also keeps the
mirror config in sync, which rezero_elbow.py did not).

WHY: a servo reads 0-4095 and rolls over at that seam. If a joint's working range
sits near 0 or 4095 it wraps, and the arm flings a full turn. Re-centering moves
the joint's CURRENT position to ~2048, putting the seam a half-turn away in both
directions.

WHY THE AUTO-UPDATE IS CORRECT: mid-position calibration shifts EVERY reading for
that servo by a single constant offset (delta). So we add that same delta to every
stored tick for this servo:
  - mirror_calibration.json : L0 if it's a leader (left) id, R0 if a follower (right) id
  - servo_calibration.json  : zero_tick, tick_min, tick_max if it's a left joint there
Because rad = (tick - zero)/tpr and mirror = R0 + (L - L0), shifting BOTH the
reference and the live readings by the same delta leaves the actual motion
identical -- calibration stays valid, only the working range moves to center.

USAGE:
  Put the joint at the pose you want to become its CENTER (its neutral / the mirror
  reference pose), then:
      python3 rezero_servo.py <id>
  e.g.  python3 rezero_servo.py 12     # right arm yaw
        python3 rezero_servo.py 14     # right forearm yaw
        python3 rezero_servo.py 5      # left elbow (replaces rezero_elbow.py)

Writes .bak-rezero backups before changing either file.
"""

import json
import os
import shutil
import sys
import time

from scservo_sdk import *   # noqa: F401,F403

HERE = os.path.dirname(os.path.abspath(__file__))
SERVO_CAL = os.path.join(HERE, "servo_calibration.json")
MIRROR_CAL = os.path.join(HERE, "mirror_calibration.json")

CAL_TRIGGER = 128    # write to Torque Enable reg -> set current pos as center (~2048)
CENTER = 2048


def shift(tick, delta):
    return (tick + delta) % 4096


def main():
    if len(sys.argv) < 2:
        print("usage: python3 rezero_servo.py <servo_id>"); return
    sid = int(sys.argv[1])

    scal = json.load(open(SERVO_CAL))
    port = scal.get("port", "/dev/ttyACM0"); baud = scal.get("baud", 1000000)
    ph = PortHandler(port); servo = sms_sts(ph)
    if not ph.openPort():
        print(f"!! cannot open {port} (permission? power?)"); return
    ph.setBaudRate(baud)

    old, _, res, _ = servo.ReadPosSpeed(sid)
    if res != 0:
        print(f"!! could not read servo {sid} (powered? on the bus?)"); ph.closePort(); return
    print(f"Servo {sid} currently reads tick {old}.")
    print("This position will become the new CENTER (~2048). If the joint isn't")
    print("where you want its center, Ctrl-C, reposition it, and rerun.\n")
    if input(f"Re-center servo {sid} so tick {old} -> ~2048? [y/N] ").strip().lower() != "y":
        print("aborted, nothing changed."); ph.closePort(); return

    # middle-position calibration (persistent EEPROM, re-doable any time)
    servo.unLockEprom(sid)
    servo.write1ByteTxRx(sid, SMS_STS_TORQUE_ENABLE, CAL_TRIGGER)
    time.sleep(0.5)
    servo.LockEprom(sid)
    time.sleep(0.3)
    servo.write1ByteTxRx(sid, SMS_STS_TORQUE_ENABLE, 0)   # relax

    new, _, res, _ = servo.ReadPosSpeed(sid)
    ph.closePort()
    if res != 0 or not (1900 <= new <= 2200):
        print(f"!! re-center landed at {new}, not near 2048 -- NOT updating any file.")
        print("   Check the servo and rerun.")
        return
    delta = new - old
    print(f"\nServo {sid} now reads {new} at this pose. offset delta = {delta:+d}")
    print("Applying that same offset to every stored tick for this servo...\n")

    touched = False

    # --- mirror_calibration.json: shift L0 (leader) or R0 (follower) ---------
    if os.path.exists(MIRROR_CAL):
        mc = json.load(open(MIRROR_CAL))
        hit = False
        for p in mc.get("pairs", []):
            if p.get("left_id") == sid:
                before = p["L0"]; p["L0"] = shift(before, delta)
                print(f"  mirror {p['joint']}: L0 {before} -> {p['L0']}")
                hit = True
            if p.get("right_id") == sid:
                before = p["R0"]; p["R0"] = shift(before, delta)
                print(f"  mirror {p['joint']}: R0 {before} -> {p['R0']}")
                hit = True
        if hit:
            shutil.copy(MIRROR_CAL, MIRROR_CAL + ".bak-rezero")
            json.dump(mc, open(MIRROR_CAL, "w"), indent=2)
            touched = True
        else:
            print(f"  (servo {sid} not in mirror_calibration.json -- skipped)")

    # --- servo_calibration.json: shift zero_tick / tick_min / tick_max -------
    hit = False
    for name, j in scal.get("joints", {}).items():
        if j.get("servo_id") == sid:
            z0, mn, mx = j["zero_tick"], j["tick_min"], j["tick_max"]
            j["zero_tick"] = shift(z0, delta)
            j["tick_min"] = shift(mn, delta)
            j["tick_max"] = shift(mx, delta)
            print(f"  servo_cal {name}: zero {z0}->{j['zero_tick']}, "
                  f"range [{mn},{mx}]->[{j['tick_min']},{j['tick_max']}]")
            if j["tick_min"] > j["tick_max"]:
                print(f"    !! range now straddles the 0/4095 seam "
                      f"({j['tick_min']} > {j['tick_max']}) -- re-check this joint's limits.")
            hit = True
    if hit:
        shutil.copy(SERVO_CAL, SERVO_CAL + ".bak-rezero")
        json.dump(scal, open(SERVO_CAL, "w"), indent=2)
        touched = True
    else:
        print(f"  (servo {sid} not in servo_calibration.json -- skipped)")

    if touched:
        print("\ndone -- calibration updated. The teleop will use the new center automatically.")
    else:
        print(f"\nre-centered servo {sid}, but it wasn't in any calibration file to update.")


if __name__ == "__main__":
    main()
