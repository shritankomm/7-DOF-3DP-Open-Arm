#!/usr/bin/env python3
"""
Re-center the LEFT elbow servo (ID 5) so its STRAIGHT position reads tick ~2048
(middle of range) instead of ~4070 (top). This removes the wraparound: the
intended bend direction then climbs 2048 -> ~3754 with no 4095->0 rollover, so
sign +1 works directly.

Uses the Feetech mid-position calibration: write 128 to the Torque Enable
register while the joint is at the desired center. Writes EEPROM (persistent,
but re-doable any time).

RUN WITH THE ELBOW PHYSICALLY STRAIGHT (zero pose). Then it updates
servo_calibration.json (zero_tick=2048, sign=+1, tick range recomputed).

    python3 rezero_elbow.py
"""
import json
import os
import time

from scservo_sdk import *   # noqa: F401,F403

HERE = os.path.dirname(os.path.abspath(__file__))
CAL = os.path.join(HERE, "servo_calibration.json")
SID = 5                      # leftelbowpitch
CAL_TRIGGER = 128            # write to Torque Enable reg -> set current pos as center (2048)


def main():
    cal = json.load(open(CAL))
    port = cal.get("port", "/dev/ttyACM0"); baud = cal.get("baud", 1000000)
    ph = PortHandler(port); servo = sms_sts(ph)
    if not ph.openPort():
        print(f"!! could not open {port} (permission?)"); return
    ph.setBaudRate(baud)

    pos, _, res, _ = servo.ReadPosSpeed(SID)
    if res != 0:
        print(f"!! could not read servo {SID}"); ph.closePort(); return
    print(f"Elbow (ID {SID}) currently reads tick {pos}.")
    print("This should be your STRAIGHT / zero pose (~4070). If it's not straight,")
    print("Ctrl-C now, straighten the elbow, and rerun.\n")
    ans = input("Elbow is straight -> re-center it to 2048? [y/N] ").strip().lower()
    if ans != "y":
        print("aborted, nothing changed."); ph.closePort(); return

    # middle-position calibration
    servo.unLockEprom(SID)
    servo.write1ByteTxRx(SID, SMS_STS_TORQUE_ENABLE, CAL_TRIGGER)
    time.sleep(0.5)
    servo.LockEprom(SID)
    time.sleep(0.3)
    servo.write1ByteTxRx(SID, SMS_STS_TORQUE_ENABLE, 0)   # relax

    newpos, _, res, _ = servo.ReadPosSpeed(SID)
    print(f"\nElbow now reads tick {newpos} at straight (target ~2048).")
    ph.closePort()

    if res != 0 or not (1900 <= newpos <= 2200):
        print("!! re-center didn't land near 2048 -- NOT updating calibration.")
        print("   Check the elbow was straight and rerun.")
        return

    # update calibration: zero=newpos, sign=+1, direct-drive scale, range from URDF [0,2.618]
    tpr = 651.899
    j = cal["joints"]["leftelbowpitch"]
    j["zero_tick"] = newpos
    j["sign"] = 1
    j["ticks_per_rad"] = tpr
    j["tick_min"] = newpos
    j["tick_max"] = int(round(newpos + tpr * 2.61799))
    j.pop("note", None)
    json.dump(cal, open(CAL, "w"), indent=2)
    print(f"updated calibration: zero_tick={newpos}, sign=+1, "
          f"tick range [{j['tick_min']}, {j['tick_max']}]")
    print("Re-test with:  python3 hw_goto.py elbow_bend_test.json")


if __name__ == "__main__":
    main()
