#!/usr/bin/env python3
"""
KINESTHETIC TEACHING -- record a motion by moving the arm BY HAND.

Disables torque on all 7 left-arm servos so the arm is limp/backdrivable, then
continuously logs every servo's present tick (with timestamps) while you move the
arm around by hand. Stop with Ctrl-C and it saves a dense trajectory you can play
back with teleop_replay.py. This captures the WHOLE smooth motion, not just a few
keyframes -- and never touches the laggy MoveIt sim.

SAFETY:
  - The arm goes LIMP the instant recording starts. HOLD IT so it doesn't sag or
    fall. There is a countdown before torque is released.
  - This is otherwise read-only (it only reads present positions).
  - Leaves torque OFF at the end -- keep holding the arm until you've set it down.

Usage:
    python3 teleop_record.py                 # 50 Hz, auto-named file
    python3 teleop_record.py --rate 30
    python3 teleop_record.py --out mywave.json
"""

import argparse
import json
import os
import time
from datetime import datetime

from scservo_sdk import *   # noqa: F401,F403

HERE = os.path.dirname(os.path.abspath(__file__))
CAL = os.path.join(HERE, "servo_calibration.json")
JOINTS = ["leftshoulderpitch", "leftshoulderroll", "leftarmyaw", "leftelbowpitch",
          "leftforearmyaw", "leftwristpitch", "leftwristroll"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rate", type=float, default=50.0, help="samples per second")
    ap.add_argument("--out", help="output file (default teleop_left_<timestamp>.json)")
    ap.add_argument("--countdown", type=int, default=3)
    args = ap.parse_args()

    cal = json.load(open(CAL))
    ids = [cal["joints"][n]["servo_id"] for n in JOINTS]

    ph = PortHandler(cal["port"]); servo = sms_sts(ph)
    if not ph.openPort():
        print(f"!! cannot open {cal['port']} (permission?)"); return
    ph.setBaudRate(cal["baud"])

    print("KINESTHETIC RECORD -- the arm will go LIMP. HOLD IT NOW.")
    print(f"Servos {ids} @ {args.rate:.0f} Hz. Ctrl-C to stop and save.\n")
    for n in range(args.countdown, 0, -1):
        print(f"  releasing torque in {n} ...", end="\r", flush=True); time.sleep(1)

    # go limp
    for sid in ids:
        servo.write1ByteTxRx(sid, SMS_STS_TORQUE_ENABLE, 0)
    print("  torque OFF -- move the arm by hand.            ")

    period = 1.0 / args.rate
    samples = []
    last = [cal["joints"][n]["zero_tick"] for n in JOINTS]   # fallback on read glitch
    t0 = time.time(); next_t = t0
    try:
        while True:
            ticks = []
            for k, sid in enumerate(ids):
                pos, _, res, _ = servo.ReadPosSpeed(sid)
                if res == 0:
                    last[k] = pos
                ticks.append(last[k])
            t = time.time() - t0
            samples.append({"t": round(t, 4), "ticks": ticks})
            print(f"  t={t:6.2f}s  n={len(samples):5d}  ticks={ticks}", end="\r", flush=True)
            next_t += period
            slp = next_t - time.time()
            if slp > 0:
                time.sleep(slp)
            else:
                next_t = time.time()   # fell behind; resync
    except KeyboardInterrupt:
        pass
    finally:
        ph.closePort()

    dur = samples[-1]["t"] if samples else 0.0
    out = args.out or os.path.join(
        HERE, f"teleop_left_{datetime.now():%Y%m%d_%H%M%S}.json")
    rec = {
        "type": "teleop_recording",
        "arm": "left",
        "port": cal["port"], "baud": cal["baud"],
        "servo_ids": ids,
        "joint_names": JOINTS,
        "rate_hz": args.rate,
        "created": datetime.now().isoformat(timespec="seconds"),
        "duration_s": round(dur, 3),
        "n_samples": len(samples),
        "samples": samples,
    }
    with open(out, "w") as f:
        json.dump(rec, f)
    eff = len(samples) / dur if dur > 0 else 0.0
    print(f"\n\nsaved {len(samples)} samples over {dur:.2f}s "
          f"(~{eff:.0f} Hz effective) -> {out}")
    print("torque is OFF -- set the arm down gently. Replay with teleop_replay.py")


if __name__ == "__main__":
    main()
