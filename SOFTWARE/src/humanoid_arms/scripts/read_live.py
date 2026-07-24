#!/usr/bin/env python3
"""
Live servo position readout. Disables torque on the given servo so you can move
that joint BY HAND, and prints its tick continuously. Use it to read a joint's
tick at known physical positions (e.g. elbow fully straight vs fully bent).

Usage:
    python3 read_live.py 5      # servo ID 5 = leftelbowpitch
Ctrl-C to stop (leaves the servo limp).
"""
import sys
import time

from scservo_sdk import *   # noqa: F401,F403

PORT = "/dev/ttyACM0"
BAUD = 1000000

sid = int(sys.argv[1]) if len(sys.argv) > 1 else 5

ph = PortHandler(PORT)
servo = sms_sts(ph)
if not ph.openPort():
    print(f"!! could not open {PORT} (permission?)")
    sys.exit(1)
ph.setBaudRate(BAUD)

servo.write1ByteTxRx(sid, SMS_STS_TORQUE_ENABLE, 0)   # limp: move it by hand
print(f"Servo {sid} torque OFF -- move the joint by hand.")
print("Reading tick live (Ctrl-C to stop):")
try:
    while True:
        pos, _, res, _ = servo.ReadPosSpeed(sid)
        if res == 0:
            print(f"    tick = {pos:4d}        ", end="\r", flush=True)
        time.sleep(0.15)
except KeyboardInterrupt:
    print("\nstopped.")
finally:
    ph.closePort()
