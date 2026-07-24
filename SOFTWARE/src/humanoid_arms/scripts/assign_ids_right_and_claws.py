#!/usr/bin/env python3
"""
Assign servo IDs for the LEFT CLAW (9) and the whole RIGHT ARM (10-17).

Left arm is already IDs 2-8 and is left untouched. Every FACTORY-FRESH Feetech
servo ships at ID 1, so we assign one servo at a time: plug in ONE new (ID 1)
servo, press Enter, it gets the next ID in the list.

    ID  9 : leftgripper       (left claw)
    ID 10 : rightshoulderpitch
    ID 11 : rightshoulderroll
    ID 12 : rightarmyaw
    ID 13 : rightelbowpitch
    ID 14 : rightforearmyaw
    ID 15 : rightwristpitch
    ID 16 : rightwristroll
    ID 17 : rightgripper       (right claw)

CRITICAL: only ONE unconfigured (ID 1) servo may be on the bus at a time. Two
servos both sitting at ID 1 collide on the wire and the write will fail or hit
the wrong one. If your right arm is pre-chained, either add servos to the chain
one at a time, or isolate each servo, assign it, then chain it in.

Safe to re-run: it ping-checks the target ID first and refuses to overwrite a
servo that already lives there (so it can't stomp your 2-8 left arm or a servo
you assigned on a previous pass).

RESUME: pass the IDs you still need on the command line to skip the rest, e.g.
    python3 assign_ids_right_and_claws.py 15 16 17
With no args it does the whole 9-17 list.
"""
import sys
from scservo_sdk import *

PORT = '/dev/ttyACM0'
BAUD = 1000000
DEFAULT_ID = 1  # every unconfigured servo sits here

# (target_id, human-readable joint)  -- assigned in this order
TARGETS = [
    (9,  "leftgripper (left claw)"),
    (10, "rightshoulderpitch"),
    (11, "rightshoulderroll"),
    (12, "rightarmyaw"),
    (13, "rightelbowpitch"),
    (14, "rightforearmyaw"),
    (15, "rightwristpitch"),
    (16, "rightwristroll"),
    (17, "rightgripper (right claw)"),
]

# Optional resume: only assign the IDs passed on the command line.
if len(sys.argv) > 1:
    wanted = {int(a) for a in sys.argv[1:]}
    TARGETS = [(i, j) for (i, j) in TARGETS if i in wanted]
    if not TARGETS:
        raise SystemExit(f"None of {sorted(wanted)} are in the 9-17 list. Nothing to do.")

ph = PortHandler(PORT)
servo = sms_sts(ph)
if not ph.openPort():
    raise SystemExit(f"Failed to open {PORT} -- is the adapter plugged in and readable?")
ph.setBaudRate(BAUD)

print("Assigning IDs for the left claw + right arm.")
print("Left arm (2-8) is untouched. Only ONE new servo on the bus at a time.\n")

for new_id, joint in TARGETS:
    input(f"Plug in the servo for ID {new_id} ({joint}), then press Enter... ")

    # Guardrail 1: refuse to overwrite an ID that's already taken.
    _, result, _ = servo.ping(new_id)
    if result == 0:
        print(f"  SKIP: ID {new_id} is ALREADY in use. Not overwriting. "
              f"(Is this servo already assigned, or is it a left-arm servo?)\n")
        continue

    # Guardrail 2: the new servo must actually be sitting at the factory ID 1.
    _, result, _ = servo.ping(DEFAULT_ID)
    if result != 0:
        print(f"  No servo responding at default ID {DEFAULT_ID}. "
              f"Check power (12V), the data cable, and that exactly one fresh servo is connected.\n")
        continue

    servo.unLockEprom(DEFAULT_ID)
    servo.write1ByteTxRx(DEFAULT_ID, SMS_STS_ID, new_id)
    servo.LockEprom(new_id)

    _, result, _ = servo.ping(new_id)
    if result == 0:
        print(f"  OK: servo now responds at ID {new_id} ({joint}).\n")
    else:
        print(f"  FAILED: ID {new_id} isn't responding after the write. Retry this one.\n")

ph.closePort()
print("Done. Run  python3 ping_all_ids.py  (after updating its EXPECTED_IDS to 2-17) to confirm.")
