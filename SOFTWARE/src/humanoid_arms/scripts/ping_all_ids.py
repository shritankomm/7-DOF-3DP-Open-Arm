from scservo_sdk import *

PORT = '/dev/ttyACM0'
BAUD = 1000000
EXPECTED_IDS = list(range(2, 18))  # 2-8 left arm, 9 left claw, 10-16 right arm, 17 right claw

ph = PortHandler(PORT)
servo = sms_sts(ph)
ph.openPort()
ph.setBaudRate(BAUD)

print("Pinging all expected servo IDs...\n")
found = []
for sid in EXPECTED_IDS:
    model, result, error = servo.ping(sid)
    if result == 0:
        print(f"  ID {sid}: OK (model {model})")
        found.append(sid)
    else:
        print(f"  ID {sid}: NO RESPONSE")

print(f"\n{len(found)}/{len(EXPECTED_IDS)} servos responding: {found}")
ph.closePort()
