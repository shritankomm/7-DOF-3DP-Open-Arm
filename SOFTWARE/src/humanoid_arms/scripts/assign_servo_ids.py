from scservo_sdk import *

PORT = '/dev/ttyACM0'
BAUD = 1000000
DEFAULT_ID = 1  # every unconfigured servo sits here

# Edit this to match your joint scheme, in the order you'll plug them in
TARGET_IDS = [2, 3, 4, 5, 6, 7, 8]

ph = PortHandler(PORT)
servo = sms_sts(ph)
ph.openPort()
ph.setBaudRate(BAUD)

for new_id in TARGET_IDS:
    input(f"\nPlug in the next unconfigured servo, then press Enter to assign ID {new_id}...")

    model, result, error = servo.ping(DEFAULT_ID)
    if result != 0:
        print(f"  No servo responding at default ID {DEFAULT_ID}, check the connection")
        continue

    servo.unLockEprom(DEFAULT_ID)
    servo.write1ByteTxRx(DEFAULT_ID, SMS_STS_ID, new_id)
    servo.LockEprom(new_id)

    model, result, error = servo.ping(new_id)
    if result == 0:
        print(f"  Success, servo now responds at ID {new_id}")
    else:
        print(f"  Something went wrong, ID {new_id} isn't responding")

ph.closePort()
print("\nAll done. Run the ping-all-IDs script to confirm every joint responds correctly.")
