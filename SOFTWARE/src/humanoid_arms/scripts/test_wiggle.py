from scservo_sdk import *
import time

PORT = '/dev/ttyACM0'
BAUD = 1000000

ph = PortHandler(PORT)
servo = sms_sts(ph)
ph.openPort()
ph.setBaudRate(BAUD)

sid = int(input("Enter servo ID to test (2-8): "))

center, speed, result, error = servo.ReadPosSpeed(sid)
if result != 0:
    print(f"Could not read servo {sid}, aborting")
    ph.closePort()
    exit()

print(f"Center position: {center}")
step = int(input("Wiggle size in position units (start small, e.g. 50 = ~4.4 deg): "))

print("Starting in...")
for i in range(3, 0, -1):
    print(f"  {i}...")
    time.sleep(1)

def move_to(p):
    p = max(0, min(4095, p))   # clamp to valid range
    servo.WritePosEx(sid, p, 200, 30)  # slow speed, gentle accel
    time.sleep(1.2)

print("Wiggling one way...")
move_to(center + step)
print("Back to center...")
move_to(center)
print("Wiggling the other way...")
move_to(center - step)
print("Back to center...")
move_to(center)

final, _, _, _ = servo.ReadPosSpeed(sid)
print(f"Returned to {final} (started at {center})")

servo.write1ByteTxRx(sid, SMS_STS_TORQUE_ENABLE, 0)  # disable torque, joint goes limp
print("Torque disabled, joint is now free to move by hand")
ph.closePort()
