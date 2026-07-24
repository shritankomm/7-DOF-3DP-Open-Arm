from scservo_sdk import *
import time

PORT = '/dev/ttyACM0'
BAUD = 1000000

ph = PortHandler(PORT)
servo = sms_sts(ph)
ph.openPort()
ph.setBaudRate(BAUD)

id_a = int(input("Enter first servo ID: "))
id_b = int(input("Enter second servo ID: "))

center_a, _, res_a, _ = servo.ReadPosSpeed(id_a)
center_b, _, res_b, _ = servo.ReadPosSpeed(id_b)
if res_a != 0 or res_b != 0:
    print("Could not read one of the servos, aborting")
    ph.closePort()
    exit()

print(f"Center A (ID {id_a}): {center_a}")
print(f"Center B (ID {id_b}): {center_b}")
step = int(input("Wiggle size in position units (start small, e.g. 50): "))

print("Starting in...")
for i in range(3, 0, -1):
    print(f"  {i}...")
    time.sleep(1)

def sync_move(pos_a, pos_b):
    pos_a = max(0, min(4095, pos_a))
    pos_b = max(0, min(4095, pos_b))
    # queue both targets, then fire them together
    servo.SyncWritePosEx(id_a, pos_a, 200, 30)
    servo.SyncWritePosEx(id_b, pos_b, 200, 30)
    servo.groupSyncWrite.txPacket()
    servo.groupSyncWrite.clearParam()
    time.sleep(1.2)

print("Wiggling one way...")
sync_move(center_a + step, center_b + step)
print("Back to center...")
sync_move(center_a, center_b)
print("Wiggling the other way...")
sync_move(center_a - step, center_b - step)
print("Back to center...")
sync_move(center_a, center_b)

servo.write1ByteTxRx(id_a, SMS_STS_TORQUE_ENABLE, 0)
servo.write1ByteTxRx(id_b, SMS_STS_TORQUE_ENABLE, 0)
print("Torque disabled on both, joints free to move by hand")
ph.closePort()
