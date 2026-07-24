#!/usr/bin/env python3
"""
Determine the SIGN (+1/-1) of each left-arm joint for the radians->ticks mapping.

For each joint it:
  1. Commands the SIM (RViz) to rotate THAT joint to a clear positive angle,
     leaving the others at zero -- you watch which way it rotates in RViz.
  2. Gently jogs the REAL servo a small amount (direction chosen to stay in the
     0-4095 range, so near-limit joints like the elbow jog the safe way).
  3. Asks you: did the real joint move the SAME way as RViz, or OPPOSITE?
       same    -> sign = jog direction
       opposite-> sign = -jog direction
Then returns that joint to zero and moves on. Torque is disabled on all servos
at the end and on Ctrl-C.

Needs BOTH the ROS env (to command the sim) and servo access. Run in a terminal
where you've sourced install/setup.bash, with demo.launch.py running:
    cd ~/ros2_ws && source install/setup.bash
    python3 src/humanoid_arms/scripts/sign_calibrate.py
"""

import json
import os
import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from scservo_sdk import *   # noqa: F401,F403

HERE = os.path.dirname(os.path.abspath(__file__))
CAL = os.path.join(HERE, "servo_calibration.json")
CONTROLLER = "/left_arm_controller/joint_trajectory"
JOINTS = ["leftshoulderpitch", "leftshoulderroll", "leftarmyaw", "leftelbowpitch",
          "leftforearmyaw", "leftwristpitch", "leftwristroll"]
REF_ANGLE = 0.6      # rad the sim joint rotates (clear, positive, within limits)
JOG = 250            # tick jog on the real servo (~22 deg)
SPEED = 150          # slow
ACCEL = 20           # gentle
SETTLE = 1.5


class SimCmd(Node):
    def __init__(self):
        super().__init__("sign_calibrate_sim")
        self.pub = self.create_publisher(JointTrajectory, CONTROLLER, 10)
        time.sleep(0.5)

    def pose(self, positions, secs=1.5):
        t = JointTrajectory(); t.joint_names = list(JOINTS)
        p = JointTrajectoryPoint()
        p.positions = [float(x) for x in positions]
        p.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        t.points = [p]
        self.pub.publish(t)


def main():
    cal = json.load(open(CAL))
    port = cal.get("port", "/dev/ttyACM0"); baud = cal.get("baud", 1000000)

    rclpy.init()
    sim = SimCmd()

    ph = PortHandler(port); servo = sms_sts(ph)
    if not ph.openPort():
        print(f"!! could not open {port} (permission? dialout group?)"); rclpy.shutdown(); return
    ph.setBaudRate(baud)

    def torque_off_all():
        for j in cal["joints"].values():
            servo.write1ByteTxRx(j["servo_id"], SMS_STS_TORQUE_ENABLE, 0)

    print("Make sure the arm is at its ZERO pose to start.\n")
    try:
        sim.pose([0.0] * 7)
        time.sleep(SETTLE)
        for name in JOINTS:
            j = cal["joints"][name]
            sid = j["servo_id"]; zero = j["zero_tick"]
            jog_dir = 1 if zero <= 4095 - JOG else -1     # stay in range
            target = zero + jog_dir * JOG

            while True:
                # 1) sim rotates ONLY this joint, positive
                pos = [REF_ANGLE if n == name else 0.0 for n in JOINTS]
                sim.pose(pos); print(f"\n=== {name} (ID {sid}) ===")
                print(f"  RViz: {name} rotating to +{REF_ANGLE} rad -- note which way it goes")
                time.sleep(SETTLE + 0.5)
                # 2) jog the real servo
                print(f"  real: jogging servo {jog_dir*JOG:+d} ticks ...")
                servo.WritePosEx(sid, target, SPEED, ACCEL); time.sleep(SETTLE)
                # 3) compare
                ans = input("  Real joint moved SAME as RViz or OPPOSITE? [s/o, r=repeat] ").strip().lower()
                # return this joint to zero (sim + real) before deciding/moving on
                servo.WritePosEx(sid, zero, SPEED, ACCEL)
                sim.pose([0.0] * 7); time.sleep(SETTLE)
                if ans == "r":
                    continue
                if ans in ("s", "o"):
                    j["sign"] = jog_dir if ans == "s" else -jog_dir
                    print(f"  => {name} sign = {j['sign']:+d}")
                    break
                print("  ? enter s, o, or r")

        print("\n=== SIGNS ===")
        for name in JOINTS:
            print(f"  {name:20s} sign = {cal['joints'][name]['sign']:+d}")
        json.dump(cal, open(CAL, "w"), indent=2)
        print(f"\nsaved -> {CAL}")
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        torque_off_all()
        ph.closePort()
        rclpy.shutdown()
        print("torque disabled on all servos, port closed.")


if __name__ == "__main__":
    main()
