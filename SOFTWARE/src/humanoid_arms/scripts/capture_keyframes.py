#!/usr/bin/env python3
"""
Capture keyframes for the humanoid_arms 7-DOF arms.

Workflow: pose the arm however you like (MoveIt Plan & Execute is the intended
way now that planning works), then snapshot the current joint angles as a
keyframe. No interactive marker -- posing is MoveIt's job; this just records.

Keyboard (in this terminal, one key + ENTER):
    <ENTER>  capture the current /joint_states as a keyframe
    l        list captured keyframes
    u        undo (drop last keyframe)
    s        save keyframes JSON now
    q        save + quit

Writes keyframes_<arm>_<timestamp>.json (replay with replay_keyframes.py).

Usage:
    python3 capture_keyframes.py            # left arm (default)
    python3 capture_keyframes.py left
    python3 capture_keyframes.py right
"""

import json
import sys
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM_CONFIG = {
    "left": {
        "group": "left_arm",
        "tip": "wrist_holder",
        "controller": "/left_arm_controller/joint_trajectory",
        "joints": ["leftshoulderpitch", "leftshoulderroll", "leftarmyaw",
                   "leftelbowpitch", "leftforearmyaw", "leftwristpitch", "leftwristroll"],
    },
    "right": {
        "group": "right_arm",
        "tip": "part_51",
        "controller": "/right_arm_controller/joint_trajectory",
        "joints": ["rightshoulderpitch", "rightshoulderroll", "rightarmyaw",
                   "rightelbowpitch", "rightforearmyaw", "rightwristpitch", "rightwristroll"],
    },
}


class Capture(Node):
    def __init__(self, cfg):
        super().__init__("capture_keyframes")
        self.cfg = cfg
        self.joint_names = cfg["joints"]
        self.latest = {}
        self.keyframes = []
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

    def _on_js(self, msg):
        self.latest.update(dict(zip(msg.name, msg.position)))

    def current(self):
        try:
            return [self.latest[j] for j in self.joint_names]
        except KeyError:
            return None

    def capture(self):
        pos = self.current()
        if pos is None:
            print("  ! no /joint_states yet -- nothing captured")
            return
        self.keyframes.append(pos)
        print(f"  captured keyframe {len(self.keyframes) - 1}: "
              + " ".join(f"{p:+.3f}" for p in pos))

    def undo(self):
        if not self.keyframes:
            print("  ! nothing to undo"); return
        self.keyframes.pop()
        print(f"  dropped last -> {len(self.keyframes)} remain")

    def list_kf(self):
        if not self.keyframes:
            print("  (no keyframes yet)"); return
        for i, kf in enumerate(self.keyframes):
            print(f"  {i:02d}: " + " ".join(f"{p:+.3f}" for p in kf))

    def save(self, arm):
        if not self.keyframes:
            print("  ! no keyframes to save"); return None
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = f"keyframes_{arm}_{stamp}.json"
        data = {
            "arm": arm,
            "group": self.cfg["group"],
            "tip": self.cfg["tip"],
            "controller": self.cfg["controller"],
            "joint_names": self.joint_names,
            "created": datetime.now().isoformat(timespec="seconds"),
            "keyframes": [{"index": i, "positions": kf}
                          for i, kf in enumerate(self.keyframes)],
        }
        with open(path, "w") as f:
            json.dump(data, f, indent=2)
        print(f"  saved {len(self.keyframes)} keyframes -> {path}")
        return path


def main():
    arm = sys.argv[1].lower() if len(sys.argv) > 1 else "left"
    if arm not in ARM_CONFIG:
        print(f"Unknown arm '{arm}'. Use 'left' or 'right'."); return

    rclpy.init()
    node = Capture(ARM_CONFIG[arm])
    print(f"=== capture keyframes: {arm.upper()} arm ===")
    print("Waiting for /joint_states ...")
    t0 = time.time()
    while node.current() is None and time.time() - t0 < 10.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.current() is None:
        print("  ! no /joint_states after 10s -- is demo.launch.py running?")
        rclpy.shutdown(); return

    def spin():
        try:
            rclpy.spin(node)
        except Exception:
            pass
    th = threading.Thread(target=spin, daemon=True)
    th.start()

    print("Pose the arm (MoveIt Plan & Execute), then snapshot it here.")
    print("Keys:  <ENTER> capture   l list   u undo   s save   q save+quit\n")
    try:
        while True:
            cmd = input("capture> ").strip().lower()
            if cmd == "":
                node.capture()
            elif cmd == "l":
                node.list_kf()
            elif cmd == "u":
                node.undo()
            elif cmd == "s":
                node.save(arm)
            elif cmd == "q":
                node.save(arm); break
            else:
                print("  ? keys: <ENTER> capture  l list  u undo  s save  q save+quit")
    except (EOFError, KeyboardInterrupt):
        print("\ninterrupted -- saving")
        node.save(arm)

    rclpy.shutdown()
    th.join(timeout=2.0)


if __name__ == "__main__":
    main()
