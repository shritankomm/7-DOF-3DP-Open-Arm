#!/usr/bin/env python3
"""
Replay captured keyframes for the humanoid_arms arms.

Reads a keyframes_<arm>_<timestamp>.json (from capture_keyframes.py) and plays
the sequence back through the arm's joint_trajectory controller. The controller
interpolates between keyframes, so the arm reproduces the recorded motion.

Usage:
    python3 replay_keyframes.py FILE.json                # smooth playback
    python3 replay_keyframes.py FILE.json --dt 1.5       # 1.5 s per keyframe
    python3 replay_keyframes.py FILE.json --step         # ENTER between each
    python3 replay_keyframes.py FILE.json --goto 3       # jump to keyframe 3 only
    python3 replay_keyframes.py FILE.json --loop 2       # repeat whole thing twice

Requires demo.launch.py running (provides the controllers). Playback MOVES the
(sim) arm for real -- it publishes to the controller command topic.
"""

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def dur(seconds):
    d = Duration()
    d.sec = int(seconds)
    d.nanosec = int((seconds - int(seconds)) * 1e9)
    return d


class Replay(Node):
    def __init__(self, controller_topic, joint_names):
        super().__init__("replay_keyframes")
        self.joint_names = joint_names
        self.pub = self.create_publisher(JointTrajectory, controller_topic, 10)
        # give discovery a moment so the first message isn't dropped
        t0 = time.time()
        while self.pub.get_subscription_count() == 0 and time.time() - t0 < 5.0:
            time.sleep(0.1)

    def _traj(self, points_with_times):
        traj = JointTrajectory()
        traj.joint_names = list(self.joint_names)
        for positions, t in points_with_times:
            pt = JointTrajectoryPoint()
            pt.positions = [float(p) for p in positions]
            pt.time_from_start = dur(t)
            traj.points.append(pt)
        return traj

    def play_smooth(self, keyframes, dt):
        """One trajectory, all keyframes at t = dt, 2dt, ... controller chains them."""
        pts = [(kf, (i + 1) * dt) for i, kf in enumerate(keyframes)]
        self.pub.publish(self._traj(pts))
        total = len(keyframes) * dt
        print(f"  playing {len(keyframes)} keyframes over {total:.1f}s ...")
        time.sleep(total + 0.3)

    def play_step(self, keyframes, dt):
        for i, kf in enumerate(keyframes):
            input(f"  [ENTER] -> keyframe {i} ")
            self.pub.publish(self._traj([(kf, dt)]))
            time.sleep(dt + 0.1)

    def goto(self, kf, dt):
        print(f"  going to keyframe over {dt:.1f}s ...")
        self.pub.publish(self._traj([(kf, dt)]))
        time.sleep(dt + 0.3)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("file", help="keyframes JSON from capture_keyframes.py")
    ap.add_argument("--dt", type=float, default=2.0, help="seconds per keyframe")
    ap.add_argument("--step", action="store_true", help="pause for ENTER before each keyframe")
    ap.add_argument("--goto", type=int, metavar="N", help="jump to keyframe N only")
    ap.add_argument("--loop", type=int, default=1, help="repeat the whole sequence N times")
    args = ap.parse_args()

    try:
        data = json.load(open(args.file))
    except (OSError, json.JSONDecodeError) as e:
        print(f"could not read {args.file}: {e}"); return

    joint_names = data["joint_names"]
    controller = data["controller"]
    keyframes = [kf["positions"] for kf in data["keyframes"]]
    if not keyframes:
        print("no keyframes in file"); return
    print(f"=== replay {data.get('arm','?')} arm: {len(keyframes)} keyframes "
          f"-> {controller} ===")

    rclpy.init()
    node = Replay(controller, joint_names)
    if node.pub.get_subscription_count() == 0:
        print(f"  ! nothing subscribed to {controller} -- is demo.launch.py running?")
        rclpy.shutdown(); return

    try:
        if args.goto is not None:
            if not 0 <= args.goto < len(keyframes):
                print(f"  ! --goto {args.goto} out of range 0..{len(keyframes)-1}")
            else:
                node.goto(keyframes[args.goto], args.dt)
        else:
            for rep in range(args.loop):
                if args.loop > 1:
                    print(f"  --- loop {rep+1}/{args.loop} ---")
                if args.step:
                    node.play_step(keyframes, args.dt)
                else:
                    node.play_smooth(keyframes, args.dt)
        print("  done.")
    except (EOFError, KeyboardInterrupt):
        print("\ninterrupted")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
