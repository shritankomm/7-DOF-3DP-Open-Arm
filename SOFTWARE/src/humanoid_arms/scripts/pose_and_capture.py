#!/usr/bin/env python3
"""
Pose-and-capture for the humanoid_arms 7-DOF arms.

Spawns its own 6-DOF interactive marker (topic /pose_capture/update). Drag it in
RViz; /compute_ik solves for the arm and the solution is published DIRECTLY to
/<arm>_arm_controller/joint_trajectory -- bypassing OMPL entirely. The transparent
start-state arm in RViz (the real robot state) follows the marker live.

Keyboard (in this terminal, one key + ENTER):
    <ENTER>  capture current /joint_states as a keyframe
    l        list captured keyframes
    u        undo (drop last keyframe)
    s        save keyframes JSON now
    q        save + quit

Writes keyframes_<arm>_<timestamp>.json next to where you run it.

Requires demo.launch.py running (provides /compute_ik, /compute_fk, the
controllers, and RViz). In RViz: Add -> InteractiveMarkers -> Update Topic
/pose_capture/update  to see and drag the marker.

Usage:
    python3 pose_and_capture.py            # left arm (default)
    python3 pose_and_capture.py left
    python3 pose_and_capture.py right
"""

import json
import sys
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)
from interactive_markers import InteractiveMarkerServer

# ---- config -----------------------------------------------------------------
BASE_LINK = "chest_plate"      # shared base for both arms (world/root TF is broken)
AVOID_COLLISIONS = False       # OMPL/collision goal-checking is the deferred-broken bit
FOLLOW_TIME = 0.30             # sec the controller takes to reach each IK solution
IK_RATE = 0.05                 # sec between IK solves while dragging (20 Hz)
IK_TIMEOUT = 0.1               # per-solve budget handed to pick_ik
# -----------------------------------------------------------------------------

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


class PoseCapture(Node):
    def __init__(self, cfg):
        super().__init__("pose_and_capture")
        self.group = cfg["group"]
        self.tip = cfg["tip"]
        self.joint_names = cfg["joints"]
        self.controller_topic = cfg["controller"]

        self.cb = ReentrantCallbackGroup()

        # services (shared with ik_trajectory.py's proven plumbing)
        self.ik = self.create_client(GetPositionIK, "/compute_ik", callback_group=self.cb)
        self.fk = self.create_client(GetPositionFK, "/compute_fk", callback_group=self.cb)
        for cli, name in [(self.ik, "/compute_ik"), (self.fk, "/compute_fk")]:
            if not cli.wait_for_service(timeout_sec=10.0):
                self.get_logger().error(f"{name} not available -- is demo.launch.py running?")
                sys.exit(1)

        # latest joint states (what we capture, and seed IK with for continuity)
        self.latest_js = {}
        self.create_subscription(JointState, "/joint_states", self._on_js, 10,
                                 callback_group=self.cb)

        # direct-to-controller publisher (bypasses OMPL)
        self.traj_pub = self.create_publisher(JointTrajectory, self.controller_topic, 10)

        # marker follow state
        self._target_pose = None      # geometry_msgs/Pose in BASE_LINK, set by feedback
        self._pose_frame = BASE_LINK
        self._dirty = False           # target changed since last solve
        self._solving = False         # a call_async is in flight
        self._lock = threading.Lock()

        self.keyframes = []           # list of [7] joint positions
        self.last_solve_ok = None

    # ---- callbacks ----------------------------------------------------------
    def _on_js(self, msg: JointState):
        self.latest_js.update(dict(zip(msg.name, msg.position)))

    def _on_feedback(self, feedback: InteractiveMarkerFeedback):
        if feedback.event_type != InteractiveMarkerFeedback.POSE_UPDATE:
            return
        with self._lock:
            self._target_pose = feedback.pose
            self._pose_frame = feedback.header.frame_id or BASE_LINK
            self._dirty = True

    def _tick(self):
        """Timer at IK_RATE: if the marker moved, solve IK and command the arm."""
        with self._lock:
            if not self._dirty or self._solving or self._target_pose is None:
                return
            pose = self._target_pose
            frame = self._pose_frame
            self._dirty = False
            self._solving = True

        ps = PoseStamped()
        ps.header.frame_id = frame
        ps.pose = pose

        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group
        req.ik_request.ik_link_name = self.tip
        req.ik_request.pose_stamped = ps
        req.ik_request.avoid_collisions = AVOID_COLLISIONS
        req.ik_request.timeout.nanosec = int(IK_TIMEOUT * 1e9)
        seed = self._current_arm_positions()
        if seed is not None:
            req.ik_request.robot_state = self._robot_state(seed)

        fut = self.ik.call_async(req)
        fut.add_done_callback(self._on_ik_done)

    def _on_ik_done(self, fut):
        resp = fut.result()
        with self._lock:
            self._solving = False
        if resp is None or resp.error_code.val != 1:
            self.last_solve_ok = False
            return
        m = dict(zip(resp.solution.joint_state.name, resp.solution.joint_state.position))
        try:
            sol = [m[j] for j in self.joint_names]
        except KeyError:
            self.last_solve_ok = False
            return
        self.last_solve_ok = True
        self._command(sol)

    # ---- helpers ------------------------------------------------------------
    def _robot_state(self, positions):
        rs = RobotState()
        js = JointState()
        js.name = list(self.joint_names)
        js.position = list(positions)
        rs.joint_state = js
        return rs

    def _current_arm_positions(self):
        try:
            return [self.latest_js[j] for j in self.joint_names]
        except KeyError:
            return None

    def _command(self, positions):
        traj = JointTrajectory()
        traj.joint_names = list(self.joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        pt.time_from_start.sec = int(FOLLOW_TIME)
        pt.time_from_start.nanosec = int((FOLLOW_TIME % 1.0) * 1e9)
        traj.points = [pt]
        self.traj_pub.publish(traj)

    def fk_tip(self, positions):
        """Blocking FK -- called once at startup before the executor thread runs."""
        req = GetPositionFK.Request()
        req.header.frame_id = BASE_LINK
        req.fk_link_names = [self.tip]
        req.robot_state = self._robot_state(positions)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        resp = fut.result()
        if resp is None or resp.error_code.val != 1 or not resp.pose_stamped:
            return None
        return resp.pose_stamped[0]

    # ---- keyframe ops -------------------------------------------------------
    def capture(self):
        pos = self._current_arm_positions()
        if pos is None:
            print("  ! no /joint_states yet -- nothing captured")
            return
        self.keyframes.append(pos)
        print(f"  captured keyframe {len(self.keyframes) - 1}: "
              + " ".join(f"{p:+.3f}" for p in pos))

    def undo(self):
        if not self.keyframes:
            print("  ! nothing to undo")
            return
        self.keyframes.pop()
        print(f"  dropped last keyframe -> {len(self.keyframes)} remain")

    def list_keyframes(self):
        if not self.keyframes:
            print("  (no keyframes yet)")
            return
        for i, kf in enumerate(self.keyframes):
            print(f"  {i:02d}: " + " ".join(f"{p:+.3f}" for p in kf))

    def save(self, arm):
        if not self.keyframes:
            print("  ! no keyframes to save")
            return None
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = f"keyframes_{arm}_{stamp}.json"
        data = {
            "arm": arm,
            "group": self.group,
            "tip": self.tip,
            "controller": self.controller_topic,
            "joint_names": self.joint_names,
            "created": datetime.now().isoformat(timespec="seconds"),
            "keyframes": [
                {"index": i, "positions": kf} for i, kf in enumerate(self.keyframes)
            ],
        }
        with open(path, "w") as f:
            json.dump(data, f, indent=2)
        print(f"  saved {len(self.keyframes)} keyframes -> {path}")
        return path


def build_marker(pose_stamped, frame):
    """A full 6-DOF interactive marker (3 translate arrows + 3 rotate rings)
    seeded at the arm's tip, with a visible cyan sphere at the center.

    The explicit per-axis handles stick out from the wrist mesh so they are easy
    to grab and unmistakably distinct from MoveIt's own end-effector marker."""
    im = InteractiveMarker()
    im.header.frame_id = frame
    im.name = "pose_capture_target"
    im.description = "pose-capture target"
    im.scale = 0.20
    im.pose = pose_stamped.pose

    # visible cyan sphere at the center (always shown, not just on hover)
    sphere = Marker()
    sphere.type = Marker.SPHERE
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.05
    sphere.color.r = 0.2
    sphere.color.g = 0.8
    sphere.color.b = 1.0
    sphere.color.a = 0.9
    vis = InteractiveMarkerControl()
    vis.always_visible = True
    vis.markers.append(sphere)
    im.controls.append(vis)

    # one translate-arrow pair + one rotate-ring per axis (classic 6-DOF handles)
    for x, y, z in [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]:
        norm = (1.0 + x * x + y * y + z * z) ** 0.5
        qw, qx, qy, qz = 1.0 / norm, x / norm, y / norm, z / norm
        for mode, tag in [(InteractiveMarkerControl.ROTATE_AXIS, "rotate"),
                          (InteractiveMarkerControl.MOVE_AXIS, "move")]:
            c = InteractiveMarkerControl()
            c.orientation.w, c.orientation.x = qw, qx
            c.orientation.y, c.orientation.z = qy, qz
            c.name = f"{tag}_{int(x)}{int(y)}{int(z)}"
            c.interaction_mode = mode
            c.always_visible = True
            im.controls.append(c)
    return im


def main():
    arm = sys.argv[1].lower() if len(sys.argv) > 1 else "left"
    if arm not in ARM_CONFIG:
        print(f"Unknown arm '{arm}'. Use 'left' or 'right'.")
        return
    cfg = ARM_CONFIG[arm]

    rclpy.init()
    node = PoseCapture(cfg)
    print(f"=== pose-and-capture: {arm.upper()} arm "
          f"(group={cfg['group']}, tip={cfg['tip']}) ===")

    # wait for first /joint_states so we can seed the marker at the real tip
    print("Waiting for /joint_states ...")
    t0 = time.time()
    while node._current_arm_positions() is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - t0 > 10.0:
            print("  ! no /joint_states after 10s -- is the sim up? continuing at zeros")
            break
    seed = node._current_arm_positions() or [0.0] * 7

    anchor = node.fk_tip(seed)
    if anchor is None:
        print("FK failed -- couldn't locate the tip. Check joint names / service.")
        rclpy.shutdown()
        return
    frame = anchor.header.frame_id
    p = anchor.pose.position
    print(f"Marker seeded at tip x={p.x:.3f} y={p.y:.3f} z={p.z:.3f} in '{frame}'")

    # interactive marker on /pose_capture/update
    server = InteractiveMarkerServer(node, "pose_capture")
    im = build_marker(anchor, frame)
    server.insert(im, feedback_callback=node._on_feedback,
                  feedback_type=InteractiveMarkerFeedback.POSE_UPDATE)
    server.applyChanges()

    # solve/command loop
    node.create_timer(IK_RATE, node._tick, callback_group=node.cb)

    # spin ROS in the background; keyboard drives the main thread
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    def _spin():
        try:
            executor.spin()
        except Exception:
            pass  # swallow the teardown-race exception when we shut down

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()

    print("\nIn RViz: Add -> InteractiveMarkers -> Update Topic /pose_capture/update")
    print("Drag the cyan sphere; the arm follows live.")
    print("Keys:  <ENTER> capture   l list   u undo   s save   q save+quit\n")

    try:
        while True:
            cmd = input("pose> ").strip().lower()
            if cmd == "":
                node.capture()
            elif cmd == "l":
                node.list_keyframes()
            elif cmd == "u":
                node.undo()
            elif cmd == "s":
                node.save(arm)
            elif cmd == "q":
                node.save(arm)
                break
            else:
                print("  ? keys: <ENTER> capture  l list  u undo  s save  q save+quit")
    except (EOFError, KeyboardInterrupt):
        print("\ninterrupted -- saving")
        node.save(arm)

    # ordered teardown so the spin thread stops before the context is destroyed
    executor.shutdown()
    spin_thread.join(timeout=2.0)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
