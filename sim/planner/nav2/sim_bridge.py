"""Nav2 side of the planner head-to-head: the shared arena (arena2d) as a ROS 2 simulator, and the mission runner.

The node owns sim time. It steps the same skid-steer model with slip as compare.py, publishes /clock, /odom, the
odom -> base_link transform, a static map -> odom identity (Nav2 gets the true pose, as the custom planners do) and
/map (lethal cells for the obstacle discs, the berm box and the walls). It subscribes /cmd_vel and sends each leg of
each mission to Nav2 as a NavigateToPose goal, then scores the leg exactly as compare.py does and writes the runs in
compare.py's format, so compare.py's summary can put every planner side by side.

Run with Nav2 already up (run_nav2.sh does both):
  python3 sim_bridge.py out.json --label nav2_smac_mppi --seeds 100 --seed0 3000 --slip nominal [--rtf 2.0]

--rtf is the real-time factor: sim seconds per wall second. Nav2 keeps its own timers on /clock, so if it cannot keep
up its control loop runs late in sim time; the node reports the controller's command rate per leg so a run that was
too fast can be seen and rerun slower.
"""
import argparse
import json
import math
import os
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import arena2d as a2  # noqa: E402

import rclpy  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from builtin_interfaces.msg import Time as TimeMsg  # noqa: E402
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist  # noqa: E402
from nav_msgs.msg import OccupancyGrid, Odometry, Path  # noqa: E402
from nav2_msgs.action import NavigateToPose  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster  # noqa: E402

MAP_RES = 0.05              # m, must match the costmaps and the Smac lattice primitives
ODOM_HZ = 50.0


def yaw_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


def stamp(t):
    m = TimeMsg()
    m.sec = int(t)
    m.nanosec = int(round((t - int(t)) * 1e9)) % 1_000_000_000
    return m


def occupancy(w):
    """Lethal (100) cells for every obstacle disc, the berm box and a one-cell wall around the arena."""
    nx, ny = int(round(w.L / MAP_RES)) + 2, int(round(w.W / MAP_RES)) + 2
    x = (np.arange(nx) - 1 + 0.5) * MAP_RES          # cell centres; the map origin is one cell outside the walls
    y = (np.arange(ny) - 1 + 0.5) * MAP_RES
    X, Y = np.meshgrid(x, y, indexing="xy")
    occ = np.zeros((ny, nx), np.int8)
    half = MAP_RES / 2
    for o in w.obstacles:                             # any cell the disc touches
        dx = np.maximum(np.abs(X - o.x) - half, 0)
        dy = np.maximum(np.abs(Y - o.y) - half, 0)
        occ[np.hypot(dx, dy) < o.r] = 100
    b = w.berm
    occ[(X + half > b[0]) & (X - half < b[1]) & (Y + half > b[2]) & (Y - half < b[3])] = 100
    occ[0, :] = occ[-1, :] = 100
    occ[:, 0] = occ[:, -1] = 100
    return occ, (-MAP_RES, -MAP_RES)


class Bridge(Node):
    def __init__(self, rtf):
        super().__init__("arena2d_bridge", parameter_overrides=[])
        self.rtf = rtf
        self.t = 0.0
        self.cmd = (0.0, 0.0)
        self.n_cmd = 0
        self.lock = threading.Lock()
        self.robot = None
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", latched)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.n_plan = 0
        self.create_subscription(Path, "/plan", self._on_plan, 10)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = "map", "odom"
        t.transform.rotation.w = 1.0
        self.static_tf.sendTransform(t)

    def _on_plan(self, msg):
        with self.lock:
            self.n_plan += 1

    def _on_cmd(self, msg):
        with self.lock:
            self.cmd = (float(msg.linear.x), float(msg.angular.z))
            self.n_cmd += 1

    def publish_map(self, w):
        occ, origin = occupancy(w)
        m = OccupancyGrid()
        m.header.frame_id = "map"
        m.header.stamp = stamp(self.t)
        m.info.resolution = MAP_RES
        m.info.width, m.info.height = occ.shape[1], occ.shape[0]
        m.info.origin.position.x, m.info.origin.position.y = origin
        m.info.origin.orientation.w = 1.0
        m.data = occ.ravel().tolist()
        self.map_pub.publish(m)

    def _publish_state(self, vb=0.0, wb=0.0):
        c = Clock()
        c.clock = stamp(self.t)
        self.clock_pub.publish(c)
        if self.robot is None:
            return
        x, y, yaw = self.robot.pose
        q = yaw_quat(yaw)
        tf = TransformStamped()
        tf.header.stamp = stamp(self.t)
        tf.header.frame_id, tf.child_frame_id = "odom", "base_link"
        tf.transform.translation.x, tf.transform.translation.y = float(x), float(y)
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = q
        self.tf.sendTransform(tf)
        od = Odometry()
        od.header.stamp = stamp(self.t)
        od.header.frame_id, od.child_frame_id = "odom", "base_link"
        od.pose.pose.position.x, od.pose.pose.position.y = float(x), float(y)
        (od.pose.pose.orientation.x, od.pose.pose.orientation.y, od.pose.pose.orientation.z,
         od.pose.pose.orientation.w) = q
        od.twist.twist.linear.x, od.twist.twist.angular.z = float(vb), float(wb)
        self.odom_pub.publish(od)

    def advance(self, seconds, w=None, track=None):
        """Step the robot for `seconds` of sim time at the real-time factor; returns the body speed of the last step."""
        n = int(round(seconds / a2.SIM_DT))
        n_odom = int(round(1 / (ODOM_HZ * a2.SIM_DT)))
        wall0, t0 = time.time(), self.t
        vb = wb = 0.0
        for k in range(n):
            with self.lock:
                cmd = self.cmd
            if self.robot is not None:
                p0 = self.robot.pose[:2].copy()
                vb, wb = self.robot.step(cmd[0], cmd[1], a2.SIM_DT)
                if track is not None:
                    track["dist"] += float(np.hypot(*(self.robot.pose[:2] - p0)))
                    cl = a2.clearance(w, self.robot.pose)
                    track["min_clear"] = min(track["min_clear"], cl)
                    track["contact_steps"] += cl < 0
            self.t += a2.SIM_DT
            if k % n_odom == 0:
                self._publish_state(vb, wb)
            lag = (self.t - t0) / self.rtf - (time.time() - wall0)
            if lag > 0:
                time.sleep(lag)
        return vb, wb

    def goal_msg(self, g):
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = stamp(self.t)
        ps.pose.position.x, ps.pose.position.y = float(g[0]), float(g[1])
        q = yaw_quat(g[2])
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = q
        goal.pose = ps
        return goal

    def run_leg(self, w, g, timeout):
        track = {"dist": 0.0, "min_clear": np.inf, "contact_steps": 0}
        with self.lock:
            self.n_cmd, n_plan0 = 0, self.n_plan
        # sim time stands still until Nav2's first plan for this goal arrives, as planning is left out of the custom
        # planners' leg times too; the wall time it took is reported as plan_s (this machine, not the Orin NX)
        wall0 = time.time()
        fut = self.nav.send_goal_async(self.goal_msg(g))
        while not fut.done() and time.time() - wall0 < 10.0:
            time.sleep(0.001)
        gh = fut.result() if fut.done() else None
        if gh is None or not gh.accepted:
            return {"planned": False, "fail": "goal rejected", "plan_s": time.time() - wall0}
        while self.n_plan == n_plan0 and time.time() - wall0 < 10.0:
            time.sleep(0.001)
        plan_s = time.time() - wall0
        if self.n_plan == n_plan0:
            gh.cancel_goal_async()
            return {"planned": False, "fail": "no plan within 10 s", "plan_s": plan_s}
        t0 = self.t
        res_fut = gh.get_result_async()
        while not res_fut.done() and self.t - t0 < timeout:
            self.advance(0.05, w, track)
        status = None
        if not res_fut.done():
            cf = gh.cancel_goal_async()
            while not cf.done():
                self.advance(0.05, w, track)
            status = "timeout"
        else:
            status = {4: "succeeded", 5: "canceled", 6: "aborted"}.get(res_fut.result().status, str(res_fut.result().status))
        with self.lock:
            self.cmd = (0.0, 0.0)
            n_cmd = self.n_cmd
        t_end = self.t
        while abs(self.robot.vl) + abs(self.robot.vr) > 1e-6:     # wheels spin down, counted in the leg time
            self.advance(a2.SIM_DT, w, track)
        e_xy = float(np.hypot(*(self.robot.pose[:2] - np.asarray(g[:2]))))
        e_yaw = float(abs(math.degrees(a2.wrap(self.robot.pose[2] - g[2]))))
        tol = a2.v(a2.PARAMS, "goal_tol")
        reached = status == "succeeded"
        return {"planned": True, "fail": None, "plan_s": plan_s, "status": status, "time": self.t - t0, "reached": reached,
                "e_xy": e_xy, "e_yaw": e_yaw, "dist": track["dist"], "min_clearance": float(track["min_clear"]),
                "contact_s": track["contact_steps"] * a2.SIM_DT, "ok": bool(reached and e_xy <= tol[0] and e_yaw <= tol[1]),
                "cmd_hz": n_cmd / max(t_end - t0, 1e-6)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--label", default="nav2")
    ap.add_argument("--seeds", type=int, default=100)
    ap.add_argument("--seed0", type=int, default=3000)
    ap.add_argument("--slip", default="nominal")
    ap.add_argument("--rtf", type=float, default=2.0)
    ap.add_argument("--settle", type=float, default=2.5, help="wall seconds after a new map before the first goal "
                    "(the global costmap takes a new map on its next update, 1 Hz on the wall clock)")
    a = ap.parse_args()
    rclpy.init()
    node = Bridge(a.rtf)
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    runs = json.load(open(a.out))["runs"] if os.path.exists(a.out) else []
    done = {(r["seed"], r["slip"]) for r in runs}
    todo = [(level, seed) for level in a.slip.split(",") for seed in range(a.seed0, a.seed0 + a.seeds)
            if (seed, level) not in done]

    def setup(level, seed):
        w = a2.world(seed)
        start, goals = a2.mission(w, np.random.default_rng(seed))
        node.robot = a2.SkidSteer(start, level, np.random.default_rng([seed, 7]))
        node.publish_map(w)
        return w, goals

    if todo:            # Nav2's costmaps only activate once they have a map and the robot's transform
        setup(*todo[0])
    node.get_logger().info("waiting for navigate_to_pose ...")
    while not node.nav.wait_for_server(timeout_sec=1.0):
        node.advance(0.1)
    for level, seed in todo:
        w, goals = setup(level, seed)
        node.advance(a.settle * a.rtf)
        legs, ok = [], True
        for k, (kind, g) in enumerate(goals):
            m = node.run_leg(w, g, a2.v(a2.PARAMS, "leg_timeout"))
            legs.append({"k": k, "kind": kind, **m})
            node.get_logger().info(f"seed {seed} {level} leg {k} {kind}: {m.get('status')} "
                                   f"{m.get('time', 0):.1f} s, e_xy {m.get('e_xy', -1):.3f}, contact {m.get('contact_s', 0):.2f} s, "
                                   f"cmd {m.get('cmd_hz', 0):.1f} Hz")
            if not m.get("ok"):
                ok = False
                break
        runs.append({"planner": a.label, "seed": seed, "slip": level, "complete": ok, "legs": legs,
                     "time": float(sum(l.get("time", 0.0) for l in legs))})
        json.dump({"meta": {"label": a.label, "rtf": a.rtf, "map_res": MAP_RES}, "runs": runs}, open(a.out, "w"))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
