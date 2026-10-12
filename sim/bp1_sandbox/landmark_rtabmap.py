"""RTAB-Map relocalization on Isaac images before and after wheel ruts, with the team's RTAB-Map settings.

Usage (ROS 2 Humble sourced; system python3): python3 landmark_rtabmap.py [LANDMARK_DIR] [--light diffuse|low|both]
        [--starts 10,25] [--edt ~/lunabotics/edt-lunabotics-2026] [--out DIR]
LANDMARK_DIR holds isaac_landmarks.py output: per pass and light the D455-like IR image, its depth and the camera pose
for every frame of the seed-1 trip.

Stack: rtabmap_sync/rgbd_sync + rtabmap_slam/rtabmap, as in the team's EDT-Lunabotics-2026 branch comp-day-one
(src/neptune_bringup/launch/test.launch.py: rgbd_sync1 and slam_node; parameters from
src/config_pkg/params/rtabmap/rtabmap_params.yaml, read from the checkout). Every parameter of that yaml is used as is
except the overrides listed in OVERRIDES (with reasons); everything else is RTAB-Map 0.23.7's default (TEAM_DEFAULTS
names the ones that matter for this test). Odometry is ground truth (the frames' settled robot poses), so the test
isolates place recognition and relocalization, not visual-odometry drift.

Sessions:
- map: pass A (no ruts), all frames in order, incremental memory, saved to A.db.
- revisit, per pass (Arep, Acam32, B32, B32cam, B50), light and start frame k0: a copy of A.db in localization mode;
  frames k0..end are played with odometry relative to frame k0 (as after a random start, so RTAB-Map must find where it
  is). Per frame: whether a loop closure (or proximity detection) to the map was accepted, its inliers, and after the
  first relocalization the map->base_link error against ground truth. Reported: success per frame and the first frame
  of the revisit that relocalized.
"""
import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
TEAM_YAML = "src/config_pkg/params/rtabmap/rtabmap_params.yaml"
# Changed from the team's yaml / launch, and why
OVERRIDES = {
    "Mem/IncrementalMemory": ("false in revisit sessions", "localization mode on the pass-A map"),
    "Mem/InitWMWithAllNodes": ("true in revisit sessions", "load the whole pass-A map for relocalization"),
    "Rtabmap/DatabaseInMemory": ("false", "the map must be saved to A.db and reloaded"),
    "subscribe_scan_cloud": ("false", "no LiDAR cloud in this test (team: Point-LIO cloud)"),
    "Grid/Sensor": ("1 (depth only)", "team 2 uses the scan cloud, which is absent here"),
    "odom_topic": ("/odom_gt", "ground-truth odometry instead of the UKF's /odometry/filtered"),
    "use_sim_time": ("true", "frames are played on a simulated clock"),
    "rgbd_sync approx_sync": ("false", "image and depth carry identical stamps here"),
    "qos / qos_image / qos_camera_info": ("1 (reliable)", "team 2 (best effort) drops large 848x480 depth frames on "
                                         "one machine: only 1 in 4-6 frames got through in the first test"),
    "imu_topic / wait_imu_to_init": ("not used", "no IMU in this test"),
}
WRONG_M = 0.5       # an accepted relocalization with a pose error above this is a wrong one
TEAM_DEFAULTS = ["Kp/DetectorStrategy", "Vis/FeatureType", "Kp/MaxFeatures", "Vis/MaxFeatures", "Vis/CorType",
                 "Vis/MaxDepth", "Vis/InlierDistance", "Kp/NNStrategy", "Rtabmap/LoopThr", "Rtabmap/MemoryThr"]


def team_params(edt):
    """The team's rtabmap_params.yaml as a dict (two stray launch-style lines at its end are skipped)."""
    # Always the comp-day-one branch: main's copy differs (Reg/Strategy 2, Reg/Force3DoF false, Grid/Sensor 1, ...)
    txt = subprocess.check_output(["git", "-C", edt, "show", f"origin/comp-day-one:{TEAM_YAML}"], text=True)
    lines = [ln for ln in txt.splitlines() if not ln.strip().startswith('"')]   # '"tf_delay": 0.0,' etc.
    return yaml.safe_load("\n".join(lines))["rtabmap"]["ros__parameters"]


def db_contents(db):
    """(nodes, words) in a map database, from rtabmap-info's 'LTM: N nodes and W words' line; (0, 0) if unreadable."""
    import re
    try:
        txt = subprocess.run(["/opt/ros/humble/bin/rtabmap-info", db], capture_output=True, text=True, timeout=60).stdout
        m = re.search(r"LTM:\s+(\d+) nodes and (\d+) words", re.sub(r"\x1b\[[0-9;]*m", "", txt))
        return (int(m.group(1)), int(m.group(2))) if m else (0, 0)
    except Exception:          # noqa: BLE001
        return (0, 0)


def write_params(path, base, localization, db):
    p = {k: str(v) for k, v in base.items()}
    p.update({"Rtabmap/DatabaseInMemory": "false", "Grid/Sensor": "1"})
    if localization:
        p.update({"Mem/IncrementalMemory": "false", "Mem/InitWMWithAllNodes": "true"})
    node = {"use_sim_time": True, "rgbd_cameras": 1, "subscribe_depth": False, "subscribe_rgbd": True,
            "subscribe_rgb": False, "subscribe_odom_info": False, "odom_sensor_sync": True,
            "frame_id": "base_link", "map_frame_id": "map", "odom_frame_id": "odom", "publish_tf": True,
            "approx_sync": True, "approx_sync_max_interval": 0.1, "sync_queue_size": 1000, "topic_queue_size": 30,
            "subscribe_scan_cloud": False, "subscribe_scan": False, "database_path": db,
            "qos": 1, "qos_image": 1, "qos_camera_info": 1, "qos_odom": 1,
            "tf_delay": 0.0, "tf_tolerance": 0.3, **p}
    with open(path, "w") as f:
        yaml.safe_dump({"/**": {"ros__parameters": node}}, f)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("dir", nargs="?", default=os.path.join(HERE, "isaac_out", "landmarks_seed1"))
    ap.add_argument("--light", default="both")
    ap.add_argument("--starts", default="10,25", help="revisit start frames (odometry restarts there)")
    ap.add_argument("--passes", default="Arep,Acam32,B32,B32cam,B50")
    ap.add_argument("--edt", default=os.path.expanduser("~/lunabotics/edt-lunabotics-2026"))
    ap.add_argument("--dt", type=float, default=0.5, help="simulated seconds between frames")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from rtabmap_msgs.msg import Info, RGBDImage
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_msgs.msg import TFMessage

    meta = json.load(open(os.path.join(a.dir, "meta.json")))
    W, H = meta["image"]
    f = (W / 2) / np.tan(np.radians(meta["hfov_deg"] / 2))
    cx, cy = (W - 1) / 2, (H - 1) / 2
    lights = ["diffuse", "low"] if a.light == "both" else [a.light]
    starts = [int(s) for s in a.starts.split(",")]
    out = a.out or os.path.join(a.dir, "rtabmap")
    os.makedirs(out, exist_ok=True)
    base = team_params(a.edt)
    sys.path.insert(0, HERE)
    from robot import Robot
    rb = Robot()
    cam_xyz, cam_R = rb.mount("d455_link")
    # optical frame (z forward, x right, y down) in the sandbox camera frame (x forward, y left, z up)
    OPT = np.array([[0, 0, 1.0], [-1, 0, 0], [0, -1, 0]])
    T_base_opt = np.eye(4)
    T_base_opt[:3, :3], T_base_opt[:3, 3] = cam_R @ OPT, np.asarray(cam_xyz) - [0, 0, rb.base_height]

    def quat(R):
        t = np.trace(R)
        if t > 0:
            s = 2 * np.sqrt(1 + t)
            return [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, s / 4]
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2 * np.sqrt(1 + R[i, i] - R[j, j] - R[k, k])
        q = [0.0] * 4
        q[i] = s / 4
        q[j] = (R[j, i] + R[i, j]) / s
        q[k] = (R[k, i] + R[i, k]) / s
        q[3] = (R[k, j] - R[j, k]) / s
        return q

    rclpy.init()
    node = Node("isaac_frame_player", parameter_overrides=[])
    pub_clock = node.create_publisher(Clock, "/clock", 10)
    pub_img = node.create_publisher(Image, "/cam/image", 10)
    pub_depth = node.create_publisher(Image, "/cam/depth", 10)
    pub_info = node.create_publisher(CameraInfo, "/cam/camera_info", 10)
    pub_odom = node.create_publisher(Odometry, "/odom_gt", 10)
    pub_tf = node.create_publisher(TFMessage, "/tf", 10)
    pub_tfs = node.create_publisher(TFMessage, "/tf_static", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    got = {"info": [], "loc": [], "rgbd": 0}
    node.create_subscription(RGBDImage, "/cam/rgbd_image", lambda m: got.__setitem__("rgbd", got["rgbd"] + 1), 10)
    node.create_subscription(Info, "/rtabmap/info", lambda m: got["info"].append(m), 50)
    node.create_subscription(PoseWithCovarianceStamped, "/rtabmap/localization_pose", lambda m: got["loc"].append(m), 50)

    def stamp(t):
        from builtin_interfaces.msg import Time
        return Time(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)))

    def tf_msg(T, parent, child, t):
        m = TransformStamped()
        m.header.stamp, m.header.frame_id, m.child_frame_id = stamp(t), parent, child
        m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = map(float, T[:3, 3])
        q = quat(T[:3, :3])
        m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = map(float, q)
        return m

    def play(pass_name, light, k0, localization, db):
        d = np.load(os.path.join(a.dir, f"{pass_name}_{light}.npz"))
        gray, depth, camT = d["gray"], d["depth"], d["cam_T"]
        Tb = [Tc @ np.linalg.inv(np.r_[np.c_[cam_R, np.asarray(cam_xyz)], [[0, 0, 0, 1]]]) for Tc in camT]  # world-from-robot frame
        for T in Tb:                                   # robot frame (ground) -> base_link (up by base height)
            T[:3, 3] = T[:3, 3] + T[:3, :3] @ [0, 0, rb.base_height]
        T0inv = np.linalg.inv(Tb[k0])
        pfile = os.path.join(out, f"params_{'loc' if localization else 'map'}.yaml")
        write_params(pfile, base, localization, db)
        env = dict(os.environ)
        # executables started directly: "ros2 run" forks the node as a child that survives killing the wrapper
        procs = [subprocess.Popen(["/opt/ros/humble/lib/rtabmap_sync/rgbd_sync", "--ros-args", "-r", "__ns:=/cam",
                                   "-p", "use_sim_time:=true", "-p", "approx_sync:=false",
                                   "-p", "qos:=1", "-p", "qos_camera_info:=1", "-p", "topic_queue_size:=30",
                                   "-p", "sync_queue_size:=30",
                                   "-r", "rgb/image:=/cam/image", "-r", "depth/image:=/cam/depth",
                                   "-r", "rgb/camera_info:=/cam/camera_info", "-r", "rgbd_image:=/cam/rgbd_image"],
                                  env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL),
                 subprocess.Popen(["/opt/ros/humble/lib/rtabmap_slam/rtabmap"] + ([] if localization else ["-d"])
                                  + ["--ros-args", "--params-file", pfile, "-r", "__ns:=/rtabmap",
                                     "-r", "rgbd_image:=/cam/rgbd_image", "-r", "odom:=/odom_gt", "-r", "__node:=rtabmap"],
                                  env=env, stdout=open(os.path.join(out, "rtabmap_last.log"), "w"), stderr=subprocess.STDOUT)]
        import atexit
        atexit.register(lambda: [q.kill() for q in procs if q.poll() is None])
        t_sim = 1.0
        for _ in range(30):                            # let the nodes come up on a ticking clock
            pub_clock.publish(Clock(clock=stamp(t_sim)))
            pub_tfs.publish(TFMessage(transforms=[tf_msg(T_base_opt, "base_link", "cam_optical", t_sim)]))
            rclpy.spin_once(node, timeout_sec=0.1)
        got["info"].clear()
        got["loc"].clear()
        frames = []
        for k in range(k0, len(gray)):
            t_sim += a.dt
            pub_clock.publish(Clock(clock=stamp(t_sim)))
            To = T0inv @ Tb[k]                          # odometry relative to the start frame
            pub_tf.publish(TFMessage(transforms=[tf_msg(To, "odom", "base_link", t_sim)]))
            od = Odometry()
            od.header.stamp, od.header.frame_id, od.child_frame_id = stamp(t_sim), "odom", "base_link"
            od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = map(float, To[:3, 3])
            q = quat(To[:3, :3])
            od.pose.pose.orientation.x, od.pose.pose.orientation.y, od.pose.pose.orientation.z, od.pose.pose.orientation.w = map(float, q)
            od.pose.covariance = [1e-4 if i % 7 == 0 else 0.0 for i in range(36)]
            od.twist.covariance = [1e-4 if i % 7 == 0 else 0.0 for i in range(36)]
            pub_odom.publish(od)
            im = Image(height=H, width=W, encoding="mono8", step=W, data=gray[k].tobytes())
            im.header.stamp, im.header.frame_id = stamp(t_sim), "cam_optical"
            dz = np.nan_to_num(depth[k].astype(np.float32), nan=0.0)
            dm = Image(height=H, width=W, encoding="32FC1", step=4 * W, data=dz.tobytes())
            dm.header = im.header
            ci = CameraInfo(height=H, width=W, distortion_model="plumb_bob", d=[0.0] * 5,
                            k=[f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0], r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                            p=[f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0])
            ci.header = im.header
            n_before = len(got["info"])
            pub_img.publish(im)
            pub_depth.publish(dm)
            pub_info.publish(ci)
            t_end = time.time() + 6.0
            while time.time() < t_end and len(got["info"]) == n_before:
                pub_clock.publish(Clock(clock=stamp(t_sim + 0.01)))
                pub_tf.publish(TFMessage(transforms=[tf_msg(To, "odom", "base_link", t_sim + 0.01)]))
                rclpy.spin_once(node, timeout_sec=0.05)
            info = got["info"][-1] if len(got["info"]) > n_before else None
            # the localization pose of this frame carries this frame's stamp and can arrive after the info message
            want = stamp(t_sim)
            loc_now = None
            t_end = time.time() + (1.0 if localization and info is not None and
                                   (info.loop_closure_id > 0 or info.proximity_detection_id > 0) else 0.0)
            while True:
                for m in reversed(got["loc"]):
                    if m.header.stamp.sec == want.sec and m.header.stamp.nanosec == want.nanosec:
                        loc_now = m
                        break
                if loc_now is not None or time.time() >= t_end:
                    break
                rclpy.spin_once(node, timeout_sec=0.02)
            stats = dict(zip(info.stats_keys, info.stats_values)) if info is not None else {}
            rec = {"k": k, "processed": info is not None,
                   "loop_id": int(info.loop_closure_id) if info else 0,
                   "proximity_id": int(info.proximity_detection_id) if info else 0,
                   "inliers": float(stats.get("Loop/Visual_inliers/", stats.get("Loop/Inliers/", 0.0))),
                   "words": float(stats.get("Keypoint/Current_frame/words", 0.0)), "rgbd_msgs": got["rgbd"]}
            if localization and loc_now is not None:
                L = loc_now.pose.pose
                p = np.array([L.position.x, L.position.y, L.position.z])
                rec["loc_xyz"] = p.tolist()
                rec["truth_xyz_world"] = Tb[k][:3, 3].tolist()
            frames.append(rec)
        for pr_ in procs:
            pr_.send_signal(signal.SIGINT)
        for pr_ in procs:
            try:
                pr_.wait(timeout=15)
            except subprocess.TimeoutExpired:
                pr_.kill()
        return frames, [np.asarray(T).tolist() for T in Tb]

    results = {"team_params_file": TEAM_YAML + " (comp-day-one)", "team_params": base, "overrides": OVERRIDES,
               "left_at_default": TEAM_DEFAULTS, "starts": starts, "sessions": {}}
    for light in lights:
        db = os.path.join(out, f"A_{light}.db")
        if os.path.exists(db):
            os.remove(db)
        fr, TbA = play("A", light, 0, False, db)
        results["sessions"][f"map_A_{light}"] = {"frames": fr, "processed": sum(x["processed"] for x in fr)}
        print(f"[rtabmap] map A {light}: {sum(x['processed'] for x in fr)}/{len(fr)} frames processed, "
              f"{sum(x['loop_id'] > 0 for x in fr)} loop closures", flush=True)
        nodes, words = db_contents(db)
        results["sessions"][f"map_A_{light}"].update(db_nodes=nodes, db_words=words)
        print(f"[rtabmap] {db}: {nodes} nodes, {words} words", flush=True)
        if nodes == 0 or words == 0:
            # rtabmap aborts (SIGABRT, Ubuntu crash dialog) when it loads a map without descriptors: never start it
            results["sessions"][f"map_A_{light}"]["error"] = "empty map database: localization not started"
            print(f"[rtabmap] map for {light} is empty; skipping its revisits", flush=True)
            continue
        # map frame = robot pose of pass A's frame 0 (RGBD/StartAtOrigin true)
        T_world_map = np.array(TbA[0])
        for pname in [p_ for p_ in a.passes.split(",") if p_]:
            for k0 in starts:
                dbl = os.path.join(out, f"loc_{pname}_{light}_{k0}.db")
                shutil.copy(db, dbl)
                fr, TbB = play(pname, light, k0, True, dbl)
                ok = [x for x in fr if x["loop_id"] > 0 or x["proximity_id"] > 0]
                first = ok[0]["k"] - k0 if ok else None
                errs = []
                for x in fr:
                    if "loc_xyz" in x:
                        truth_map = np.linalg.inv(T_world_map) @ np.r_[x["truth_xyz_world"], 1.0]
                        dxyz = np.array(x["loc_xyz"]) - truth_map[:3]
                        x["loc_err_m"] = float(np.linalg.norm(dxyz[:2]))       # planar: team uses Reg/Force3DoF true
                        x["loc_err_3d_m"] = float(np.linalg.norm(dxyz))
                        errs.append(x["loc_err_m"])
                key = f"{pname}_{light}_start{k0}"
                wrong = [x for x in ok if x.get("loc_err_m", 0.0) > WRONG_M]
                results["sessions"][key] = {"frames": fr, "n": len(fr), "relocalized_frames": len(ok),
                                            "first_reloc_frame_after_start": first,
                                            "mean_inliers_when_ok": float(np.mean([x["inliers"] for x in ok])) if ok else 0.0,
                                            "loc_err_median_m": float(np.median(errs)) if errs else None,
                                            "loc_err_p90_m": float(np.percentile(errs, 90)) if errs else None,
                                            "wrong_relocalizations": len(wrong),
                                            "wrong_frames": [x["k"] for x in wrong]}
                print(f"[rtabmap] {key}: relocalized in {len(ok)}/{len(fr)} frames, first after {first} frames, "
                      f"median error {results['sessions'][key]['loc_err_median_m']}", flush=True)
                os.remove(dbl)
    with open(os.path.join(out, "rtabmap_reloc.json"), "w") as fh:
        json.dump(results, fh, indent=1, default=float)
    L = ["# RTAB-Map relocalization before and after wheel ruts (team settings, Isaac images)", "",
         f"Map: pass A (no ruts). Revisits start at frames {starts} with odometry restarted there (ground-truth odometry, "
         "so only place recognition is tested). Team parameters from " + TEAM_YAML + " (comp-day-one); overrides and "
         "defaults listed in rtabmap_reloc.json.", "",
         f"Pose error is planar (x, y; the team runs Reg/Force3DoF true). Wrong relocalization: an accepted loop closure "
         f"whose localization error exceeds {WRONG_M} m.", "",
         "| light | revisit pass | start | frames relocalized | first relocalization (frames after start) | mean inliers | "
         "pose error median / p90 (m) | wrong relocalizations |",
         "| --- | --- | --- | --- | --- | --- | --- | --- |"]
    for key, s in results["sessions"].items():
        if key.startswith("map_"):
            continue
        pname, light, st = key.split("_")[0], key.split("_")[1], key.split("start")[1]
        e = "-" if s["loc_err_median_m"] is None else f"{s['loc_err_median_m']:.3f} / {s['loc_err_p90_m']:.3f}"
        L.append(f"| {light} | {pname} | {st} | {s['relocalized_frames']}/{s['n']} | {s['first_reloc_frame_after_start']} | "
                 f"{s['mean_inliers_when_ok']:.0f} | {e} | {s['wrong_relocalizations']} |")
    open(os.path.join(out, "rtabmap_reloc.md"), "w").write("\n".join(L) + "\n")
    print("\n".join(L))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
