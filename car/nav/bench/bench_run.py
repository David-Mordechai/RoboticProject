"""Nav2 test bench run (ASUS, image robotlab-nav2:humble): Phase A Nav2 + fake_car.py on a saved car map, one goal,
then the same numbers we measured on the floor (car/maps/phase_a_runs/*.py), so a settings change is judged off the car.

  python3 bench_run.py MAP.npz PARAMS.yaml BT.xml START_X START_Y START_YAW GOAL_X GOAL_Y GOAL_YAW [--set k.k.k=v ...]
         [--steer-max 33] [--timeout 120] [--out result.json]

--set edits the params file before the run, e.g. --set planner_server.GridBased.reverse_penalty=4.0
(the first key is the node; "ros__parameters" is inserted). Results (JSON): reached, time_s, contacts, recoveries,
reverse segments / seconds, driver-level steering flips per s and % at full lock (ackermann.py: atan(0.145 w / v)),
mean |v| while moving, path length driven.
Localization: none (map->odom identity, as fake_car.py publishes); AMCL is not part of the test.
"""
import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
import time

import numpy as np
import yaml

ap = argparse.ArgumentParser()
for k in ("map", "params", "bt"): ap.add_argument(k)
for k in ("sx", "sy", "syaw", "gx", "gy", "gyaw"): ap.add_argument(k, type=float)
ap.add_argument("--set", action="append", default=[])
ap.add_argument("--steer-max", type=float, default=33.0)
ap.add_argument("--steer-max-right", type=float, default=None)   # asymmetric car (car #1: 28 L / 24 R measured)
ap.add_argument("--timeout", type=float, default=120.0)
ap.add_argument("--out", default="")
ap.add_argument("--logdir", default="bench_logs")
a = ap.parse_args()
HERE = os.path.dirname(os.path.abspath(__file__))
os.makedirs(a.logdir, exist_ok=True)
tmp = tempfile.mkdtemp(dir=a.logdir, prefix=time.strftime("%H%M%S_"))   # params, map, nav2.log kept

# map for map_server (same conversion as car/nav/npz_to_pgm.py)
d = np.load(a.map); g, res, org = d["grid"], float(d["res"]), d["origin"]
img = np.full(g.shape, 205, np.uint8); img[g == 0] = 254; img[g == 100] = 0
with open(os.path.join(tmp, "map.pgm"), "wb") as f:
  f.write(f"P5\n{g.shape[1]} {g.shape[0]}\n255\n".encode()); f.write(img[::-1].tobytes())
yaml.safe_dump({"image": "map.pgm", "mode": "trinary", "resolution": res, "origin": [float(org[0]), float(org[1]), 0.0],
                "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.25}, open(os.path.join(tmp, "map.yaml"), "w"))

P = yaml.safe_load(open(a.params))
for node in ("amcl", "lifecycle_manager_localization"):
  P.pop(node, None)
P["map_server"] = {"ros__parameters": {"yaml_filename": os.path.join(tmp, "map.yaml")}}
P["bt_navigator"]["ros__parameters"]["default_nav_to_pose_bt_xml"] = os.path.abspath(a.bt)
P["bt_navigator"]["ros__parameters"]["default_nav_through_poses_bt_xml"] = os.path.abspath(a.bt)
for oc in ("local_costmap", "global_costmap"):   # no depth camera on the bench (the floor runs that pass used LiDAR only)
  ol = P[oc][oc]["ros__parameters"]["obstacle_layer"]; ol["observation_sources"] = "scan"; ol.pop("depth_scan", None)
cm = P["collision_monitor"]["ros__parameters"]; cm["observation_sources"] = ["scan"]; cm.pop("depth_scan", None)
for s in a.set:
  k, v = s.split("=", 1); ks = k.split(".")
  node = P[ks[0]]["ros__parameters"] if ks[0] not in ("local_costmap", "global_costmap") else P[ks[0]][ks[0]]["ros__parameters"]
  for kk in ks[1:-1]: node = node.setdefault(kk, {})
  node[ks[-1]] = yaml.safe_load(v)
P["lifecycle_manager_bench"] = {"ros__parameters": {"autostart": True, "bond_timeout": 0.0, "node_names": [
  "map_server", "controller_server", "planner_server", "behavior_server", "bt_navigator", "velocity_smoother",
  "collision_monitor"]}}


class NoAlias(yaml.SafeDumper):
  def ignore_aliases(self, data): return True


pfile = os.path.join(tmp, "params.yaml"); yaml.dump(P, open(pfile, "w"), Dumper=NoAlias)
R = ["--ros-args", "--params-file", pfile]
nodes = [("nav2_map_server", "map_server", []), ("nav2_controller", "controller_server", ["-r", "cmd_vel:=cmd_vel_nav"]),
         ("nav2_planner", "planner_server", []), ("nav2_behaviors", "behavior_server", ["-r", "cmd_vel:=cmd_vel_nav"]),
         ("nav2_bt_navigator", "bt_navigator", []),
         ("nav2_velocity_smoother", "velocity_smoother", ["-r", "cmd_vel:=cmd_vel_nav"]),
         ("nav2_collision_monitor", "collision_monitor", [])]
log = open(os.path.join(tmp, "nav2.log"), "w")
procs = [subprocess.Popen([sys.executable, os.path.join(HERE, "fake_car.py"), a.map, str(a.sx), str(a.sy), str(a.syaw),
                           "--steer-max", str(a.steer_max)] + ([] if a.steer_max_right is None else ["--steer-max-right", str(a.steer_max_right)]), stdout=log, stderr=log)]
time.sleep(2)
for pkg, exe, rm in nodes:
  procs.append(subprocess.Popen(["ros2", "run", pkg, exe] + R + rm, stdout=log, stderr=log))
procs.append(subprocess.Popen(["ros2", "run", "nav2_lifecycle_manager", "lifecycle_manager", "--ros-args", "-r",
                               "__node:=lifecycle_manager_bench", "--params-file", pfile], stdout=log, stderr=log))
out = {"reached": False}
try:
  import rclpy
  from geometry_msgs.msg import PoseStamped, Twist
  from nav2_msgs.action import NavigateToPose
  from nav_msgs.msg import Odometry, Path
  from rclpy.action import ActionClient
  from std_msgs.msg import Int32
  rclpy.init(); n = rclpy.create_node("bench_run")
  rec = {"nav": [], "mot": [], "odom": [], "contacts": 0, "plans": 0}
  n.create_subscription(Twist, "/cmd_vel_nav", lambda m: rec["nav"].append((time.time(), m.linear.x, m.angular.z)), 50)
  n.create_subscription(Twist, "/controller/cmd_vel", lambda m: rec["mot"].append((time.time(), m.linear.x, m.angular.z)), 50)
  n.create_subscription(Odometry, "/odom", lambda m: rec["odom"].append((time.time(), m.pose.pose.position.x, m.pose.pose.position.y)), 50)
  n.create_subscription(Int32, "/bench/contacts", lambda m: rec.__setitem__("contacts", m.data), 10)
  n.create_subscription(Path, "/plan", lambda m: rec.__setitem__("plans", rec["plans"] + 1), 10)
  cli = ActionClient(n, NavigateToPose, "navigate_to_pose")
  assert cli.wait_for_server(timeout_sec=90), "bt_navigator not up"
  time.sleep(4)
  goal = NavigateToPose.Goal(); p = PoseStamped(); p.header.frame_id = "map"
  p.pose.position.x, p.pose.position.y = a.gx, a.gy
  p.pose.orientation.z, p.pose.orientation.w = math.sin(math.radians(a.gyaw) / 2), math.cos(math.radians(a.gyaw) / 2)
  goal.pose = p
  t0 = time.time(); fb = {"rec": 0}
  fut = cli.send_goal_async(goal, feedback_callback=lambda f: fb.__setitem__("rec", f.feedback.number_of_recoveries))
  rclpy.spin_until_future_complete(n, fut, timeout_sec=10); gh = fut.result(); rf = gh.get_result_async()
  while not rf.done() and time.time() - t0 < a.timeout:
    rclpy.spin_once(n, timeout_sec=0.05)
  out["time_s"] = round(time.time() - t0, 1)
  if rf.done():
    out["reached"] = rf.result().status == 4          # GoalStatus.STATUS_SUCCEEDED
  else:
    gh.cancel_goal_async(); rclpy.spin_once(n, timeout_sec=0.5)
  out["recoveries"] = fb["rec"]; out["contacts"] = rec["contacts"]; out["replans"] = rec["plans"]
  # reverse segments (controller wish) and driver-level steering
  nav = np.array(rec["nav"]) if rec["nav"] else np.zeros((0, 3))
  segs, cur = [], None
  for t, v, w in nav:
    if v < -1e-3 and cur is None: cur = t
    if v >= -1e-3 and cur is not None: segs.append(t - cur); cur = None
  out["reverse_segments"] = len(segs); out["reverse_s"] = round(sum(segs), 1)
  mot = np.array(rec["mot"]) if rec["mot"] else np.zeros((0, 3)); mv = np.abs(mot[:, 1]) > 1e-3
  if mv.any():
    dl = np.degrees(np.clip(np.arctan(0.145 * mot[mv, 2] / mot[mv, 1]), -math.radians(40), math.radians(40)))
    sg = np.where(np.abs(dl) < 3, 0, np.sign(dl)); sg = sg[sg != 0]
    dur = max(mot[mv, 0][-1] - mot[mv, 0][0], 1e-3)
    out["flips_per_s"] = round(float((np.diff(sg) != 0).sum() / dur), 2)
    out["full_lock_pct"] = round(float(np.mean(np.abs(dl) >= 39) * 100), 1)
    out["mean_speed"] = round(float(np.mean(np.abs(mot[mv, 1]))), 3)
  od = np.array(rec["odom"])
  if len(od) > 1:
    out["driven_m"] = round(float(np.sum(np.linalg.norm(np.diff(od[:, 1:], axis=0), axis=1))), 2)
    out["end"] = [round(float(od[-1, 1]), 2), round(float(od[-1, 2]), 2)]
finally:
  for pr in procs: pr.terminate()
  time.sleep(1)
  for pr in procs: pr.kill()
out["set"] = a.set; out["bt"] = os.path.basename(a.bt); out["steer_max"] = a.steer_max; out["steer_max_right"] = a.steer_max_right
print(json.dumps(out))
if a.out: json.dump(out, open(a.out, "w"))
