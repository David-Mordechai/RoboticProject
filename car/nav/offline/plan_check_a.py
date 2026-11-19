"""Offline Phase A planner check (2026-11-09): Nav2 Humble Smac Hybrid-A* with car/nav/nav2_params_a.yaml on a saved
car map, no car needed. Plans the floor trip's legs and measures each path's clearance to the map's occupied cells.

  (in robotlab-nav2:humble) python3 plan_check_a.py MAP.npz PARAMS.yaml

Legs (map frame of goto6_map_after.npz, the frame of the floor route): charger (0, 0) -> door -> hall end (U-turn) ->
charger. Goals are places only: the planner chooses the way (no hand-typed waypoints).
"""
import math
import os
import subprocess
import sys
import tempfile
import time

import numpy as np
import yaml

MAP, PARAMS = sys.argv[1], sys.argv[2]
ONLY_FIRST = len(sys.argv) > 3 and sys.argv[3] == "--first"   # leg 1 only, goal door (x, y) from argv[4:6]
# goals face the way the car drives there; the U-turn is the planner's job on the way back (a goal facing back made
# it turn mid-hall and reverse 2 m to the hall end, 2026-11-09)
LEGS = [("undocked (0.3 m out) -> door", (0.3, 0.0, 0.0), (2.95, -2.3, -math.pi / 2)),   # the dock is in the map: undock straight first
        ("door -> hall end", (2.95, -2.3, -math.pi / 2), (2.95, -7.0, -math.pi / 2)),
        ("hall end -> charger (U-turn)", (2.95, -7.0, -math.pi / 2), (0.0, 0.0, math.pi))]

if ONLY_FIRST:
  LEGS = [("undocked (0.3 m out) -> door", (0.3, 0.0, 0.0), (float(sys.argv[4]), float(sys.argv[5]), -math.pi / 2))]
d = np.load(MAP)
g, res, org = d["grid"], float(d["res"]), d["origin"]
tmp = tempfile.mkdtemp()
img = np.full(g.shape, 205, np.uint8)
img[g == 0] = 254
img[g == 100] = 0
with open(os.path.join(tmp, "map.pgm"), "wb") as f:      # map_server: row 0 = top (max y)
  f.write(f"P5\n{g.shape[1]} {g.shape[0]}\n255\n".encode())
  f.write(img[::-1].tobytes())
yaml.safe_dump({"image": "map.pgm", "mode": "trinary", "resolution": res, "origin": [float(org[0]), float(org[1]), 0.0],
                "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.25}, open(os.path.join(tmp, "map.yaml"), "w"))
P = yaml.safe_load(open(PARAMS))
gc = P["global_costmap"]["global_costmap"]["ros__parameters"]
gc["plugins"] = ["static_layer", "inflation_layer"]           # offline: the saved map only (no live scans)
gc.pop("obstacle_layer", None)
P["map_server"] = {"ros__parameters": {"yaml_filename": os.path.join(tmp, "map.yaml")}}
P["lifecycle_manager_offline"] = {"ros__parameters": {"autostart": True, "bond_timeout": 0.0,
                                                      "node_names": ["map_server", "planner_server"]}}
pfile = os.path.join(tmp, "params.yaml")
yaml.safe_dump(P, open(pfile, "w"))
env = dict(os.environ)
procs = [
  subprocess.Popen(["ros2", "run", "tf2_ros", "static_transform_publisher", "--frame-id", "map", "--child-frame-id",
                    "base_footprint"], env=env),
  subprocess.Popen(["ros2", "run", "nav2_map_server", "map_server", "--ros-args", "--params-file", pfile], env=env),
  subprocess.Popen(["ros2", "run", "nav2_planner", "planner_server", "--ros-args", "--params-file", pfile], env=env),
  subprocess.Popen(["ros2", "run", "nav2_lifecycle_manager", "lifecycle_manager", "--ros-args", "-r",
                    "__node:=lifecycle_manager_offline", "--params-file", pfile], env=env)]
try:
  import rclpy
  from geometry_msgs.msg import PoseStamped
  from nav2_msgs.action import ComputePathToPose
  from rclpy.action import ActionClient
  rclpy.init()
  node = rclpy.create_node("plan_check_a")
  cli = ActionClient(node, ComputePathToPose, "compute_path_to_pose")
  assert cli.wait_for_server(timeout_sec=60.0), "planner_server not up"
  time.sleep(5.0)                                               # costmap from the map
  occ = np.argwhere(g != 0)
  ox, oy = org[0] + (occ[:, 1] + 0.5) * res, org[1] + (occ[:, 0] + 0.5) * res

  def pose(x, y, yaw):
    p = PoseStamped(); p.header.frame_id = "map"
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p

  for name, s, gl in LEGS:
    goal = ComputePathToPose.Goal(); goal.goal = pose(*gl); goal.start = pose(*s); goal.use_start = True
    goal.planner_id = "GridBased"
    fut = cli.send_goal_async(goal); rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
    res_f = fut.result().get_result_async(); rclpy.spin_until_future_complete(node, res_f, timeout_sec=60)
    path = res_f.result().result.path.poses
    if not path:
      print(f"{name}: NO PATH (status {res_f.result().status})"); continue
    xy = np.array([[q.pose.position.x, q.pose.position.y] for q in path])
    clear = np.array([np.min(np.hypot(ox - x, oy - y)) for x, y in xy])
    L = float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))
    rev = 0
    for i in range(1, len(path) - 1):     # direction changes: heading vs motion direction flips sign
      yaw = 2 * math.atan2(path[i].pose.orientation.z, path[i].pose.orientation.w)
      mv = xy[i + 1] - xy[i]
      rev += int(np.dot(mv, [math.cos(yaw), math.sin(yaw)]) < 0)
    print(f"{name}: {len(xy)} poses, {L:.2f} m, min clearance to map obstacles {clear.min():.2f} m "
          f"(5th pct {np.percentile(clear, 5):.2f}), reverse poses {rev}")
    print("   path every 0.25 m:", " ".join(f"({x:.2f},{y:.2f})" for x, y in xy[::5]))
    np.save(os.path.join(os.path.dirname(MAP) or ".", f"path_{name.split()[0]}_{name.split()[-1].strip('()')}.npy"), xy)
finally:
  for pr in procs:
    pr.terminate()
