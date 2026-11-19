"""Sensor-use gate before a floor test (2026-11-09; floor goto18/19 failed because the student ignored its sensors).

Replays observations recorded on the car (explore_policy.py --log) through an exported goto policy, and measures how
much the commanded steering changes when one sensor group is blanked (set to its "nothing seen" value). A policy that
avoids obstacles must react: blanking the LiDAR / depth / maps must change its steering where obstacles are near.

  python lab/scripts/probe_sensors.py --onnx M.onnx [M2.onnx ...] --npz goto19.npz [--near 0.6]

Prints, per model: mean |steer change| (steer in [-1, 1], clipped) per blanked group, over the steps where an obstacle
was within --near m (LiDAR or depth). Gate (our choice, not from a source): the sum over sensor groups >= 0.6; on floor goto19 inputs the old RL model v6p 8750 scored 1.52, the failed student_v6 model_40 0.30.
Groups come from the model's obs_layout metadata.
"""
import argparse

import numpy as np
import onnxruntime as ort

p = argparse.ArgumentParser()
p.add_argument("--onnx", nargs="+", required=True)
p.add_argument("--npz", nargs="+", required=True)
p.add_argument("--near", type=float, default=0.6, help="m: only steps with an obstacle this close (LiDAR or depth)")
p.add_argument("--every", type=int, default=3, help="use every k-th step")
a = p.parse_args()

# "Nothing seen" values per group: free LiDAR / depth sectors read 1 (normalised range), maps 0 (free / unknown).
BLANK = {"lidar_sectors": 1.0, "depth": 1.0, "map_fine": 0.0, "map_coarse": 0.0, "lidar_hist": 1.0}
SENSORS = ("lidar_sectors", "depth", "map_fine", "map_coarse", "lidar_hist")


def layout(meta):
  out = {}
  for part in meta["obs_layout"].split(";"):
    name, rest = part.split(":")
    n, at = rest.split("@")
    out[name] = (int(at), int(at) + int(n))
  return out


obs = []
for f in a.npz:
  d = np.load(f)
  obs.append(d["obs"][:: a.every].astype(np.float32))
obs = np.concatenate(obs)

for m in a.onnx:
  s = ort.InferenceSession(m)
  name, dim = s.get_inputs()[0].name, s.get_inputs()[0].shape[1]
  L = layout(s.get_modelmeta().custom_metadata_map)
  x = obs[:, :dim]
  lid = L["lidar_sectors"]
  # LiDAR sectors are range / LIDAR_RANGE (car/policy/explore_obs.py: 3.0 m). Front half only (sectors 0..8 and 27..35
  # = +-90 deg, sector 0 straight ahead, explore_obs.lidar_sectors), so the charger behind the car does not count.
  front = np.r_[0:9, 27:36] + lid[0]
  near = x[:, front].min(1) * 3.0 < a.near
  x = x[near]
  run = lambda z: np.clip(np.concatenate([s.run(None, {name: z[i:i + 1]})[0] for i in range(len(z))]), -1, 1)
  base = run(x)
  print(f"{m.split('/')[-1].split(chr(92))[-1]}: {len(x)} steps with an obstacle within {a.near} m")
  tot = 0.0
  for g in SENSORS:
    if g not in L:
      continue
    y = x.copy()
    y[:, L[g][0]:L[g][1]] = BLANK[g]
    dsteer = np.abs(run(y)[:, 1] - base[:, 1]).mean()
    tot += dsteer
    print(f"  blank {g:14s} mean |steer change| {dsteer:.3f}")
  print(f"  total {tot:.3f} -> {'PASS' if tot >= 0.6 else 'FAIL'} (gate >= 0.6)")
