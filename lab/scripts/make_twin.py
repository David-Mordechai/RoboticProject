"""Build the office twin (2026-11-09): the real office as a sim scene, for an exam that must rank models as the floor does.

Input: a map saved on the car (car/measure/map_dump.py npz: grid int8 [rows=y, cols=x], 100 occupied / 0 free / -1
unknown, res, origin = map-frame xy of cell (0, 0)), the floor route (map frame) and the start pose (the charger).
Output npz, patch frame (= map frame - origin, so the map's corner is at 0, 0):
  occ [OCC_N, OCC_N] bool, index [ix, iy] (office_terrain convention): occupied or unknown, and everything off the map;
  start (x, y, yaw); route [K, 2] (start first); turn_backs [2, 2] (the U-turn corners at the hall end).

  python lab/scripts/make_twin.py car/maps/office_goto/goto6_map_after.npz lab/src/robotlab/tasks/twin_office.npz

Unknown cells count as walls: the car never saw past them, so the twin is closed. Thin and low things the LiDAR saw
(desk legs at the charger, cabinet legs at the door) are in the map as occupied cells. What only the depth camera sees
(the blue door stopper, chair bases) is NOT in this map: a known gap of the twin.
"""
import sys

import numpy as np

OCC_RES, OCC_N = 0.05, 288          # = office_terrain.OCC_RES / OCC_N (14.4 m patch)
# Floor route of goto17-22 (car/measure go17.sh, map frame of goto6_map_after.npz) and the charger start.
ROUTE = [1.2, -0.3, 2.0, -0.6, 2.45, -1.1, 2.45, -1.9, 2.45, -3.0, 2.4, -6.5, 2.4, -7.4, 2.8, -7.4, 2.8, -6.5, 2.6, -3.0,
         2.55, -1.9, 2.55, -1.1, 2.0, -0.6, 1.2, -0.3, 0.0, 0.0]
START = (0.0, 0.0, 0.0)
TURN_BACKS = [(2.4, -7.4), (2.8, -7.4)]

# Corrected route (2026-11-09, twin check): the floor route's first waypoint (1.2, -0.3) lies on the desk leg (0.03 m
# clearance in the map), the door waypoints (x 2.45-2.55, y -1.9) on the left door frame (opening x 2.55-3.4), and the
# hall lanes (x 2.4 / 2.6) 0.1-0.3 m from the left wall (hall x 2.2-3.9). Waypoints re-placed 0.35-0.6 m from obstacles.
ROUTE_FIX = [1.4, -0.7, 2.1, -0.8, 2.75, -1.2, 2.95, -1.9, 2.8, -2.8, 2.75, -6.5, 2.75, -7.3, 3.15, -7.3, 3.15, -6.5,
             3.1, -2.8, 3.0, -1.9, 2.8, -1.2, 2.1, -0.8, 1.4, -0.7, 0.0, 0.0]
TURN_BACKS_FIX = [(2.75, -7.3), (3.15, -7.3)]
src, out = sys.argv[1], sys.argv[2]
if len(sys.argv) > 3 and sys.argv[3] == "--fix":
  ROUTE, TURN_BACKS = ROUTE_FIX, TURN_BACKS_FIX
d = np.load(src)
g, res, org = d["grid"], float(d["res"]), d["origin"]
assert abs(res - OCC_RES) < 1e-6, res
rows, cols = g.shape
occ = np.ones((OCC_N, OCC_N), bool)                       # off the map: wall
cx, cy = min(cols, OCC_N), min(rows, OCC_N)
occ[:cx, :cy] = (g[:cy, :cx] != 0).T                     # [ix = col, iy = row]
to_patch = lambda p: np.asarray(p, float) - org
start = np.array([*to_patch(START[:2]), START[2]])
# the charger dock the car sits on at the start is in the map 0.05-0.2 m behind / beside base_footprint: in the sim the
# car would spawn touching it (twin v1: 59% "bumps" at the start). Clear CHARGER_CLEAR around the start (ESTIMATE).
CHARGER_CLEAR = 0.22
cxs = (np.arange(OCC_N) + 0.5) * OCC_RES
occ[(cxs[:, None] - start[0]) ** 2 + (cxs[None, :] - start[1]) ** 2 < CHARGER_CLEAR ** 2] = False
route = np.array([to_patch(START[:2])] + [to_patch(p) for p in zip(ROUTE[::2], ROUTE[1::2])])
tb = np.array([to_patch(p) for p in TURN_BACKS])
ix, iy = (start[:2] / OCC_RES).astype(int)
print(f"map {rows}x{cols} cells, origin {org}; start patch ({start[0]:.2f}, {start[1]:.2f}); occupied around start "
      f"(0.15 m): {int(occ[ix - 3:ix + 4, iy - 3:iy + 4].sum())} cells; occupied share {occ.mean():.2f}")
for p in route:
  i, j = (p / OCC_RES).astype(int)
  if occ[i, j]:
    print(f"warning: route point ({p[0]:.2f}, {p[1]:.2f}) is on an occupied cell")
np.savez(out, occ=occ, start=start, route=route, turn_backs=tb)
print(f"wrote {out}: {len(route)} route points")
