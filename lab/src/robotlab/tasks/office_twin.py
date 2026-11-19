"""Office twin (2026-11-09): the real office (car #1's saved map) as the sim scene, with the floor route and the charger
start, for an exam that must rank models as the floor does (floor goto17/22: RL v6p full trip ~65 s; goto20 student
v10 + skill stuck at the door; goto21 v6p + steering smoothing not back in 120 s).

Enabled by env ROBOTLAB_TWIN=<npz from lab/scripts/make_twin.py> (goto task only):
  - terrain: one box per merged rectangle of occupied cells (walls, desk / cabinet legs the LiDAR saw, unknown space);
  - layout: the twin's occupancy (rewards, maps, teacher) and a dummy 4x4 hallway graph so the random-route code keeps
    working; reset_explore / reset_goto then put the car at the charger with the floor route (TWIN hooks);
  - no people (the floor runs had none on the route).
Known gaps: things only the depth camera sees (door stopper, chair bases) are not in the map.
"""
import os
from dataclasses import dataclass

import mujoco
import numpy as np
import torch
from mjlab.terrains.terrain_generator import SubTerrainCfg, TerrainGeometry, TerrainOutput

from robotlab.tasks import office_terrain as OT

TWIN_PATH = os.environ.get("ROBOTLAB_TWIN", "")
_DATA = None


def data() -> dict:
  global _DATA
  if _DATA is None:
    d = np.load(TWIN_PATH)
    _DATA = {k: d[k] for k in d.files}
  return _DATA


def _rects(occ: np.ndarray) -> list[tuple[int, int, int, int]]:
  """Greedy cover of the occupied cells by rectangles (ix0, ix1, iy0, iy1), inclusive."""
  todo = occ.copy()
  out = []
  n_x, n_y = occ.shape
  for i in range(n_x):
    j = 0
    while j < n_y:
      if not todo[i, j]:
        j += 1
        continue
      j1 = j
      while j1 + 1 < n_y and todo[i, j1 + 1]:
        j1 += 1
      i1 = i
      while i1 + 1 < n_x and todo[i1 + 1, j:j1 + 1].all():
        i1 += 1
      todo[i:i1 + 1, j:j1 + 1] = False
      out.append((i, i1, j, j1))
      j = j1 + 1
  return out


@dataclass(kw_only=True)
class OfficeTwinTerrainCfg(SubTerrainCfg):
  def function(self, difficulty: float, spec: mujoco.MjSpec, rng: np.random.Generator) -> TerrainOutput:
    body = spec.body("terrain")
    geoms: list[TerrainGeometry] = []
    if not any(g.name == "ground" for g in body.geoms):
      body.add_geom(name="ground", type=mujoco.mjtGeom.mjGEOM_PLANE, size=(0, 0, 1), rgba=(0, 0, 0, 0))
    floor = body.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=(OT.PATCH_SIZE / 2, OT.PATCH_SIZE / 2, 0.05),
                          pos=(OT.PATCH_SIZE / 2, OT.PATCH_SIZE / 2, -0.05), rgba=(0.62, 0.62, 0.60, 1.0))
    floor.contype, floor.conaffinity = 0, 0
    geoms.append(TerrainGeometry(geom=floor, color=(0.62, 0.62, 0.60, 1.0)))
    occ = data()["occ"].astype(bool)
    r = OT.OCC_RES
    rgba = (0.85, 0.85, 0.88, 1.0)
    for i0, i1, j0, j1 in _rects(occ):
      x0, x1, y0, y1 = i0 * r, (i1 + 1) * r, j0 * r, (j1 + 1) * r
      g = body.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=((x1 - x0) / 2, (y1 - y0) / 2, OT.WALL_HEIGHT / 2),
                        pos=((x0 + x1) / 2, (y0 + y1) / 2, OT.WALL_HEIGHT / 2), rgba=rgba)
      g.contype, g.conaffinity = 2, 2
      geoms.append(TerrainGeometry(geom=g, color=rgba))
    # dummy hallway graph (the full 4x4 grid) so the random-route / walker code keeps working; the TWIN hooks override
    n = OT.NODES * OT.NODES
    node_xy = np.array([[OT._NODE_C[a // OT.NODES], OT._NODE_C[a % OT.NODES]] for a in range(n)])
    nbr = np.full((n, 4), -1, np.int64)
    for a in range(n):
      ix, iy = divmod(a, OT.NODES)
      for d, (dx, dy) in enumerate(OT._DIRS):
        if 0 <= ix + dx < OT.NODES and 0 <= iy + dy < OT.NODES:
          nbr[a, d] = OT._node(ix + dx, iy + dy)
    route = np.zeros((n, 4, 2, 2))
    for a in range(n):
      for d in range(4):
        if nbr[a, d] >= 0:
          route[a, d] = (node_xy[a] * 0.65 + node_xy[nbr[a, d]] * 0.35, node_xy[a] * 0.35 + node_xy[nbr[a, d]] * 0.65)
    OT.LAYOUTS.append(OT.OfficeLayout(node_xy, nbr, route, occ, occ.copy(), np.zeros(occ.shape, np.int16), 1.4, ()))
    return TerrainOutput(origin=np.zeros(3), geometries=geoms)


def start_pose(n: int, device) -> tuple[torch.Tensor, torch.Tensor]:
  """The charger start, +-0.05 m and +-10 deg (the floor runs' route alignments were -4..-15 deg: ESTIMATE range)."""
  s = data()["start"]
  xy = torch.tensor(s[:2], dtype=torch.float32, device=device).repeat(n, 1)
  xy += 0.05 * (2 * torch.rand(n, 2, device=device) - 1)
  yaw = float(s[2]) + torch.deg2rad(torch.tensor(10.0, device=device)) * (2 * torch.rand(n, device=device) - 1)
  return xy, yaw


def route_points(device) -> list[torch.Tensor]:
  return [torch.tensor(p, dtype=torch.float32, device=device) for p in data()["route"]]
