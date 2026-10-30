"""Procedural office floors: a grid of hallways between closed rooms, with clutter in the halls.

Each terrain patch is one office floor. Hallway intersections ("nodes") sit on a 4 x 4 grid; the
hallway segments between them form a random spanning tree plus extra loops, with no dead ends (the
scripted leader never has to U-turn). Everything that is not hallway is solid room block.

Obstacles (boxes, bins, plants, chairs) stand against one hallway wall and leave a free lane on the
other side. For each segment the generator also writes the route the leader takes through it, so the
leader drives around obstacles instead of through them.

Collision groups match the car model: floor = 1 (wheels), walls and obstacles = 2 (car chassis).
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np
import torch

from mjlab.terrains.terrain_generator import SubTerrainCfg, TerrainGeometry, TerrainOutput

NODES = 4                        # intersections per axis
HALL_WIDTH = 1.4                 # m, a narrow office hallway
ROOM = 2.2                       # m, room block between parallel hallways
EDGE_BAND = 1.1                  # m, outer wall band
WALL_HEIGHT = 0.35               # m, high enough for the 2D LiDAR plane, low enough to see over in the viewer
PATCH_SIZE = 2 * EDGE_BAND + (NODES - 1) * ROOM + NODES * HALL_WIDTH   # 14.4 m

# Directions out of a node: +x, -x, +y, -y. The reverse of d is d ^ 1.
_DIRS = ((1, 0), (-1, 0), (0, 1), (0, -1))

# Cell sizes along one axis: wall band, hall, room, hall, ..., hall, wall band (odd index = hall).
_CELLS = np.array([EDGE_BAND] + [HALL_WIDTH, ROOM] * (NODES - 1) + [HALL_WIDTH, EDGE_BAND])
_CELL_LO = np.concatenate([[0.0], np.cumsum(_CELLS)[:-1]])
_NODE_C = (_CELL_LO + _CELLS / 2)[1::2]           # node centre coordinate per index

# Look (visual only; physics ignores colour). Walls are white, each room gets a coloured roof so
# rooms read as rooms, and each office floor gets its own carpet tint.
_WALL_RGBA = (0.94, 0.93, 0.90, 1.0)
_CARPETS = ((0.36, 0.45, 0.58, 1.0), (0.52, 0.40, 0.36, 1.0), (0.38, 0.50, 0.42, 1.0), (0.55, 0.52, 0.45, 1.0))
_ROOFS = ((0.93, 0.60, 0.20, 1.0), (0.30, 0.55, 0.88, 1.0), (0.40, 0.70, 0.35, 1.0), (0.85, 0.38, 0.45, 1.0),
          (0.58, 0.45, 0.82, 1.0))
_FOLIAGE_RGBA = (0.18, 0.55, 0.22, 1.0)
# Hallway clutter: (name, shape, rgba). Cylinders use the smaller footprint half-size as radius.
_CLUTTER = (
  ("box", "box", (0.74, 0.55, 0.32, 1.0)),          # cardboard box
  ("bin", "cylinder", (0.20, 0.24, 0.22, 1.0)),     # waste bin
  ("plant", "cylinder", (0.72, 0.36, 0.20, 1.0)),   # pot; foliage added on top
  ("cabinet", "box", (0.16, 0.24, 0.46, 1.0)),      # filing cabinet / chair
)


@dataclass
class OfficeLayout:
  node_xy: np.ndarray            # [N, 2] node centres, patch frame
  nbr: np.ndarray                # [N, 4] neighbour node per direction, -1 = wall
  route: np.ndarray              # [N, 4, 2, 2] two route points from node n in direction d


# Layouts in generation order (row-major over the terrain grid), read back by the task.
LAYOUTS: list[OfficeLayout] = []


def _node(ix: int, iy: int) -> int:
  return ix * NODES + iy


def _random_halls(rng: np.random.Generator, extra_loops: int) -> np.ndarray:
  """Adjacency [N, 4] (bool): spanning tree, then no dead ends, then extra loops."""
  n = NODES * NODES
  adj = np.zeros((n, 4), bool)

  def link(a: int, d: int) -> None:
    ix, iy = divmod(a, NODES)
    b = _node(ix + _DIRS[d][0], iy + _DIRS[d][1])
    adj[a, d] = adj[b, d ^ 1] = True

  def options(a: int) -> list[int]:
    ix, iy = divmod(a, NODES)
    return [d for d, (dx, dy) in enumerate(_DIRS) if 0 <= ix + dx < NODES and 0 <= iy + dy < NODES]

  seen, stack = {0}, [0]                          # randomized DFS spanning tree
  while stack:
    a = stack[-1]
    free = [d for d in options(a) if _neighbor(a, d) not in seen]
    if not free:
      stack.pop()
      continue
    d = int(rng.choice(free))
    link(a, d)
    seen.add(_neighbor(a, d))
    stack.append(_neighbor(a, d))
  for a in range(n):                              # no dead ends: the leader never U-turns
    if adj[a].sum() == 1:
      link(a, int(rng.choice([d for d in options(a) if not adj[a, d]])))
  closed = [(a, d) for a in range(n) for d in options(a) if not adj[a, d] and d in (0, 2)]
  for i in rng.permutation(len(closed))[:extra_loops]:
    link(*closed[i])
  return adj


def _neighbor(a: int, d: int) -> int:
  ix, iy = divmod(a, NODES)
  return _node(ix + _DIRS[d][0], iy + _DIRS[d][1])


@dataclass(kw_only=True)
class OfficeTerrainCfg(SubTerrainCfg):
  extra_loops: int = 3
  obstacle_prob: tuple[float, float] = (0.35, 0.8)   # per hallway segment, at difficulty 0 and 1

  def function(self, difficulty: float, spec: mujoco.MjSpec, rng: np.random.Generator) -> TerrainOutput:
    body = spec.body("terrain")
    geoms: list[TerrainGeometry] = []

    def add(shape, center, size, rgba, solid=True) -> None:
      g = body.add_geom(type=shape, size=size, pos=center, rgba=rgba)
      g.contype, g.conaffinity = (2, 2) if solid else (0, 0)
      geoms.append(TerrainGeometry(geom=g, color=rgba))   # the generator applies `color`

    def box(center, half, rgba, solid=True) -> None:
      add(mujoco.mjtGeom.mjGEOM_BOX, center, half, rgba, solid)

    carpet = _CARPETS[rng.integers(len(_CARPETS))]
    floor = body.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=(PATCH_SIZE / 2, PATCH_SIZE / 2, 0.05),
                          pos=(PATCH_SIZE / 2, PATCH_SIZE / 2, -0.05), rgba=carpet)
    geoms.append(TerrainGeometry(geom=floor, color=carpet))

    adj = _random_halls(rng, self.extra_loops)

    # Occupancy over the cell grid: node cells and opened segment cells are hallway.
    m = len(_CELLS)
    hall = np.zeros((m, m), bool)
    for a in range(NODES * NODES):
      ix, iy = divmod(a, NODES)
      hall[2 * ix + 1, 2 * iy + 1] = True
      for d, (dx, dy) in enumerate(_DIRS):
        if adj[a, d]:
          hall[2 * ix + 1 + dx, 2 * iy + 1 + dy] = True

    # Walls: merge runs of solid cells along x into one box per run.
    for j in range(m):
      i = 0
      while i < m:
        if hall[i, j]:
          i += 1
          continue
        k = i
        while k + 1 < m and not hall[k + 1, j]:
          k += 1
        x0, x1 = _CELL_LO[i], _CELL_LO[k] + _CELLS[k]
        y0, y1 = _CELL_LO[j], _CELL_LO[j] + _CELLS[j]
        box(((x0 + x1) / 2, (y0 + y1) / 2, WALL_HEIGHT / 2), ((x1 - x0) / 2, (y1 - y0) / 2, WALL_HEIGHT / 2), _WALL_RGBA)
        i = k + 1

    # Room roofs: a thin coloured cap on every interior room block.
    for i in range(2, m - 2, 2):
      for j in range(2, m - 2, 2):
        x0, y0 = _CELL_LO[i] + 0.05, _CELL_LO[j] + 0.05
        hx, hy = _CELLS[i] / 2 - 0.05, _CELLS[j] / 2 - 0.05
        box((x0 + hx, y0 + hy, WALL_HEIGHT + 0.005), (hx, hy, 0.005), _ROOFS[rng.integers(len(_ROOFS))], solid=False)

    # Hallway segments: optional obstacle against one wall, and the leader's route past it.
    n = NODES * NODES
    node_xy = np.array([[_NODE_C[a // NODES], _NODE_C[a % NODES]] for a in range(n)])
    nbr = np.full((n, 4), -1, np.int64)
    route = np.zeros((n, 4, 2, 2))
    pitch = HALL_WIDTH + ROOM
    p_obs = self.obstacle_prob[0] + (self.obstacle_prob[1] - self.obstacle_prob[0]) * difficulty
    for a in range(n):
      for d in (0, 2):                              # each segment once, from its lower node
        if not adj[a, d]:
          continue
        b = _neighbor(a, d)
        nbr[a, d], nbr[b, d ^ 1] = b, a
        axis = np.array(_DIRS[d], float)            # along the hallway
        lat = np.array([-axis[1], axis[0]])         # across it
        if rng.random() < p_obs:
          s_c = rng.uniform(1.5, pitch - 1.5)       # along position, clear of both intersections
          h_a = rng.uniform(0.1, 0.25)              # half length
          h_l = rng.uniform(0.12, 0.28)             # half width: lane left is >= 0.84 m
          side = rng.choice((-1.0, 1.0))
          c = node_xy[a] + axis * s_c + lat * side * (HALL_WIDTH / 2 - h_l)
          half = np.abs(axis * h_a + lat * h_l)
          kind, shape, rgba = _CLUTTER[rng.integers(len(_CLUTTER))]
          hz = rng.uniform(0.15, 0.35)            # half height; all reach above the LiDAR plane
          if shape == "box":
            box((c[0], c[1], hz), (half[0], half[1], hz), rgba)
          else:
            r = min(h_a, h_l)
            add(mujoco.mjtGeom.mjGEOM_CYLINDER, (c[0], c[1], hz), (r, hz, 0), rgba)
            if kind == "plant":                 # within the pot footprint and above the LiDAR plane
              add(mujoco.mjtGeom.mjGEOM_SPHERE, (c[0], c[1], 2 * hz + 0.9 * r), (r, 0, 0), _FOLIAGE_RGBA, solid=False)
          s1, s2, off = s_c - h_a - 0.4, s_c + h_a + 0.4, -side * h_l   # centre of the free lane, with run-up
        else:
          s1, s2, off = pitch * 0.35, pitch * 0.65, 0.0
        p1 = node_xy[a] + axis * s1 + lat * off
        p2 = node_xy[a] + axis * s2 + lat * off
        route[a, d] = (p1, p2)
        route[b, d ^ 1] = (p2, p1)

    LAYOUTS.append(OfficeLayout(node_xy, nbr, route))
    return TerrainOutput(origin=np.zeros(3), geometries=geoms)


@dataclass
class LayoutTensors:
  node_xy: torch.Tensor          # [L, N, 2]
  nbr: torch.Tensor              # [L, N, 4]
  route: torch.Tensor            # [L, N, 4, 2, 2]


def layout_tensors(num_patches: int, device: str) -> LayoutTensors:
  """The last num_patches generated layouts (the current scene's), stacked."""
  if len(LAYOUTS) < num_patches:
    raise RuntimeError("Office layouts missing: the scene terrain must use OfficeTerrainCfg.")
  ls = LAYOUTS[-num_patches:]
  t = lambda f, dt: torch.tensor(np.stack([getattr(l, f) for l in ls]), dtype=dt, device=device)
  return LayoutTensors(t("node_xy", torch.float32), t("nbr", torch.long), t("route", torch.float32))
