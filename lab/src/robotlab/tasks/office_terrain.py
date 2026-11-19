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
HALL_WIDTH = 1.4                 # m, a narrow office hallway (default width)
PITCH = 3.6                      # m between parallel hallway centres: hallway + room block (1.4 + 2.2 originally)
WALL_HEIGHT = 0.35               # m, high enough for the 2D LiDAR plane, low enough to see over in the viewer
PATCH_SIZE = NODES * PITCH       # 14.4 m; the outer wall band is (PITCH - width) / 2, so node centres never move
MAX_CLUTTER = 3 * 2 * NODES * (NODES - 1)   # per hallway segment: one obstacle / chair + up to 2 desk legs (goto v8)

# Directions out of a node: +x, -x, +y, -y. The reverse of d is d ^ 1.
_DIRS = ((1, 0), (-1, 0), (0, 1), (0, -1))



def _cells(width: float) -> tuple[np.ndarray, np.ndarray]:
  """Cell sizes along one axis (wall band, hall, room, hall, ..., hall, wall band; odd index = hall) and their
  lower edges, for a hallway width."""
  edge = (PITCH - width) / 2
  cells = np.array([edge] + [width, PITCH - width] * (NODES - 1) + [width, edge])
  return cells, np.concatenate([[0.0], np.cumsum(cells)[:-1]])


_NODE_C = PITCH / 2 + PITCH * np.arange(NODES)    # node centre coordinate per index (same for every width)

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


# Office chair bases (goto v7, 2026-11-08; floor goto17: car #1 hit a chair's caster near the charger). A 5-star base
# lies under the LiDAR plane (~0.14 m here, mentorpi_a1.xml), so the LiDAR sees only the centre column; the depth camera
# sees the legs. ESTIMATES (no chair measured): base radius CHAIR_R (common 5-star bases are ~0.6-0.7 m across), leg top
# height CHAIR_LEG_TOP (legs + casters), leg width 0.05 m, column radius 0.025 m up to the seat at ~0.45 m.
CHAIR_R = (0.28, 0.34)
CHAIR_LEG_TOP = (0.08, 0.12)
CHAIR_LEG_HALF_W = 0.025
CHAIR_COLUMN_R, CHAIR_SEAT_Z = 0.025, 0.45
_CHAIR_RGBA = (0.12, 0.12, 0.13, 1.0)

# Desk legs on the route (goto v8, 2026-11-09). Floor goto18/19: the imitation student drove straight into a desk leg
# beside the charger. In every sim hallway the route ran around the obstacles, so following the route alone was enough
# and the student learned to ignore its sensors ("causal confusion", de Haan et al., NeurIPS 2019). Legs placed on or
# near the route line, which the route does not know about, make the sensors necessary. ESTIMATES (the office desk legs
# are unmeasured). Camera frames goto18 0018.6 / 0052.4: round tubes, slanted, in A-frame pairs (two legs from one top
# point splayed apart, the feet ~0.3-0.5 m apart), so a leg's foot (depth camera) and its LiDAR-height point differ.
# Modelled: capsules of radius DESK_LEG_R from a shared top at DESK_LEG_H to feet DESK_LEG_SPREAD apart each side (a
# random horizontal direction), or one leg tilted alone (DESK_LEG_SINGLE share); lateral offset from the route line up
# to DESK_LEG_OFF.
DESK_LEG_R = (0.018, 0.028)
DESK_LEG_H = 0.72
DESK_LEG_SPREAD = (0.12, 0.26)
DESK_LEG_SINGLE = 0.3
DESK_LEG_OFF = 0.15
# Clear passage kept beside a leg set (design choice): car width ~0.2 m + the stop's corridor 2 x 0.14 m + margin.
DESK_LEG_PASS = 0.55
_DESK_LEG_RGBA = (0.62, 0.78, 0.30, 1.0)

OCC_RES = 0.05                   # m per occupancy cell (ground truth for "is there room here", used by rewards)
OCC_N = int(round(PATCH_SIZE / OCC_RES))


@dataclass
class OfficeLayout:
  node_xy: np.ndarray            # [N, 2] node centres, patch frame
  nbr: np.ndarray                # [N, 4] neighbour node per direction, -1 = wall
  route: np.ndarray              # [N, 4, 2, 2] two route points from node n in direction d
  occ: np.ndarray                # [OCC_N, OCC_N] bool, walls and clutter footprints, index [ix, iy]
  walls: np.ndarray              # [OCC_N, OCC_N] bool, walls only
  clutter: np.ndarray            # [OCC_N, OCC_N] int16, obstacle id 1..MAX_CLUTTER, 0 = none
  hall_width: float              # m
  chairs: tuple[int, ...] = ()   # clutter ids that are chair bases (goto v7)


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
  hall_width: tuple[float, float] = (HALL_WIDTH, HALL_WIDTH)   # m, drawn once per floor
  # Doorways (goto, 2026-11-04): with this probability a hallway segment gets a wall across it with a door-sized gap
  # off to one side instead of an obstacle, and the route goes through the gap: the car meets a wall ahead and has to
  # turn sharply into a narrow opening, as at the office door (goto2 froze there, facing the wall). 0 = none, and the
  # generator draws no extra random numbers, so other tasks' floors are unchanged.
  door_prob: float = 0.0
  door_gap: tuple[float, float] = (0.70, 0.90)   # m clear width: ESTIMATE (interior doors ~0.8 m; office door unmeasured)
  # Chair bases (goto v7): with this probability a hallway segment without a door or obstacle gets an office chair
  # against one wall, and the route runs down the free lane beside it. 0 = none, and no extra random numbers are drawn.
  chair_prob: float = 0.0
  # goto v8 (see DESK_LEG_*): per hallway segment without a door, the probability of 1-2 desk legs on the route line,
  # and a lateral random shift of the route (m, uniform +-route_jitter; kept 0.25 m from the walls), so the route is not
  # always the free lane. 0 = none, and no extra random numbers are drawn.
  route_leg_prob: float = 0.0
  route_jitter: float = 0.0

  def function(self, difficulty: float, spec: mujoco.MjSpec, rng: np.random.Generator) -> TerrainOutput:
    body = spec.body("terrain")
    geoms: list[TerrainGeometry] = []

    def add(shape, center, size, rgba, solid=True, quat=None) -> None:
      g = body.add_geom(type=shape, size=size, pos=center, rgba=rgba)
      if quat is not None:
        g.quat = quat
      g.contype, g.conaffinity = (2, 2) if solid else (0, 0)
      geoms.append(TerrainGeometry(geom=g, color=rgba))   # the generator applies `color`

    def box(center, half, rgba, solid=True) -> None:
      add(mujoco.mjtGeom.mjGEOM_BOX, center, half, rgba, solid)

    # The wheels touch the floor through one ground plane (top surface z = 0) shared by all patches. The
    # carpet boxes are visual only. Reason: MuJoCo Warp has no analytic cylinder-box collider; it sends
    # tyre-on-box through float32 convex CCD, which for a grazing tyre on a 14 m box sometimes returns a
    # contact metres away with a horizontal normal, and the step goes NaN (CPU MuJoCo is correct on the same
    # state; still in mujoco-warp 3.17). Plane-cylinder has an exact primitive collider on both.
    if not any(g.name == "ground" for g in body.geoms):
      body.add_geom(name="ground", type=mujoco.mjtGeom.mjGEOM_PLANE, size=(0, 0, 1), rgba=(0, 0, 0, 0))
    carpet = _CARPETS[rng.integers(len(_CARPETS))]
    floor = body.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=(PATCH_SIZE / 2, PATCH_SIZE / 2, 0.05),
                          pos=(PATCH_SIZE / 2, PATCH_SIZE / 2, -0.05), rgba=carpet)
    floor.contype, floor.conaffinity = 0, 0
    geoms.append(TerrainGeometry(geom=floor, color=carpet))

    adj = _random_halls(rng, self.extra_loops)
    width = float(rng.uniform(*self.hall_width))
    _CELLS, _CELL_LO = _cells(width)

    # Occupancy over the cell grid: node cells and opened segment cells are hallway.
    m = len(_CELLS)
    hall = np.zeros((m, m), bool)
    for a in range(NODES * NODES):
      ix, iy = divmod(a, NODES)
      hall[2 * ix + 1, 2 * iy + 1] = True
      for d, (dx, dy) in enumerate(_DIRS):
        if adj[a, d]:
          hall[2 * ix + 1 + dx, 2 * iy + 1 + dy] = True

    occ = np.zeros((OCC_N, OCC_N), bool)
    clutter = np.zeros((OCC_N, OCC_N), np.int16)
    cell = (np.arange(OCC_N) + 0.5) * OCC_RES                  # cell centres
    def fill_box(x0, x1, y0, y1, cid=0):
      m_ = np.zeros_like(occ)
      m_[np.ix_((cell >= x0) & (cell <= x1), (cell >= y0) & (cell <= y1))] = True
      occ[m_] = True
      clutter[m_] = cid
    def fill_disc(cx, cy, r, cid=0):
      m_ = (cell[:, None] - cx) ** 2 + (cell[None, :] - cy) ** 2 <= r * r
      occ[m_] = True
      clutter[m_] = cid

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
        fill_box(x0, x1, y0, y1)
        i = k + 1
    walls = occ.copy()

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
    pitch = PITCH
    cid = 0
    chairs: list[int] = []
    p_obs = self.obstacle_prob[0] + (self.obstacle_prob[1] - self.obstacle_prob[0]) * difficulty
    for a in range(n):
      for d in (0, 2):                              # each segment once, from its lower node
        if not adj[a, d]:
          continue
        b = _neighbor(a, d)
        nbr[a, d], nbr[b, d ^ 1] = b, a
        axis = np.array(_DIRS[d], float)            # along the hallway
        lat = np.array([-axis[1], axis[0]])         # across it
        door = self.door_prob > 0 and rng.random() < self.door_prob   # door segments keep their exact gap route
        if door:
          s_c = rng.uniform(1.2, pitch - 1.2)       # along position, clear of both intersections
          g = rng.uniform(*self.door_gap)
          off = rng.uniform(-(width - g) / 2, (width - g) / 2)   # gap centre across the hallway
          for lo, hi in ((-width / 2, off - g / 2), (off + g / 2, width / 2)):
            if hi - lo < 0.02:
              continue
            c = node_xy[a] + axis * s_c + lat * (lo + hi) / 2
            half = np.abs(axis * 0.05 + lat * (hi - lo) / 2)
            box((c[0], c[1], WALL_HEIGHT / 2), (half[0], half[1], WALL_HEIGHT / 2), _WALL_RGBA)
            fill_box(c[0] - half[0], c[0] + half[0], c[1] - half[1], c[1] + half[1])
          s1, s2 = s_c - 0.6, s_c + 0.6             # straight through the gap, with run-up on both sides
        elif rng.random() < p_obs:
          s_c = rng.uniform(1.5, pitch - 1.5)       # along position, clear of both intersections
          h_a = rng.uniform(0.1, 0.25)              # half length
          h_l = rng.uniform(0.12, 0.28)             # half width: lane left is >= 0.84 m
          side = rng.choice((-1.0, 1.0))
          c = node_xy[a] + axis * s_c + lat * side * (width / 2 - h_l)
          cid += 1
          half = np.abs(axis * h_a + lat * h_l)
          kind, shape, rgba = _CLUTTER[rng.integers(len(_CLUTTER))]
          hz = rng.uniform(0.15, 0.35)            # half height; all reach above the LiDAR plane
          if shape == "box":
            box((c[0], c[1], hz), (half[0], half[1], hz), rgba)
            fill_box(c[0] - half[0], c[0] + half[0], c[1] - half[1], c[1] + half[1], cid)
          else:
            r = min(h_a, h_l)
            fill_disc(c[0], c[1], r, cid)
            add(mujoco.mjtGeom.mjGEOM_CYLINDER, (c[0], c[1], hz), (r, hz, 0), rgba)
            if kind == "plant":                 # within the pot footprint and above the LiDAR plane
              add(mujoco.mjtGeom.mjGEOM_SPHERE, (c[0], c[1], 2 * hz + 0.9 * r), (r, 0, 0), _FOLIAGE_RGBA, solid=False)
          s1, s2, off = s_c - h_a - 0.4, s_c + h_a + 0.4, -side * h_l   # centre of the free lane, with run-up
        elif self.chair_prob > 0 and rng.random() < self.chair_prob and width > 2 * CHAIR_R[1] + 0.30:
          s_c = rng.uniform(1.4, pitch - 1.4)       # along position, clear of both intersections
          R = rng.uniform(*CHAIR_R)
          top = rng.uniform(*CHAIR_LEG_TOP)
          side = rng.choice((-1.0, 1.0))
          c = node_xy[a] + axis * s_c + lat * side * (width / 2 - R - rng.uniform(0.0, 0.10))
          cid += 1
          chairs.append(cid)
          fill_disc(c[0], c[1], R, cid)
          add(mujoco.mjtGeom.mjGEOM_CYLINDER, (c[0], c[1], CHAIR_SEAT_Z / 2), (CHAIR_COLUMN_R, CHAIR_SEAT_Z / 2, 0),
              _CHAIR_RGBA)                          # the centre column: the only part in the LiDAR plane
          add(mujoco.mjtGeom.mjGEOM_CYLINDER, (c[0], c[1], CHAIR_SEAT_Z), (0.23, 0.03, 0), _CHAIR_RGBA, solid=False)
          phi = rng.uniform(0, 2 * np.pi)
          half_l = (R - CHAIR_COLUMN_R) / 2
          for k in range(5):                        # 5 legs from the column out to R, under the LiDAR plane
            ang = phi + k * 2 * np.pi / 5
            m = c + (CHAIR_COLUMN_R + half_l) * np.array([np.cos(ang), np.sin(ang)])
            add(mujoco.mjtGeom.mjGEOM_BOX, (m[0], m[1], (0.01 + top) / 2), (half_l, CHAIR_LEG_HALF_W, (top - 0.01) / 2),
                _CHAIR_RGBA, quat=(np.cos(ang / 2), 0.0, 0.0, np.sin(ang / 2)))
          s1, s2, off = s_c - R - 0.4, s_c + R + 0.4, -side * R   # centre of the free lane beside the chair
        else:
          s1, s2, off = pitch * 0.35, pitch * 0.65, 0.0
        if self.route_jitter > 0 and not door:
          off = float(np.clip(off + rng.uniform(-self.route_jitter, self.route_jitter), -(width / 2 - 0.25),
                              width / 2 - 0.25))
        if self.route_leg_prob > 0 and not door and rng.random() < self.route_leg_prob:
          for _ in range(int(rng.integers(1, 3))):
            s_l = rng.uniform(1.0, pitch - 1.0)    # along: between the intersections, independent of the route points
            lo_l = float(np.clip(off + rng.uniform(-DESK_LEG_OFF, DESK_LEG_OFF), -(width / 2 - 0.45), width / 2 - 0.45))
            r = rng.uniform(*DESK_LEG_R)          # (>= 0.4 m free on one side of the leg)
            c = node_xy[a] + axis * s_l + lat * lo_l   # top point, above the route region
            cid += 1
            phi = rng.uniform(0, 2 * np.pi)
            u = np.array([np.cos(phi), np.sin(phi)])
            sp = rng.uniform(*DESK_LEG_SPREAD)
            feet = [c + u * sp] if rng.random() < DESK_LEG_SINGLE else [c + u * sp, c - u * sp]
            # keep a passage of DESK_LEG_PASS beside the legs (halls are 1.0-1.8 m; an A-frame across a 1.0 m hall
            # left no way through: teacher 10/16, idle 7.5%): shift the legs across the hall if needed
            lats = [float((q_ - node_xy[a]) @ lat) for q_ in feet + [c]]
            lo_e, hi_e = min(lats) - r, max(lats) + r
            gap_hi, gap_lo = width / 2 - hi_e, lo_e + width / 2
            if max(gap_hi, gap_lo) < DESK_LEG_PASS:
              sh = -(DESK_LEG_PASS - gap_hi) if gap_hi >= gap_lo else DESK_LEG_PASS - gap_lo
              if (hi_e - lo_e) + DESK_LEG_PASS > width - 0.05:   # does not fit at all: skip this leg set
                continue
              c = c + lat * sh
              feet = [f + lat * sh for f in feet]
            for f in feet:
              # pos + quat (the terrain generator shifts geom pos later, which MuJoCo rejects together with fromto)
              p0, p1 = np.array([f[0], f[1], r]), np.array([c[0], c[1], DESK_LEG_H])
              dv = p1 - p0; L_ = float(np.linalg.norm(dv)); zc = dv / L_
              ax = np.cross([0.0, 0.0, 1.0], zc); sa = float(np.linalg.norm(ax)); th = np.arctan2(sa, zc[2])
              q = (np.cos(th / 2), *(np.sin(th / 2) * ax / max(sa, 1e-9)))
              add(mujoco.mjtGeom.mjGEOM_CAPSULE, tuple((p0 + p1) / 2), (r, L_ / 2, 0), _DESK_LEG_RGBA, quat=q)
              for z in np.linspace(0.0, 0.40, 6):   # occupancy where the sensors' band crosses the slanted leg
                p_ = f + (c - f) * (z / DESK_LEG_H)
                fill_disc(p_[0], p_[1], max(r, OCC_RES * 0.75), cid)
        p1 = node_xy[a] + axis * s1 + lat * off
        p2 = node_xy[a] + axis * s2 + lat * off
        route[a, d] = (p1, p2)
        route[b, d ^ 1] = (p2, p1)

    LAYOUTS.append(OfficeLayout(node_xy, nbr, route, occ, walls, clutter, width, tuple(chairs)))
    return TerrainOutput(origin=np.zeros(3), geometries=geoms)


@dataclass
class LayoutTensors:
  node_xy: torch.Tensor          # [L, N, 2]
  nbr: torch.Tensor              # [L, N, 4]
  route: torch.Tensor            # [L, N, 4, 2, 2]
  occ: torch.Tensor              # [L, OCC_N, OCC_N] bool
  walls: torch.Tensor            # [L, OCC_N, OCC_N] bool
  clutter: torch.Tensor          # [L, OCC_N, OCC_N] long, obstacle id, 0 = none
  hall_width: torch.Tensor       # [L]


def layout_tensors(num_patches: int, device: str) -> LayoutTensors:
  """The last num_patches generated layouts (the current scene's), stacked."""
  if len(LAYOUTS) < num_patches:
    raise RuntimeError("Office layouts missing: the scene terrain must use OfficeTerrainCfg.")
  ls = LAYOUTS[-num_patches:]
  t = lambda f, dt: torch.tensor(np.stack([getattr(l, f) for l in ls]), dtype=dt, device=device)
  return LayoutTensors(t("node_xy", torch.float32), t("nbr", torch.long), t("route", torch.float32), t("occ", torch.bool),
                       t("walls", torch.bool), t("clutter", torch.long), t("hall_width", torch.float32))
