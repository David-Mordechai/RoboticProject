"""Walking people for the office tasks: kinematic walkers on the hallway graph (robot/person.xml).

Each env has up to MAX_PEOPLE walkers. At every reset a random number of them is active; the rest are
parked far outside the office. A walker follows hallway routes (the clutter-free lane the generator
writes, plus its own lateral offset), picks a random onward hallway at each intersection (no U-turns),
stops while a car is close in front of it, and turns back if it has waited YIELD_PATIENCE.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

import torch

from mjlab.entity import Entity
from mjlab.envs.mdp import dr
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg

from robotlab.tasks.mdp import yaw_quat
from robotlab.tasks.office_terrain import NODES, LayoutTensors

if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv

MAX_PEOPLE = 4
# Walking speed, m/s. Adults' comfortable speed is ~1.3-1.4 m/s (Bohannon & Williams Andrews 2011, meta-analysis
# of 41 studies, n = 23,111; Murtagh et al. 2021, outdoor usual pace 1.31 m/s). The lower end covers slower
# strolling in an office: ESTIMATE (no indoor-corridor source found).
WALK_SPEED = (0.6, 1.4)
TURN_RATE = 4.0                  # rad/s, heading change toward the next route point: ESTIMATE (no source)
LANE_OFFSET = 0.12               # m, max lateral offset from the free-lane centre: keeps feet clear of clutter
YIELD_DIST = 0.9                 # m: a walker stops while the car is this close ahead: ESTIMATE (no source)
YIELD_HALF_ANGLE = math.radians(60.0)
YIELD_PATIENCE = 3.0             # s a walker waits for a car before turning back: ESTIMATE (no source)
SWITCH_DIST = 0.3                # m: next route point once this close
TURN_EXIT = 0.55                 # m past an intersection centre: first point of every segment (as the leader)
PARK = (-100.0, -100.0)          # env frame, outside every office patch

# Per-person size randomization, ANSUR II 5th-95th percentile over men and women (see robot/person.xml).
LEG_RADIUS_RANGE = (0.031, 0.071)     # m: ankle (min circumference) to calf (max), C / 2 pi
FOOT_HALF_LENGTH_RANGE = (0.113, 0.147)
FOOT_HALF_WIDTH_RANGE = (0.043, 0.056)

_DIR_VEC = ((1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0))


@dataclass
class Walkers:
  maps: LayoutTensors
  layout: torch.Tensor     # [N] office floor index of each env
  prev: torch.Tensor       # [N, K] node the current segment starts at
  node: torch.Tensor       # [N, K] node it walks to
  dir: torch.Tensor        # [N, K] direction prev -> node
  stage: torch.Tensor      # [N, K] 0 = turn exit, 1, 2 = route points, 3 = node
  xy: torch.Tensor         # [N, K, 2] env frame
  yaw: torch.Tensor        # [N, K]
  speed: torch.Tensor      # [N, K] walking speed when not yielding
  vel: torch.Tensor        # [N, K, 2] current velocity, env frame
  lane: torch.Tensor       # [N, K] lateral offset
  active: torch.Tensor     # [N, K] bool
  waited: torch.Tensor     # [N, K] s spent yielding


def walkers(env: ManagerBasedRlEnv, maps: LayoutTensors, layout: torch.Tensor) -> Walkers:
  if not hasattr(env, "_robotlab_walkers"):
    n, k, dev = env.num_envs, MAX_PEOPLE, env.device
    z = lambda *s, dt=torch.float32: torch.zeros(n, k, *s, device=dev, dtype=dt)
    env._robotlab_walkers = Walkers(maps, layout, z(dt=torch.long), z(dt=torch.long), z(dt=torch.long),
                                    z(dt=torch.long), z(2), z(), z(), z(2), z(), z(dt=torch.bool), z())
  return env._robotlab_walkers


def _target(w: Walkers) -> torch.Tensor:
  dev = w.stage.device
  lay = w.layout[:, None].expand_as(w.prev)
  route = w.maps.route[lay, w.prev, w.dir]                              # [N, K, 2, 2]
  point = torch.gather(route, 2, (w.stage - 1).clamp(0, 1)[..., None, None].expand(*w.stage.shape, 1, 2))[:, :, 0]
  dvec = torch.tensor(_DIR_VEC, device=dev)[w.dir]
  exit_ = w.maps.node_xy[lay, w.prev] + TURN_EXIT * dvec
  node = w.maps.node_xy[lay, w.node]
  t = torch.where((w.stage == 0)[..., None], exit_, torch.where((w.stage == 3)[..., None], node, point))
  return t + w.lane[..., None] * torch.stack([-dvec[..., 1], dvec[..., 0]], -1)


def step_walkers(env: ManagerBasedRlEnv, w: Walkers, car_xy: torch.Tensor) -> None:
  """car_xy [N, 2] or [N, M, 2]: every car a walker yields to (env frame)."""
  dt, dev = env.step_dt, env.device
  cars = car_xy if car_xy.dim() == 3 else car_xy[:, None]
  stage = w.stage + (torch.linalg.norm(_target(w) - w.xy, dim=-1) < SWITCH_DIST).long()
  at_node = stage == 4
  lay = w.layout[:, None].expand_as(w.prev)
  nbr = w.maps.nbr[lay, w.node]                                         # [N, K, 4]
  onward = nbr >= 0
  onward.scatter_(-1, (w.dir ^ 1)[..., None], False)
  onward = torch.where(onward.any(-1, keepdim=True), onward, nbr >= 0)
  new_dir = torch.multinomial(onward.reshape(-1, 4).float(), 1).reshape(w.dir.shape)
  w.prev = torch.where(at_node, w.node, w.prev)
  w.node = torch.where(at_node, torch.gather(nbr, -1, new_dir[..., None])[..., 0], w.node)
  w.dir = torch.where(at_node, new_dir, w.dir)
  w.stage = torch.where(at_node, 0, stage)

  d = _target(w) - w.xy
  err = torch.atan2(torch.sin(torch.atan2(d[..., 1], d[..., 0]) - w.yaw), torch.cos(torch.atan2(d[..., 1], d[..., 0]) - w.yaw))
  w.yaw = w.yaw + err.clamp(-TURN_RATE * dt, TURN_RATE * dt)
  heading = torch.stack([torch.cos(w.yaw), torch.sin(w.yaw)], -1)
  rel = cars[:, None] - w.xy[:, :, None]                              # [N, K, M, 2]
  dist = torch.linalg.norm(rel, dim=-1)
  ahead = (rel * heading[:, :, None]).sum(-1) > dist * math.cos(YIELD_HALF_ANGLE)
  blocked = ((dist < YIELD_DIST) & ahead).any(-1)
  w.waited = torch.where(blocked, w.waited + dt, 0.0)
  # Waited long enough: turn back (the reversed segment; head for the point now in front).
  back = blocked & (w.waited > YIELD_PATIENCE)
  if back.any():
    w.prev, w.node = torch.where(back, w.node, w.prev), torch.where(back, w.prev, w.node)
    w.dir = torch.where(back, w.dir ^ 1, w.dir)
    w.stage = torch.where(back, torch.tensor([3, 3, 2, 1], device=dev)[w.stage.clamp(0, 3)], w.stage)
    w.yaw = torch.where(back, w.yaw + math.pi, w.yaw)
    w.waited = torch.where(back, 0.0, w.waited)
    blocked = blocked & ~back
    heading = torch.stack([torch.cos(w.yaw), torch.sin(w.yaw)], -1)
  speed = torch.where(blocked, 0.0, w.speed) * w.active
  w.vel = speed[..., None] * heading
  w.xy = w.xy + w.vel * dt
  write_walkers(env, w, torch.arange(env.num_envs, device=dev))


def write_walkers(env: ManagerBasedRlEnv, w: Walkers, env_ids: torch.Tensor) -> None:
  for k in range(MAX_PEOPLE):
    person: Entity = env.scene[f"person{k}"]
    xy = torch.where(w.active[env_ids, k, None], w.xy[env_ids, k], torch.tensor(PARK, device=env.device))
    pos = torch.cat([xy, torch.zeros_like(xy[:, :1])], -1) + env.scene.env_origins[env_ids]
    person.write_mocap_pose_to_sim(torch.cat([pos, yaw_quat(w.yaw[env_ids, k])], -1), env_ids=env_ids)


def reset_walkers(env: ManagerBasedRlEnv, w: Walkers, env_ids: torch.Tensor, car_xy: torch.Tensor,
                  keep_clear: float = 1.5) -> None:
  """New walkers: random count, random hallway positions; none within keep_clear of the car's start."""
  n, k, dev = len(env_ids), MAX_PEOPLE, env.device
  lay = w.layout[env_ids, None].expand(n, k)
  a = torch.randint(0, NODES * NODES, (n, k), device=dev)
  nbr = w.maps.nbr[lay, a]
  d = torch.multinomial((nbr >= 0).reshape(-1, 4).float(), 1).reshape(n, k)
  dvec = torch.tensor(_DIR_VEC, device=dev)[d]
  xy = w.maps.node_xy[lay, a] + (0.9 + 1.6 * torch.rand(n, k, 1, device=dev)) * dvec
  w.prev[env_ids], w.dir[env_ids], w.stage[env_ids] = a, d, 1
  w.node[env_ids] = torch.gather(nbr, -1, d[..., None])[..., 0]
  w.lane[env_ids] = LANE_OFFSET * (2 * torch.rand(n, k, device=dev) - 1)
  w.xy[env_ids] = xy + w.lane[env_ids, :, None] * torch.stack([-dvec[..., 1], dvec[..., 0]], -1)
  w.yaw[env_ids] = torch.atan2(dvec[..., 1], dvec[..., 0])
  w.speed[env_ids] = WALK_SPEED[0] + (WALK_SPEED[1] - WALK_SPEED[0]) * torch.rand(n, k, device=dev)
  w.vel[env_ids] = 0.0
  w.waited[env_ids] = 0.0
  count = torch.randint(0, MAX_PEOPLE + 1, (n, 1), device=dev)
  clear = torch.linalg.norm(w.xy[env_ids] - car_xy[env_ids, None], dim=-1) > keep_clear
  w.active[env_ids] = (torch.arange(k, device=dev)[None] < count) & clear
  write_walkers(env, w, env_ids)


def people_size_events() -> dict[str, EventTermCfg]:
  """Per-person leg and foot sizes, redrawn at every reset (ANSUR II ranges above)."""
  events = {}
  for k in range(MAX_PEOPLE):
    events[f"person{k}_legs"] = EventTermCfg(
      func=dr.geom_size, mode="reset",
      params={"asset_cfg": SceneEntityCfg(f"person{k}", geom_names=("leg_.*",)), "operation": "abs",
              "ranges": {0: LEG_RADIUS_RANGE}, "axes": [0], "shared_random": True},
    )
    events[f"person{k}_feet"] = EventTermCfg(
      func=dr.geom_size, mode="reset",
      params={"asset_cfg": SceneEntityCfg(f"person{k}", geom_names=("foot_.*",)), "operation": "abs",
              "ranges": {0: FOOT_HALF_LENGTH_RANGE, 1: FOOT_HALF_WIDTH_RANGE}, "axes": [0, 1],
              "shared_random": True},
    )
  return events
