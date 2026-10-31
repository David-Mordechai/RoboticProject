"""Navigate task: a MentorPi A1 drives planned routes through the office to goal after goal, among walking people.

The route comes from a planner over the hallway graph (stand-in for Nav2 on the shared map): shortest path
from the car to a random goal intersection, through each hallway's clutter-free lane. The policy sees the
route as a preview of points ahead, in its own frame (on the real car: the Nav2 path transformed with the
car's localized pose, so localization error moves the whole preview). It must not follow it blindly: walls,
clutter and people come only from the LiDAR. When a goal is reached the next one is drawn, so an episode
is a string of errands. People (tasks/people.py) walk the hallways and stop when the car is close ahead.

This is the driving skill under every autonomous behaviour (project-plan §13): go home, go to a person's room.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
import torch

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs.mdp import action_rate_l2, reset_joints_by_offset, time_out
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.observation_manager import ObservationGroupCfg, ObservationTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.managers.termination_manager import TerminationTermCfg
from mjlab.rl import RslRlModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg
from mjlab.scene import SceneCfg
from mjlab.sensor import ContactMatch, ContactSensorCfg, ObjRef, RayCastSensorCfg
from mjlab.sim import MujocoCfg, SimulationCfg
from mjlab.terrains import TerrainEntityCfg
from mjlab.terrains.terrain_generator import TerrainGeneratorCfg
from mjlab.viewer import ViewerConfig

from robotlab.robot.mentorpi_a1_constants import STEER_2024, STEER_CODE, V_MAX, car_cfg, goal_marker_cfg, person_cfg
from robotlab.tasks import randomization as R
from robotlab.tasks.mdp import LidarPatternCfg, AckermannActionCfg, car_motion, car_pose, place_car, to_frame, yaw_quat
from robotlab.tasks.office_terrain import NODES, PATCH_SIZE, LayoutTensors, OfficeTerrainCfg, layout_tensors
from robotlab.tasks.people import MAX_PEOPLE, people_size_events, reset_walkers, step_walkers, walkers

if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv

LIDAR_RAYS = 360                 # 1 deg apart: LD19 / MS200 resolve 0.8 deg (research/03 §1.1)
LIDAR_SECTORS = 36               # what the policy sees: nearest return per 10 deg sector (min-pooled)
LIDAR_RANGE = 3.0                # m, clipped
PREVIEW = tuple(0.25 * (i + 1) for i in range(8))   # m of route ahead of the car's progress point
REMAIN_CLIP = 3.0                # m, remaining route length is clipped to this in the observation
GOAL_RADIUS = 0.35               # m: goal reached
MAX_SEGMENTS = 2 * NODES * NODES # longest shortest path is far shorter; padding bound
PATH_POINTS = 1 + 3 * (MAX_SEGMENTS + 1)
PROJ_WINDOW = 6                  # segments searched ahead of the current one for the progress point
# Personal space: Hall (1966) intimate zone < 0.45 m; robot studies use Hall's zones as reference and find
# many people accept a robot at personal distance 0.45-1.2 m (Huettenrauch et al. 2006; Walters et al. 2009).
PERSONAL_SPACE = 0.45            # m, clearance between car body and person
PERSONAL_ZONE = 1.2              # m, outer edge of Hall's personal zone: inside it the car should slow down
# Human comfort drops as robot speed rises (robot-approach studies in the proxemics literature, e.g. the review in
# Frontiers in Robotics and AI 2022, doi:10.3389/frobt.2022.915972), so speed near people is penalized below.
CAR_RADIUS = math.hypot(0.1065, 0.0795)   # m, half-diagonal of the 213 x 159 mm footprint (research/01 §2)
PERSON_RADIUS = 0.20             # m, legs (+-0.087 + r) and toes (0.19 m ahead) of robot/person.xml: ESTIMATE
OFFICE_ROWS = OFFICE_COLS = 4
_DIR_VEC = ((1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0))
_DIR_YAW = (0.0, math.pi, math.pi / 2, -math.pi / 2)


# Planner: next direction on a shortest path, per layout (BFS over the hallway graph).


def _next_dir(nbr: np.ndarray) -> np.ndarray:
  """nbr [L, N, 4] -> next [L, N, N]: direction to leave node a toward goal g (-1 if a == g)."""
  L, n, _ = nbr.shape
  out = np.full((L, n, n), -1, np.int64)
  for l in range(L):
    for g in range(n):                     # BFS from the goal; each node remembers its step toward g
      seen, q = {g}, deque([g])
      while q:
        b = q.popleft()
        for d in range(4):
          a = nbr[l, b, d]
          if a >= 0 and a not in seen:
            seen.add(a)
            out[l, a, g] = d ^ 1           # from a, go opposite of d to reach b
            q.append(a)
  return out


@dataclass
class Nav:
  maps: LayoutTensors
  next_dir: torch.Tensor   # [L, N, N]
  layout: torch.Tensor     # [N]
  path: torch.Tensor       # [N, P, 2] route polyline, env frame (padded with the goal)
  cum: torch.Tensor        # [N, P] arc length at each point
  goal: torch.Tensor       # [N] goal node
  arrive_dir: torch.Tensor # [N] direction the route enters the goal node
  seg: torch.Tensor        # [N] current segment (path[seg] -> path[seg + 1])
  s: torch.Tensor          # [N] progress along the route, m
  s_prev: torch.Tensor     # [N]
  goals_reached: torch.Tensor   # [N] this episode
  reached: torch.Tensor    # [N] bool, reached a goal this step
  loc_bias: torch.Tensor   # [N, 3] localization error (dx, dy, dyaw), per episode
  loc_jump: torch.Tensor   # [N, 3] current correction jump


def nav(env: ManagerBasedRlEnv) -> Nav:
  if not hasattr(env, "_robotlab_nav"):
    terrain, n, dev = env.scene.terrain, env.num_envs, env.device
    num_cols = terrain.terrain_origins.shape[1]
    maps = layout_tensors(terrain.terrain_origins.shape[0] * num_cols, dev)
    nd = torch.tensor(_next_dir(maps.nbr.cpu().numpy()), device=dev)
    z = lambda *s, dt=torch.float32: torch.zeros(n, *s, device=dev, dtype=dt)
    env._robotlab_nav = Nav(maps, nd, terrain.terrain_levels * num_cols + terrain.terrain_types,
                            z(PATH_POINTS, 2), z(PATH_POINTS), z(dt=torch.long), z(dt=torch.long), z(dt=torch.long),
                            z(), z(), z(), z(dt=torch.bool), z(3), z(3))
  return env._robotlab_nav


def _walk_to_goal(nv: Nav, ids: torch.Tensor, pts: list[torch.Tensor], node: torch.Tensor, goal: torch.Tensor,
                  arrive: torch.Tensor) -> torch.Tensor:
  """Append route points from node to goal (three per hallway: two lane points, then the next node)."""
  lay = nv.layout[ids]
  for _ in range(MAX_SEGMENTS):
    d = nv.next_dir[lay, node, goal]
    going = d >= 0
    dd = d.clamp(min=0)
    r = nv.maps.route[lay, node, dd]                                    # [n, 2, 2]
    nxt = torch.where(going, nv.maps.nbr[lay, node, dd], node)
    here = nv.maps.node_xy[lay, nxt]
    pts += [torch.where(going[:, None], r[:, 0], here), torch.where(going[:, None], r[:, 1], here), here]
    arrive = torch.where(going, dd, arrive)
    node = nxt
  return arrive


def _pick_goal(nv: Nav, ids: torch.Tensor, start: torch.Tensor, heading_dir: torch.Tensor) -> torch.Tensor:
  """A random goal node != start whose route does not begin with a U-turn."""
  lay = nv.layout[ids]
  first = nv.next_dir[lay, start]                                       # [n, N]
  ok = (first >= 0) & (first != (heading_dir ^ 1)[:, None])
  ok = torch.where(ok.any(-1, keepdim=True), ok, first >= 0)
  return torch.multinomial(ok.float(), 1)[:, 0]


def _set_path(nv: Nav, ids: torch.Tensor, pts: list[torch.Tensor], car_xy: torch.Tensor) -> None:
  p = torch.stack(pts, 1)
  p = torch.cat([p, p[:, -1:].expand(-1, PATH_POINTS - p.shape[1], -1)], 1)[:, :PATH_POINTS]
  nv.path[ids] = p
  nv.cum[ids] = torch.cat([torch.zeros_like(p[:, :1, 0]), torch.linalg.norm(p.diff(dim=1), dim=-1).cumsum(1)], 1)
  nv.seg[ids], nv.s[ids], nv.s_prev[ids] = 0, 0.0, 0.0
  _project(nv, ids, car_xy)
  nv.s_prev[ids] = nv.s[ids]


def _project(nv: Nav, ids: torch.Tensor, xy: torch.Tensor) -> torch.Tensor:
  """Advance the progress point to the closest route point within the next PROJ_WINDOW segments (never back).
  Returns the distance from xy to the route."""
  ar = torch.arange(PROJ_WINDOW, device=xy.device)
  i = (nv.seg[ids, None] + ar).clamp(max=PATH_POINTS - 2)              # [n, W]
  a = torch.gather(nv.path[ids], 1, i[..., None].expand(-1, -1, 2))
  b = torch.gather(nv.path[ids], 1, (i + 1)[..., None].expand(-1, -1, 2))
  ab = b - a
  t = (((xy[:, None] - a) * ab).sum(-1) / (ab * ab).sum(-1).clamp(min=1e-9)).clamp(0, 1)
  dist = torch.linalg.norm(a + t[..., None] * ab - xy[:, None], dim=-1)
  k = dist.argmin(-1)
  kk = k[:, None]
  seg = torch.gather(i, 1, kk)[:, 0]
  s = torch.gather(nv.cum[ids], 1, seg[:, None])[:, 0] + torch.gather(t, 1, kk)[:, 0] * torch.linalg.norm(
    torch.gather(ab, 1, kk[..., None].expand(-1, -1, 2))[:, 0], dim=-1)
  nv.seg[ids], nv.s[ids] = seg, torch.maximum(s, nv.s[ids])
  return torch.gather(dist, 1, kk)[:, 0]


def _points_at(nv: Nav, s: torch.Tensor) -> torch.Tensor:
  """Route points at arc lengths s [N, M] -> [N, M, 2]."""
  s = torch.minimum(s, nv.cum[:, -1:])
  j = (torch.searchsorted(nv.cum.contiguous(), s.contiguous(), right=True) - 1).clamp(0, PATH_POINTS - 2)
  c0, c1 = torch.gather(nv.cum, 1, j), torch.gather(nv.cum, 1, j + 1)
  t = ((s - c0) / (c1 - c0).clamp(min=1e-9)).clamp(0, 1)[..., None]
  a = torch.gather(nv.path, 1, j[..., None].expand(-1, -1, 2))
  b = torch.gather(nv.path, 1, (j + 1)[..., None].expand(-1, -1, 2))
  return a + t * (b - a)


def _remaining(nv: Nav) -> torch.Tensor:
  return nv.cum[:, -1] - nv.s


# Per-step update (runs once per policy step, from the action term, before physics).


def advance_world(env: ManagerBasedRlEnv) -> None:
  nv, (xy, yaw) = nav(env), car_pose(env)
  ids = torch.arange(env.num_envs, device=env.device)
  nv.s_prev[:] = nv.s
  _project(nv, ids, xy)
  # Goal reached: count it and plan the next errand from the goal node.
  goal_xy = nv.maps.node_xy[nv.layout, nv.goal]
  nv.reached = (_remaining(nv) < GOAL_RADIUS) & (torch.linalg.norm(goal_xy - xy, dim=-1) < GOAL_RADIUS + 0.15)
  if nv.reached.any():
    r = nv.reached.nonzero()[:, 0]
    nv.goals_reached[r] += 1
    start = nv.goal[r]
    g = _pick_goal(nv, r, start, nv.arrive_dir[r])
    pts = [xy[r]]
    nv.arrive_dir[r] = _walk_to_goal(nv, r, pts, start, g, nv.arrive_dir[r])
    nv.goal[r] = g
    _set_path(nv, r, pts, xy[r])
    nv.s_prev[r] = nv.s[r]                  # a new route is not progress
  # Localization correction jumps at the stock update cadence.
  if R.ENABLE_LOCALIZATION_NOISE and env.common_step_counter % R.LOC_JUMP_STEPS == 0:
    std = torch.tensor([R.LOC_JUMP_M, R.LOC_JUMP_M, R.LOC_JUMP_YAW], device=env.device)
    nv.loc_jump[:] = std * torch.randn_like(nv.loc_jump)
  step_walkers(env, walkers(env, nv.maps, nv.layout), xy)
  _write_goal_marker(env, ids)


def _write_goal_marker(env: ManagerBasedRlEnv, env_ids: torch.Tensor) -> None:
  nv = nav(env)
  xy = nv.maps.node_xy[nv.layout[env_ids], nv.goal[env_ids]]
  pos = torch.cat([xy, torch.zeros_like(xy[:, :1])], -1) + env.scene.env_origins[env_ids]
  env.scene["goal"].write_mocap_pose_to_sim(torch.cat([pos, yaw_quat(torch.zeros_like(xy[:, 0]))], -1), env_ids=env_ids)


# Events.


def reset_navigate(env: ManagerBasedRlEnv, env_ids: torch.Tensor) -> None:
  """Car 0.6 m into a random hallway, heading along it; route to a random goal; new people."""
  nv, n, dev = nav(env), len(env_ids), env.device
  k = nv.layout[env_ids]
  a = torch.randint(0, NODES * NODES, (n,), device=dev)
  nbr = nv.maps.nbr[k, a]
  d = torch.multinomial((nbr >= 0).float(), 1)[:, 0]
  b = nbr[torch.arange(n, device=dev), d]
  dvec = torch.tensor(_DIR_VEC, device=dev)[d]
  xy = nv.maps.node_xy[k, a] + 0.6 * dvec
  yaw = torch.tensor(_DIR_YAW, device=dev)[d] + 0.3 * (2 * torch.rand(n, device=dev) - 1)
  place_car(env, env_ids, xy, yaw)
  r = nv.maps.route[k, a, d]
  pts = [xy, r[:, 0], r[:, 1], nv.maps.node_xy[k, b]]
  g = _pick_goal(nv, env_ids, b, d)
  nv.arrive_dir[env_ids] = _walk_to_goal(nv, env_ids, pts, b, g, d)
  nv.goal[env_ids] = g
  _set_path(nv, env_ids, pts, xy)
  nv.goals_reached[env_ids] = 0
  nv.reached[env_ids] = False
  nv.loc_bias[env_ids] = torch.tensor([R.LOC_BIAS_M, R.LOC_BIAS_M, R.LOC_BIAS_YAW], device=dev) * (
    2 * torch.rand(n, 3, device=dev) - 1)
  nv.loc_jump[env_ids] = 0.0
  car_xy = torch.zeros(env.num_envs, 2, device=dev)
  car_xy[env_ids] = xy
  reset_walkers(env, walkers(env, nv.maps, nv.layout), env_ids, car_xy)
  R.reset_ray_noise(env, env_ids, LIDAR_RAYS)
  _write_goal_marker(env, env_ids)


# Observations (car frame).


def _noisy(env: ManagerBasedRlEnv) -> bool:
  return env.cfg.observations["actor"].enable_corruption


def route_preview(env: ManagerBasedRlEnv, noisy: bool = False) -> torch.Tensor:
  """Route points PREVIEW metres ahead of the progress point (8 x 2), and the remaining route length (1).
  With noise, the car's own pose is off by the localization error (bias + jumps), as on the real car."""
  nv, (xy, yaw) = nav(env), car_pose(env)
  pts = _points_at(nv, nv.s[:, None] + torch.tensor(PREVIEW, device=env.device))   # [N, 8, 2]
  if noisy and _noisy(env) and R.ENABLE_LOCALIZATION_NOISE:
    e = nv.loc_bias + nv.loc_jump
    xy, yaw = xy + e[:, :2], yaw + e[:, 2]
  rel = torch.stack([to_frame(pts[:, i] - xy, yaw) for i in range(len(PREVIEW))], 1)
  remain = (_remaining(nv).clamp(max=REMAIN_CLIP) / REMAIN_CLIP)[:, None]
  return torch.cat([rel.flatten(1), remain], -1)


def lidar_sectors(env: ManagerBasedRlEnv, noisy: bool = False) -> torch.Tensor:
  """Nearest return per 10 deg sector, normalized to [0, 1] (1 = nothing within LIDAR_RANGE). With noise, each raw
  ray is noisy before pooling, and the scan refreshes at the car's scan rate."""
  d = env.scene["lidar"].data.distances
  d = torch.where(d < 0, LIDAR_RANGE, d).clamp(max=LIDAR_RANGE)
  if noisy and _noisy(env) and R.ENABLE_LIDAR_NOISE:
    d = R.noisy_rays(env, d, LIDAR_RANGE)
  return d.view(env.num_envs, LIDAR_SECTORS, -1).amin(-1) / LIDAR_RANGE


def people_privileged(env: ManagerBasedRlEnv) -> torch.Tensor:
  """Critic only: each walker's position and velocity in the car frame, and whether it is active."""
  nv, (xy, yaw) = nav(env), car_pose(env)
  w = walkers(env, nv.maps, nv.layout)
  out = []
  for k in range(MAX_PEOPLE):
    act = w.active[:, k, None].float()
    out += [to_frame(w.xy[:, k] - xy, yaw).clamp(-5, 5) * act, to_frame(w.vel[:, k], yaw) * act, act]
  return torch.cat(out, -1)


# Rewards and terminations.


def route_distance(env: ManagerBasedRlEnv) -> torch.Tensor:
  nv = nav(env)
  return torch.linalg.norm(_points_at(nv, nv.s[:, None])[:, 0] - car_pose(env)[0], dim=-1)


def progress(env: ManagerBasedRlEnv) -> torch.Tensor:
  nv = nav(env)
  return ((nv.s - nv.s_prev) / (V_MAX * env.step_dt)).clamp(-1, 1)


def on_route(env: ManagerBasedRlEnv, std: float) -> torch.Tensor:
  return torch.exp(-(route_distance(env) / std) ** 2)


def goal_reached(env: ManagerBasedRlEnv) -> torch.Tensor:
  return nav(env).reached.float()


def person_clearance(env: ManagerBasedRlEnv) -> torch.Tensor:
  """Distance between the car's body and the nearest active person (circle approximations), m."""
  nv, (xy, _) = nav(env), car_pose(env)
  w = walkers(env, nv.maps, nv.layout)
  d = torch.linalg.norm(w.xy - xy[:, None], dim=-1) - CAR_RADIUS - PERSON_RADIUS
  return torch.where(w.active, d, 1e3).amin(-1)


def personal_space(env: ManagerBasedRlEnv) -> torch.Tensor:
  return ((PERSONAL_SPACE - person_clearance(env)) / PERSONAL_SPACE).clamp(0, 1)


def speed_near_people(env: ManagerBasedRlEnv) -> torch.Tensor:
  """(speed / V_MAX)^2, weighted by how far inside PERSONAL_ZONE the nearest person is (0 at 1.2 m, 1 at 0.45 m).
  Quadratic: passing slowly costs little, rushing past costs more than the progress it buys (v2, 2026-10-31,
  after v1 clipped standing people's feet at ~0.6 m/s in every car-person contact)."""
  near = ((PERSONAL_ZONE - person_clearance(env)) / (PERSONAL_ZONE - PERSONAL_SPACE)).clamp(0, 1)
  v = env.scene["car"].data.root_link_lin_vel_b[:, 0].abs() / V_MAX
  return near * v * v


def bumped(env: ManagerBasedRlEnv) -> torch.Tensor:
  return env.scene["bump"].data.found[:, 0] > 0


def hit_person(env: ManagerBasedRlEnv) -> torch.Tensor:
  return torch.stack([env.scene[f"touch{k}"].data.found[:, 0] > 0 for k in range(MAX_PEOPLE)], -1).any(-1)


def lost_route(env: ManagerBasedRlEnv, max_distance: float) -> torch.Tensor:
  return route_distance(env) > max_distance


def as_float(func):
  def f(env, **kw):
    return func(env, **kw).float()
  f.__name__ = func.__name__ + "_penalty"
  return f


# Config.


def make_mentorpi_navigate_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  actor_terms = {
    "route": ObservationTermCfg(func=route_preview, params={"noisy": True}),
    "car_motion": ObservationTermCfg(func=car_motion),
    "lidar_sectors": ObservationTermCfg(func=lidar_sectors, params={"noisy": True}),
  }
  critic_terms = {
    "route": ObservationTermCfg(func=route_preview),
    "car_motion": ObservationTermCfg(func=car_motion),
    "lidar_sectors": ObservationTermCfg(func=lidar_sectors),
    "people": ObservationTermCfg(func=people_privileged),
  }
  lidar = RayCastSensorCfg(
    name="lidar", frame=ObjRef(type="site", name="lidar", entity="car"),
    pattern=LidarPatternCfg(num_rays=LIDAR_RAYS), ray_alignment="yaw", max_distance=LIDAR_RANGE, debug_vis=play,
    include_geom_groups=(0, 1),  # group 2 = visual-only parts (people's torsos, the goal flag)
  )
  bump = ContactSensorCfg(
    name="bump", primary=ContactMatch(mode="geom", pattern="chassis", entity="car"),
    secondary=ContactMatch(mode="body", pattern="terrain"), fields=("found",), reduce="none", num_slots=1,
  )
  touch = tuple(ContactSensorCfg(
    name=f"touch{k}", primary=ContactMatch(mode="geom", pattern="chassis", entity="car"),
    secondary=ContactMatch(mode="body", pattern="person", entity=f"person{k}"), fields=("found",), reduce="none",
    num_slots=1,
  ) for k in range(MAX_PEOPLE))
  cfg = ManagerBasedRlEnvCfg(
    scene=SceneCfg(
      terrain=TerrainEntityCfg(
        terrain_type="generator",
        terrain_generator=TerrainGeneratorCfg(
          size=(PATCH_SIZE, PATCH_SIZE), num_rows=OFFICE_ROWS, num_cols=OFFICE_COLS, seed=11,
          sub_terrains={"office": OfficeTerrainCfg()}, color_scheme="height",
        ),
      ),
      entities={"car": car_cfg(), "goal": goal_marker_cfg(), **{f"person{k}": person_cfg() for k in range(MAX_PEOPLE)}},
      sensors=(lidar, bump, *touch),
      num_envs=1,
    ),
    observations={
      "actor": ObservationGroupCfg(actor_terms, enable_corruption=True),
      "critic": ObservationGroupCfg(critic_terms),
    },
    actions={"drive": AckermannActionCfg(entity_name="car", leader_step=advance_world, v_reverse=0.3,
                                         steer_max_range=(STEER_2024, STEER_CODE))},
    events={
      "reset_joints": EventTermCfg(
        func=reset_joints_by_offset, mode="reset",
        params={"position_range": (0.0, 0.0), "velocity_range": (0.0, 0.0), "asset_cfg": SceneEntityCfg("car")},
      ),
      "reset_navigate": EventTermCfg(func=reset_navigate, mode="reset"),
      **people_size_events(),
    },
    rewards={
      "progress": RewardTermCfg(func=progress, weight=2.0),
      "on_route": RewardTermCfg(func=on_route, weight=0.5, params={"std": 0.35}),
      "goal": RewardTermCfg(func=goal_reached, weight=10.0),
      "personal_space": RewardTermCfg(func=personal_space, weight=-2.0),
      "speed_near_people": RewardTermCfg(func=speed_near_people, weight=-3.0),
      "bump": RewardTermCfg(func=as_float(bumped), weight=-20.0),
      "hit_person": RewardTermCfg(func=as_float(hit_person), weight=-30.0),
      "lost": RewardTermCfg(func=as_float(lost_route), weight=-20.0, params={"max_distance": 1.5}),
      "action_rate": RewardTermCfg(func=action_rate_l2, weight=-0.05),
    },
    terminations={
      "time_out": TerminationTermCfg(func=time_out, time_out=True),
      "bump": TerminationTermCfg(func=bumped),
      "hit_person": TerminationTermCfg(func=hit_person),
      "lost_route": TerminationTermCfg(func=lost_route, params={"max_distance": 1.5}),
    },
    viewer=ViewerConfig(
      origin_type=ViewerConfig.OriginType.ASSET_BODY, entity_name="car", body_name="chassis",
      distance=3.5, elevation=55.0, azimuth=180.0,
    ),
    sim=SimulationCfg(nconmax=128, njmax=320, mujoco=MujocoCfg(timestep=0.005, integrator="implicitfast")),
    decimation=10,               # 20 Hz policy
    episode_length_s=40.0,
  )
  R.apply_domain_randomization(cfg)
  if play:
    cfg.episode_length_s = 120.0
    cfg.observations["actor"].enable_corruption = False
  return cfg


MentorPiNavigateRlCfg = RslRlOnPolicyRunnerCfg(
  actor=RslRlModelCfg(
    hidden_dims=(256, 128), activation="elu", obs_normalization=True,
    distribution_cfg={"class_name": "GaussianDistribution", "init_std": 0.6, "std_type": "scalar"},
  ),
  critic=RslRlModelCfg(hidden_dims=(256, 128), activation="elu", obs_normalization=True),
  algorithm=RslRlPpoAlgorithmCfg(
    value_loss_coef=1.0, use_clipped_value_loss=True, clip_param=0.2, entropy_coef=0.005,
    num_learning_epochs=5, num_mini_batches=4, learning_rate=1.0e-3, schedule="adaptive",
    gamma=0.99, lam=0.95, desired_kl=0.01, max_grad_norm=1.0,
  ),
  experiment_name="navigate",
  logger="tensorboard",
  save_interval=50,
  num_steps_per_env=32,
  max_iterations=1000,             # v2 fine-tunes v1 with --agent.resume for 600 more
)
