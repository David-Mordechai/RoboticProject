"""Formation task (T1, project-plan §13.4): the follower car holds a formation slot next to a Nav2-like leader, in the
office, among walking people, seeing with the car's real sensors (LiDAR + depth camera + stock person detections).

Leader: the office task's route driver, made to behave like the stock Nav2 the real leader runs: it slows in turns
(Nav2 Regulated Pure Pursuit scales speed down with curvature), stops for a person in front (Nav2 collision monitor
stop zone), and accelerates at a limited rate. Slot: an input (distance back along the leader's path, lateral offset),
drawn per episode from SLOTS; names follow Hiwonder's multi/formation_update.py (row, column; diagonal = the
two-car case of its triangle). The slot is defined along the leader's driven path, so it never cuts a corner.

The follower sees people three ways: legs in the LiDAR plane, everything low (feet, boxes) in the depth sectors, and
the stock YOLO person detections. v1 of the navigate policy clipped people's feet because the LiDAR can't see them.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

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

from robotlab.robot.mentorpi_a1_constants import STEER_2024, STEER_CODE, car_cfg, leader_cfg, person_cfg
from robotlab.tasks import camera as C
from robotlab.tasks import randomization as R
from robotlab.tasks.mdp import LidarPatternCfg, AckermannActionCfg, car_motion, car_pose, place_car, to_frame, wrap
from robotlab.tasks.mentorpi_navigate_env_cfg import LIDAR_RAYS, LIDAR_RANGE, CAR_RADIUS, PERSON_RADIUS, \
  PERSONAL_SPACE, lidar_sectors
from robotlab.tasks.mentorpi_office_env_cfg import LEADER_SPEED, LEADER_STEER, advance_leader, leader_relative, \
  office_leader, reset_office
from robotlab.tasks.office_terrain import OCC_N, OCC_RES, PATCH_SIZE, OfficeTerrainCfg
from robotlab.tasks.people import MAX_PEOPLE, people_size_events, reset_walkers, step_walkers, walkers

if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv

# Slots: (metres back along the leader's path, lateral offset, + = left). Our formation design, not a physical spec.
SLOTS = {
  "column": (0.6, 0.0),
  "diagonal_left": (0.45, 0.35), "diagonal_right": (0.45, -0.35),
  "row_left": (0.0, 0.45), "row_right": (0.0, -0.45),
}
SLOT_NORM = (0.6, 0.45)
TRAIL_DS = 0.02                  # m between stored leader path points
TRAIL_N = 64                     # 1.28 m of leader path
LEADER_ACCEL = 0.5               # m/s^2: ESTIMATE (Nav2 velocity smoother limits on the A1 unmeasured)
LEADER_TURN_SLOW = 0.5           # speed x (1 - 0.5 |steer| / max): Nav2 RPP curvature regulation, factor ESTIMATE
LEADER_STOP_DIST = 0.8           # m: person ahead of the leader -> stop (Nav2 collision monitor zone, ESTIMATE)
LOST_DIST = 1.5
# Leader cruise speed per episode. Hiwonder's stock Nav2 caps speed at 0.32 m/s (velocity smoother max_velocity,
# navigation/config/nav2_params.yaml); its TEB controller allows 0.6 m/s (nav2_controller_teb.yaml max_vel_x). The range
# covers stock and a raised limit, so the follower can keep up if we raise it on the real cars.
LEADER_CRUISE = (0.2, 0.6)
# Adaptive slot: where a side slot has no room, the rewarded target slides behind the leader (toward the column slot)
# and back out when the hallway opens. Room = free space beside the leader's path on the slot side (ground-truth
# occupancy of walls and clutter) minus SIDE_CLEAR. The policy only sees the commanded slot and its sensors, so it
# learns to judge the room itself, as it must on the real car (no map needed).
SIDE_CLEAR = 0.25                # m: car half-diagonal 0.133 (213 x 159 mm, research/01 §2) + 0.12 margin (our design)
SIDE_ALONG = (-0.15, 0.0, 0.15)  # m along the path: the car is 0.213 m long, so check room over its whole length
ADAPT_RATE = 0.5                 # m/s: how fast the adapted slot moves (no jumps)
COLUMN_BACK = SLOTS["column"][0]
# Single file through intersections: near a hallway crossing the leader may turn, and the inner car of a side-by-side
# pair would need a turn radius below the A1's minimum (~0.30 m measured in sim at 29 deg, research/01); people also
# cross there. Side slots fall back while the leader is within this distance of an intersection (our design rule).
NODE_SINGLE_FILE = 0.9           # m: the leader turns from 0.45 m before to 0.55 m past a crossing centre (office task)
OFFICE_ROWS = OFFICE_COLS = 4


@dataclass
class Formation:
  slot: torch.Tensor       # [N, 2] (back, lateral)
  cruise: torch.Tensor     # [N] leader cruise speed
  trail: torch.Tensor      # [N, TRAIL_N, 3] leader path (x, y, yaw), distance-spaced ring buffer
  head: torch.Tensor       # [N] index of the newest point
  last: torch.Tensor       # [N, 2] leader xy when the newest point was stored
  lat_eff: torch.Tensor    # [N] adapted lateral offset (same sign as the slot's)


def formation(env: ManagerBasedRlEnv) -> Formation:
  if not hasattr(env, "_robotlab_formation"):
    n, dev = env.num_envs, env.device
    z = lambda *s, dt=torch.float32: torch.zeros(n, *s, device=dev, dtype=dt)
    env._robotlab_formation = Formation(z(2), z(), z(TRAIL_N, 3), z(dt=torch.long), z(2), z())
  return env._robotlab_formation


def path_point(env: ManagerBasedRlEnv, back: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
  """Leader path point `back` metres behind the leader (back [N] or [N, M]): xy, yaw."""
  f, s = formation(env), office_leader(env)
  b = back if back.dim() == 2 else back[:, None]
  # Distance from the leader to its newest stored point, then whole steps of TRAIL_DS.
  lead = torch.linalg.norm(s.xy - f.last, dim=-1)[:, None]
  k = ((b - lead) / TRAIL_DS).clamp(0, TRAIL_N - 2)
  k0 = k.floor().long()
  u = (k - k0)[..., None]
  idx0 = (f.head[:, None] - k0) % TRAIL_N
  idx1 = (idx0 - 1) % TRAIL_N
  g = lambda i: torch.gather(f.trail, 1, i[..., None].expand(*i.shape, 3))
  p0, p1 = g(idx0), g(idx1)
  xy = p0[..., :2] + u * (p1[..., :2] - p0[..., :2])
  yaw = p0[..., 2] + u[..., 0] * wrap(p1[..., 2] - p0[..., 2])
  # Within the first stored step, interpolate from the leader itself.
  near = b < lead
  lead_pose = torch.cat([s.xy, s.yaw[:, None]], -1)[:, None].expand(-1, b.shape[1], -1)
  w = (b / lead.clamp(min=1e-6))[..., None]
  near_xy = lead_pose[..., :2] + w * (g(f.head[:, None].expand_as(b))[..., :2] - lead_pose[..., :2])
  xy = torch.where(near[..., None], near_xy, xy)
  yaw = torch.where(near, lead_pose[..., 2], yaw)
  if back.dim() == 1:
    return xy[:, 0], yaw[:, 0]
  return xy, yaw


def slot_target(env: ManagerBasedRlEnv) -> tuple[torch.Tensor, torch.Tensor]:
  """The commanded slot (what the follower is told): xy, yaw."""
  f = formation(env)
  xy, yaw = path_point(env, f.slot[:, 0])
  left = torch.stack([-torch.sin(yaw), torch.cos(yaw)], -1)
  return xy + f.slot[:, 1:2] * left, yaw


def _adapted_back(f: Formation) -> torch.Tensor:
  lat = f.slot[:, 1].abs()
  frac = torch.where(lat > 1e-6, f.lat_eff.abs() / lat.clamp(min=1e-6), 1.0)
  return f.slot[:, 0] + (1 - frac) * (COLUMN_BACK - f.slot[:, 0])


def adapted_target(env: ManagerBasedRlEnv) -> tuple[torch.Tensor, torch.Tensor]:
  """The slot the follower is rewarded for: the commanded one, slid behind the leader where there is no room."""
  f = formation(env)
  xy, yaw = path_point(env, _adapted_back(f))
  left = torch.stack([-torch.sin(yaw), torch.cos(yaw)], -1)
  return xy + f.lat_eff[:, None] * left, yaw


def side_room(env: ManagerBasedRlEnv) -> torch.Tensor:
  """Free distance beside the leader's path around the commanded slot's distance back (over the car's length), on the
  slot's side, from the office occupancy grid (ground truth), up to |lateral| + SIDE_CLEAR."""
  f, s = formation(env), office_leader(env)
  side = torch.sign(f.slot[:, 1])
  reach = f.slot[:, 1].abs() + SIDE_CLEAR
  steps = torch.arange(0, int(0.45 / 0.025) + int(SIDE_CLEAR / 0.025) + 2, device=env.device) * 0.025   # [S]
  room = torch.full((env.num_envs,), 1e3, device=env.device)
  for along in SIDE_ALONG:
    xy, yaw = path_point(env, (f.slot[:, 0] - along).clamp(min=0.0))
    left = torch.stack([-torch.sin(yaw), torch.cos(yaw)], -1) * side[:, None]
    pts = xy[:, None] + steps[None, :, None] * left[:, None]                                                # [N, S, 2]
    ij = (pts / OCC_RES).long().clamp(0, OCC_N - 1)
    occ = s.maps.occ[s.layout[:, None].expand(-1, len(steps)), ij[..., 0], ij[..., 1]]
    occ = occ | (steps[None] > reach[:, None])                       # nothing beyond what the slot needs
    room = torch.minimum(room, torch.where(occ, steps[None].expand_as(occ), 1e3).amin(-1))
  return room


def update_adaptive_slot(env: ManagerBasedRlEnv, instant: torch.Tensor | None = None) -> None:
  f = formation(env)
  lat = f.slot[:, 1].abs()
  s = office_leader(env)
  near_node = torch.minimum(torch.linalg.norm(s.maps.node_xy[s.layout, s.node] - s.xy, dim=-1),
                            torch.linalg.norm(s.maps.node_xy[s.layout, s.prev] - s.xy, dim=-1)) < NODE_SINGLE_FILE
  cap = torch.where(near_node, 0.0, (side_room(env) - SIDE_CLEAR).clamp(min=0.0))
  want = torch.minimum(lat, cap)
  cur = f.lat_eff.abs()
  step = ADAPT_RATE * env.step_dt
  new = cur + (want - cur).clamp(-step, step)
  if instant is not None:
    new = torch.where(instant, want, new)
  f.lat_eff = torch.sign(f.slot[:, 1]) * new


# Per policy step, before physics: the Nav2-like leader, its path record, the people.


def leader_step(env: ManagerBasedRlEnv) -> None:
  s, f, dt = office_leader(env), formation(env), env.step_dt
  w = walkers(env, s.maps, s.layout)
  heading = torch.stack([torch.cos(s.yaw), torch.sin(s.yaw)], -1)
  rel = w.xy - s.xy[:, None]
  d = torch.linalg.norm(rel, dim=-1)
  person_ahead = (w.active & (d < LEADER_STOP_DIST) & ((rel * heading[:, None]).sum(-1) > 0.5 * d)).any(-1)
  target = f.cruise * (1 - LEADER_TURN_SLOW * s.steer.abs() / LEADER_STEER)
  target = torch.where(person_ahead, 0.0, target)
  s.speed = s.speed + (target - s.speed).clamp(-LEADER_ACCEL * dt, LEADER_ACCEL * dt)
  advance_leader(env)
  moved = torch.linalg.norm(s.xy - f.last, dim=-1) >= TRAIL_DS
  if moved.any():
    f.head = torch.where(moved, (f.head + 1) % TRAIL_N, f.head)
    ar = torch.arange(env.num_envs, device=env.device)
    new = torch.cat([s.xy, s.yaw[:, None]], -1)
    f.trail[ar, f.head] = torch.where(moved[:, None], new, f.trail[ar, f.head])
    f.last = torch.where(moved[:, None], s.xy, f.last)
  update_adaptive_slot(env)
  car_xy, _ = car_pose(env)
  step_walkers(env, w, torch.stack([car_xy, s.xy], 1))


# Events.


def reset_formation(env: ManagerBasedRlEnv, env_ids: torch.Tensor) -> None:
  """After reset_office (leader placed, straight hallway ahead): slot, cruise speed, a straight path record behind
  the leader, the follower near its slot, people, sensor state."""
  s, f, n, dev = office_leader(env), formation(env), len(env_ids), env.device
  slots = torch.tensor(list(SLOTS.values()), device=dev)
  f.slot[env_ids] = slots[torch.randint(0, len(slots), (n,), device=dev)]
  f.cruise[env_ids] = LEADER_CRUISE[0] + (LEADER_CRUISE[1] - LEADER_CRUISE[0]) * torch.rand(n, device=dev)
  s.speed[env_ids] = 0.0                                              # Nav2 starts from rest
  xy, yaw = s.xy[env_ids], s.yaw[env_ids]
  back = torch.stack([-torch.cos(yaw), -torch.sin(yaw)], -1)
  steps = TRAIL_DS * torch.arange(TRAIL_N, device=dev, dtype=torch.float32)          # 0 = newest
  pts = xy[:, None] + steps[None, :, None] * back[:, None]
  f.head[env_ids] = TRAIL_N - 1
  order = (TRAIL_N - 1 - torch.arange(TRAIL_N, device=dev))                          # ring index of each age
  f.trail[env_ids] = torch.cat([pts, yaw[:, None, None].expand(n, TRAIL_N, 1)], -1)[:, order.argsort()]
  f.last[env_ids] = xy
  mask = torch.zeros(env.num_envs, dtype=torch.bool, device=dev)
  mask[env_ids] = True
  f.lat_eff[env_ids] = f.slot[env_ids, 1]
  update_adaptive_slot(env, instant=mask)                            # start in a slot that fits
  t_xy, t_yaw = adapted_target(env)
  r = 0.15 * torch.rand(n, device=dev)
  phi = 2 * math.pi * torch.rand(n, device=dev)
  start = t_xy[env_ids] + torch.stack([r * torch.cos(phi), r * torch.sin(phi)], -1)
  place_car(env, env_ids, start, t_yaw[env_ids] + 0.3 * (2 * torch.rand(n, device=dev) - 1))
  car_xy = torch.zeros(env.num_envs, 2, device=dev)
  car_xy[env_ids] = start
  w = walkers(env, s.maps, s.layout)
  reset_walkers(env, w, env_ids, car_xy, keep_clear=1.5)
  near_leader = torch.linalg.norm(w.xy[env_ids] - xy[:, None], dim=-1) < 1.5
  w.active[env_ids] &= ~near_leader
  R.reset_ray_noise(env, env_ids, LIDAR_RAYS)
  C.reset_camera(env, env_ids)


# Observations (car frame).


def slot_error(env: ManagerBasedRlEnv) -> torch.Tensor:
  xy, yaw = car_pose(env)
  t_xy, t_yaw = slot_target(env)
  e = wrap(t_yaw - yaw)
  return torch.cat([to_frame(t_xy - xy, yaw), torch.sin(e)[:, None], torch.cos(e)[:, None]], -1)


def path_preview(env: ManagerBasedRlEnv) -> torch.Tensor:
  """Two points of the leader's recent path (0.4 and 0.2 m behind it): shows a turn the follower is about to take."""
  xy, yaw = car_pose(env)
  pts, _ = path_point(env, torch.tensor([[0.4, 0.2]], device=env.device).expand(env.num_envs, 2))
  return torch.cat([to_frame(pts[:, i] - xy, yaw) for i in range(2)], -1)


def slot_command(env: ManagerBasedRlEnv) -> torch.Tensor:
  return formation(env).slot / torch.tensor(SLOT_NORM, device=env.device)


def depth(env: ManagerBasedRlEnv, noisy: bool = False) -> torch.Tensor:
  return C.depth_sectors(env, noisy)


def people_camera(env: ManagerBasedRlEnv, noisy: bool = False) -> torch.Tensor:
  s = office_leader(env)
  w = walkers(env, s.maps, s.layout)
  xy, yaw = car_pose(env)
  lid = lidar_sectors(env) * LIDAR_RANGE
  return C.person_detections(env, w.xy, w.active, xy, yaw, lid, noisy)


def people_privileged(env: ManagerBasedRlEnv) -> torch.Tensor:
  """Critic only: every walker's position and velocity in the car frame, and whether it is active."""
  s = office_leader(env)
  w = walkers(env, s.maps, s.layout)
  xy, yaw = car_pose(env)
  out = []
  for k in range(MAX_PEOPLE):
    act = w.active[:, k, None].float()
    out += [to_frame(w.xy[:, k] - xy, yaw).clamp(-5, 5) * act, to_frame(w.vel[:, k], yaw) * act, act]
  return torch.cat(out, -1)


# Rewards and terminations.


def slot_distance(env: ManagerBasedRlEnv) -> torch.Tensor:
  return torch.linalg.norm(adapted_target(env)[0] - car_pose(env)[0], dim=-1)


def track_slot(env: ManagerBasedRlEnv, std: float) -> torch.Tensor:
  return torch.exp(-(slot_distance(env) / std) ** 2)


def heading_match(env: ManagerBasedRlEnv) -> torch.Tensor:
  return torch.cos(wrap(adapted_target(env)[1] - car_pose(env)[1]))


def too_close(env: ManagerBasedRlEnv, min_gap: float) -> torch.Tensor:
  return (torch.linalg.norm(office_leader(env).xy - car_pose(env)[0], dim=-1) < min_gap).float()


def person_clearance(env: ManagerBasedRlEnv) -> torch.Tensor:
  s = office_leader(env)
  w = walkers(env, s.maps, s.layout)
  d = torch.linalg.norm(w.xy - car_pose(env)[0][:, None], dim=-1) - CAR_RADIUS - PERSON_RADIUS
  return torch.where(w.active, d, 1e3).amin(-1)


def personal_space(env: ManagerBasedRlEnv) -> torch.Tensor:
  return ((PERSONAL_SPACE - person_clearance(env)) / PERSONAL_SPACE).clamp(0, 1)


def bumped(env: ManagerBasedRlEnv) -> torch.Tensor:
  return env.scene["bump"].data.found[:, 0] > 0


def hit_person(env: ManagerBasedRlEnv) -> torch.Tensor:
  return torch.stack([env.scene[f"touch{k}"].data.found[:, 0] > 0 for k in range(MAX_PEOPLE)], -1).any(-1)


def lost_slot(env: ManagerBasedRlEnv, max_distance: float) -> torch.Tensor:
  return slot_distance(env) > max_distance


def as_float(func):
  def f(env, **kw):
    return func(env, **kw).float()
  f.__name__ = func.__name__ + "_penalty"
  return f


# Config.


def make_mentorpi_formation_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  actor_terms = {
    "slot_error": ObservationTermCfg(func=slot_error),
    "path_preview": ObservationTermCfg(func=path_preview),
    "slot": ObservationTermCfg(func=slot_command),
    "car_motion": ObservationTermCfg(func=car_motion),
    "leader": ObservationTermCfg(func=leader_relative),
    "lidar_sectors": ObservationTermCfg(func=lidar_sectors, params={"noisy": True}),
    "depth": ObservationTermCfg(func=depth, params={"noisy": True}),
    "people_camera": ObservationTermCfg(func=people_camera, params={"noisy": True}),
  }
  critic_terms = {
    "slot_error": ObservationTermCfg(func=slot_error),
    "path_preview": ObservationTermCfg(func=path_preview),
    "slot": ObservationTermCfg(func=slot_command),
    "car_motion": ObservationTermCfg(func=car_motion),
    "leader": ObservationTermCfg(func=leader_relative),
    "lidar_sectors": ObservationTermCfg(func=lidar_sectors),
    "depth": ObservationTermCfg(func=depth),
    "people_camera": ObservationTermCfg(func=people_camera),
    "people": ObservationTermCfg(func=people_privileged),
  }
  lidar = RayCastSensorCfg(
    name="lidar", frame=ObjRef(type="site", name="lidar", entity="car"),
    pattern=LidarPatternCfg(num_rays=LIDAR_RAYS), ray_alignment="yaw", max_distance=LIDAR_RANGE, debug_vis=play,
    include_geom_groups=(0, 1),
  )
  depth_cam = RayCastSensorCfg(
    name="depth", frame=ObjRef(type="site", name="depth_cam", entity="car"),
    pattern=C.DepthFanPatternCfg(), ray_alignment="base", max_distance=C.DEPTH_MAX, debug_vis=False,
    include_geom_groups=(0, 1),
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
          size=(PATCH_SIZE, PATCH_SIZE), num_rows=OFFICE_ROWS, num_cols=OFFICE_COLS, seed=13,
          sub_terrains={"office": OfficeTerrainCfg()}, color_scheme="height",
        ),
      ),
      entities={"car": car_cfg(), "leader": leader_cfg(), **{f"person{k}": person_cfg() for k in range(MAX_PEOPLE)}},
      sensors=(lidar, depth_cam, bump, *touch),
      num_envs=1,
    ),
    observations={
      "actor": ObservationGroupCfg(actor_terms, enable_corruption=True),
      "critic": ObservationGroupCfg(critic_terms),
    },
    actions={"drive": AckermannActionCfg(entity_name="car", leader_step=leader_step, v_reverse=0.3,
                                         steer_max_range=(STEER_2024, STEER_CODE))},
    events={
      "reset_joints": EventTermCfg(
        func=reset_joints_by_offset, mode="reset",
        params={"position_range": (0.0, 0.0), "velocity_range": (0.0, 0.0), "asset_cfg": SceneEntityCfg("car")},
      ),
      "reset_office": EventTermCfg(func=reset_office, mode="reset"),
      "reset_formation": EventTermCfg(func=reset_formation, mode="reset"),
      **people_size_events(),
    },
    rewards={
      "track_slot": RewardTermCfg(func=track_slot, weight=2.0, params={"std": 0.25}),
      "track_slot_coarse": RewardTermCfg(func=track_slot, weight=1.0, params={"std": 1.0}),
      "heading": RewardTermCfg(func=heading_match, weight=0.3),
      "too_close": RewardTermCfg(func=too_close, weight=-10.0, params={"min_gap": 0.3}),
      "personal_space": RewardTermCfg(func=personal_space, weight=-2.0),
      "bump": RewardTermCfg(func=as_float(bumped), weight=-20.0),
      "hit_person": RewardTermCfg(func=as_float(hit_person), weight=-30.0),
      "lost": RewardTermCfg(func=as_float(lost_slot), weight=-20.0, params={"max_distance": LOST_DIST}),
      "action_rate": RewardTermCfg(func=action_rate_l2, weight=-0.05),
    },
    terminations={
      "time_out": TerminationTermCfg(func=time_out, time_out=True),
      "bump": TerminationTermCfg(func=bumped),
      "hit_person": TerminationTermCfg(func=hit_person),
      "lost_slot": TerminationTermCfg(func=lost_slot, params={"max_distance": LOST_DIST}),
    },
    viewer=ViewerConfig(
      origin_type=ViewerConfig.OriginType.ASSET_BODY, entity_name="car", body_name="chassis",
      distance=3.0, elevation=55.0, azimuth=180.0,
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


MentorPiFormationRlCfg = RslRlOnPolicyRunnerCfg(
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
  experiment_name="formation",
  logger="tensorboard",
  save_interval=50,
  num_steps_per_env=32,
  max_iterations=600,            # ~45 min at ~7.8k env-steps/s on the RTX 2060; T1b fine-tunes T1 for 400 more
)
