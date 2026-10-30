"""Office task: a MentorPi A1 follows a scripted leader car through office hallways with obstacles.

The leader drives a random route through the hallway graph (pure pursuit, kinematic, no U-turns),
weaving around clutter on the lane the generator left free. The learning car must stay on the
leader's trail 0.6 m behind it (path distance, not a rigid slot, so the target never cuts a corner
through a wall) and must not touch walls, obstacles or the leader.

The car senses the office the way the real one will: a 36-ray 360 deg scan in the LiDAR plane, plus
its own motion and the leader's relative pose (shared over the network on the real cars).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

import mujoco
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
from mjlab.utils.noise import UniformNoiseCfg
from mjlab.viewer import ViewerConfig

from robotlab.tasks.common import (
  STEER_MAX, WHEELBASE, AckermannActionCfg, car_cfg, car_motion, car_pose, leader_cfg, place_car, to_frame,
  wrap, write_leader_pose,
)
from robotlab.tasks.office.office_terrain import NODES, PATCH_SIZE, LayoutTensors, OfficeTerrainCfg, layout_tensors

if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv

GAP = 0.6                        # m of path between leader and follower
LEADER_SPEED = (0.25, 0.45)      # m/s, drawn per episode
LIDAR_RAYS = 36                  # every 10 deg (the LD19 gives ~450 points per turn; downsampled)
LIDAR_RANGE = 3.0                # m, clipped
TRAIL_LEN = 64                   # policy steps of leader history kept (>= GAP / (min speed * dt))
TURN_EXIT = 0.35                 # m past the intersection centre: first point of every segment (tight turns)
SWITCH_DIST = (0.2, 0.35, 0.35, 0.2)   # m, per stage: move on to the next route point within this distance
OFFICE_ROWS = OFFICE_COLS = 4    # 16 different office floors per scene

_DIR_VEC = ((1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0))
_DIR_YAW = (0.0, math.pi, math.pi / 2, -math.pi / 2)


# LiDAR: horizontal rays all around the scan centre (mjlab's patterns are all parallel rays).


@dataclass
class LidarPatternCfg:
  num_rays: int = LIDAR_RAYS

  def generate_rays(self, mj_model: mujoco.MjModel | None, device: str) -> tuple[torch.Tensor, torch.Tensor]:
    del mj_model
    a = torch.arange(self.num_rays, device=device, dtype=torch.float32) * (2 * math.pi / self.num_rays)
    dirs = torch.stack([torch.cos(a), torch.sin(a), torch.zeros_like(a)], -1)
    return torch.zeros(self.num_rays, 3, device=device), dirs


# Leader: route state per env, advanced once per policy step.


@dataclass
class OfficeLeader:
  maps: LayoutTensors
  layout: torch.Tensor     # [N] office floor index of each env
  prev: torch.Tensor       # [N] node the current hallway segment starts at
  node: torch.Tensor       # [N] node it is driving to
  dir: torch.Tensor        # [N] direction prev -> node
  stage: torch.Tensor      # [N] 0 = turn exit, 1, 2 = route points in the segment, 3 = the next node
  xy: torch.Tensor         # [N, 2] env frame
  yaw: torch.Tensor        # [N]
  steer: torch.Tensor      # [N]
  speed: torch.Tensor      # [N]
  trail: torch.Tensor      # [N, TRAIL_LEN, 3] past (x, y, yaw), ring buffer
  lag: torch.Tensor        # [N] trail steps that make up GAP at this speed
  head: int = 0


def office_leader(env: ManagerBasedRlEnv) -> OfficeLeader:
  if not hasattr(env, "_robotlab_office"):
    terrain, n, dev = env.scene.terrain, env.num_envs, env.device
    num_cols = terrain.terrain_origins.shape[1]
    maps = layout_tensors(terrain.terrain_origins.shape[0] * num_cols, dev)
    z = lambda *shape, dt=torch.float32: torch.zeros(n, *shape, device=dev, dtype=dt)
    zl = lambda: z(dt=torch.long)
    env._robotlab_office = OfficeLeader(
      maps, terrain.terrain_levels * num_cols + terrain.terrain_types, zl(), zl(), zl(), zl(),
      z(2), z(), z(), z(), z(TRAIL_LEN, 3), zl(),
    )
  return env._robotlab_office


def _route_target(s: OfficeLeader) -> torch.Tensor:
  route = s.maps.route[s.layout, s.prev, s.dir]                    # [N, 2, 2]
  point = route[torch.arange(len(s.stage), device=s.stage.device), (s.stage - 1).clamp(0, 1)]
  exit_ = s.maps.node_xy[s.layout, s.prev] + TURN_EXIT * torch.tensor(_DIR_VEC, device=s.stage.device)[s.dir]
  node = s.maps.node_xy[s.layout, s.node]
  return torch.where((s.stage == 0)[:, None], exit_, torch.where((s.stage == 3)[:, None], node, point))


def advance_leader(env: ManagerBasedRlEnv) -> None:
  s, dt = office_leader(env), env.step_dt
  ar = torch.arange(env.num_envs, device=env.device)

  # Route progress; at a node pick a random onward hallway (never straight back).
  switch = torch.tensor(SWITCH_DIST, device=env.device)[s.stage]
  stage = s.stage + (torch.linalg.norm(_route_target(s) - s.xy, dim=-1) < switch).long()
  at_node = stage == 4
  nbr = s.maps.nbr[s.layout, s.node]
  onward = nbr >= 0
  onward[ar, s.dir ^ 1] = False
  onward = torch.where(onward.any(-1, keepdim=True), onward, nbr >= 0)
  new_dir = torch.multinomial(onward.float(), 1).squeeze(-1)
  s.prev = torch.where(at_node, s.node, s.prev)
  s.node = torch.where(at_node, nbr[ar, new_dir], s.node)
  s.dir = torch.where(at_node, new_dir, s.dir)
  s.stage = torch.where(at_node, 0, stage)

  # Pure pursuit on a kinematic bicycle.
  d = _route_target(s) - s.xy
  alpha = wrap(torch.atan2(d[:, 1], d[:, 0]) - s.yaw)
  look = torch.linalg.norm(d, dim=-1).clamp(min=0.3)
  s.steer = torch.atan(2 * WHEELBASE * torch.sin(alpha) / look).clamp(-0.9 * STEER_MAX, 0.9 * STEER_MAX)
  s.yaw = wrap(s.yaw + s.speed / WHEELBASE * torch.tan(s.steer) * dt)
  s.xy = s.xy + (s.speed * dt)[:, None] * torch.stack([torch.cos(s.yaw), torch.sin(s.yaw)], -1)

  s.head = (s.head + 1) % TRAIL_LEN
  s.trail[:, s.head] = torch.cat([s.xy, s.yaw[:, None]], -1)
  write_leader_pose(env, ar, s.xy, s.yaw)


def trail_point(env: ManagerBasedRlEnv, frac: float = 1.0) -> tuple[torch.Tensor, torch.Tensor]:
  """Where the leader was frac * GAP metres of path ago: xy (env frame) and yaw."""
  s = office_leader(env)
  lag = torch.round(s.lag.float() * frac).long()
  p = s.trail[torch.arange(env.num_envs, device=env.device), (s.head - lag) % TRAIL_LEN]
  return p[:, :2], p[:, 2]


def trail_target(env: ManagerBasedRlEnv) -> tuple[torch.Tensor, torch.Tensor]:
  """The follower's target: GAP metres of path behind the leader."""
  return trail_point(env, 1.0)


# Events.


def reset_office(env: ManagerBasedRlEnv, env_ids: torch.Tensor) -> None:
  """Leader starts 0.9 m into a random hallway; its trail behind it is a straight line;
  the follower starts near the trail target."""
  s, n, dev = office_leader(env), len(env_ids), env.device
  k = s.layout[env_ids]
  a = torch.randint(0, NODES * NODES, (n,), device=dev)
  nbr = s.maps.nbr[k, a]
  d = torch.multinomial((nbr >= 0).float(), 1).squeeze(-1)
  dvec = torch.tensor(_DIR_VEC, device=dev)[d]
  xy = s.maps.node_xy[k, a] + 0.9 * dvec
  yaw = torch.tensor(_DIR_YAW, device=dev)[d]
  speed = LEADER_SPEED[0] + (LEADER_SPEED[1] - LEADER_SPEED[0]) * torch.rand(n, device=dev)

  s.prev[env_ids], s.node[env_ids], s.dir[env_ids] = a, nbr[torch.arange(n, device=dev), d], d
  s.stage[env_ids] = 1                                   # already past the turn exit
  s.xy[env_ids], s.yaw[env_ids], s.steer[env_ids], s.speed[env_ids] = xy, yaw, 0.0, speed
  s.lag[env_ids] = torch.round(GAP / (speed * env.step_dt)).long().clamp(max=TRAIL_LEN - 1)
  age = ((s.head - torch.arange(TRAIL_LEN, device=dev)) % TRAIL_LEN).float()          # steps ago
  back = xy[:, None] - dvec[:, None] * (age[None, :, None] * (speed * env.step_dt)[:, None, None])
  s.trail[env_ids] = torch.cat([back, yaw[:, None, None].expand(n, TRAIL_LEN, 1)], -1)
  write_leader_pose(env, env_ids, xy, yaw)

  t_xy, t_yaw = trail_target(env)
  r = 0.2 * torch.rand(n, device=dev)
  phi = 2 * math.pi * torch.rand(n, device=dev)
  start = t_xy[env_ids] + torch.stack([r * torch.cos(phi), r * torch.sin(phi)], -1)
  place_car(env, env_ids, start, t_yaw[env_ids] + 0.4 * (2 * torch.rand(n, device=dev) - 1))


# Observations (car frame).


def trail_error(env: ManagerBasedRlEnv) -> torch.Tensor:
  """Trail target in the car frame (2) and heading error as sin/cos (2)."""
  xy, yaw = car_pose(env)
  t_xy, t_yaw = trail_target(env)
  e = wrap(t_yaw - yaw)
  return torch.cat([to_frame(t_xy - xy, yaw), torch.sin(e)[:, None], torch.cos(e)[:, None]], -1)


def trail_preview(env: ManagerBasedRlEnv) -> torch.Tensor:
  """The trail between target and leader (2/3 and 1/3 of GAP behind it), in the car frame: shows a turn
  coming before the car reaches it."""
  xy, yaw = car_pose(env)
  return torch.cat([to_frame(trail_point(env, f)[0] - xy, yaw) for f in (2 / 3, 1 / 3)], -1)


def leader_relative(env: ManagerBasedRlEnv) -> torch.Tensor:
  """Leader position (2) and heading sin/cos (2) in the car frame, its speed and steering."""
  xy, yaw = car_pose(env)
  s = office_leader(env)
  e = wrap(s.yaw - yaw)
  return torch.cat([to_frame(s.xy - xy, yaw), torch.sin(e)[:, None], torch.cos(e)[:, None],
                    s.speed[:, None], (s.steer / STEER_MAX)[:, None]], -1)


def lidar_scan(env: ManagerBasedRlEnv) -> torch.Tensor:
  """Range per ray, normalized to [0, 1]; no return = max range."""
  d = env.scene["lidar"].data.distances
  return torch.where(d < 0, LIDAR_RANGE, d).clamp(max=LIDAR_RANGE) / LIDAR_RANGE


# Rewards and terminations.


def trail_distance(env: ManagerBasedRlEnv) -> torch.Tensor:
  xy, _ = car_pose(env)
  return torch.linalg.norm(trail_target(env)[0] - xy, dim=-1)


def track_trail(env: ManagerBasedRlEnv, std: float) -> torch.Tensor:
  return torch.exp(-(trail_distance(env) / std) ** 2)


def heading_match(env: ManagerBasedRlEnv) -> torch.Tensor:
  _, yaw = car_pose(env)
  return torch.cos(wrap(trail_target(env)[1] - yaw))


def too_close(env: ManagerBasedRlEnv, min_gap: float) -> torch.Tensor:
  xy, _ = car_pose(env)
  return (torch.linalg.norm(office_leader(env).xy - xy, dim=-1) < min_gap).float()


def bumped(env: ManagerBasedRlEnv) -> torch.Tensor:
  """Chassis touches a wall or an obstacle."""
  return env.scene["bump"].data.found[:, 0] > 0


def bump_penalty(env: ManagerBasedRlEnv) -> torch.Tensor:
  return bumped(env).float()


def lost_penalty(env: ManagerBasedRlEnv, max_distance: float) -> torch.Tensor:
  """Giving up must cost as much as crashing, or stopping becomes a free way out of a hard corner."""
  return lost_trail(env, max_distance).float()


def lost_trail(env: ManagerBasedRlEnv, max_distance: float) -> torch.Tensor:
  return trail_distance(env) > max_distance


# Config.


def office_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  noise = UniformNoiseCfg(n_min=-0.02, n_max=0.02)
  actor_terms = {
    "trail_error": ObservationTermCfg(func=trail_error, noise=noise),
    "trail_preview": ObservationTermCfg(func=trail_preview, noise=noise),
    "car_motion": ObservationTermCfg(func=car_motion, noise=noise),
    "leader": ObservationTermCfg(func=leader_relative, noise=noise),
    "lidar": ObservationTermCfg(func=lidar_scan, noise=noise),
  }
  lidar = RayCastSensorCfg(
    name="lidar",
    frame=ObjRef(type="site", name="lidar", entity="car"),
    pattern=LidarPatternCfg(),
    ray_alignment="yaw",
    max_distance=LIDAR_RANGE,
    debug_vis=play,
  )
  bump = ContactSensorCfg(
    name="bump",
    primary=ContactMatch(mode="geom", pattern="chassis", entity="car"),
    secondary=ContactMatch(mode="body", pattern="terrain"),
    fields=("found",),
    reduce="none",
    num_slots=1,
  )
  cfg = ManagerBasedRlEnvCfg(
    scene=SceneCfg(
      terrain=TerrainEntityCfg(
        terrain_type="generator",
        terrain_generator=TerrainGeneratorCfg(
          size=(PATCH_SIZE, PATCH_SIZE), num_rows=OFFICE_ROWS, num_cols=OFFICE_COLS, seed=7,
          # "height" makes the generator use each geom's own TerrainGeometry.color ("none" greys everything).
          sub_terrains={"office": OfficeTerrainCfg()}, color_scheme="height",
        ),
      ),
      entities={"car": car_cfg(), "leader": leader_cfg()},
      sensors=(lidar, bump),
      num_envs=1,
    ),
    observations={
      "actor": ObservationGroupCfg(actor_terms, enable_corruption=True),
      "critic": ObservationGroupCfg({**actor_terms}),
    },
    actions={"drive": AckermannActionCfg(entity_name="car", leader_step=advance_leader, v_reverse=0.3)},
    events={
      "reset_joints": EventTermCfg(
        func=reset_joints_by_offset, mode="reset",
        params={"position_range": (0.0, 0.0), "velocity_range": (0.0, 0.0), "asset_cfg": SceneEntityCfg("car")},
      ),
      "reset_office": EventTermCfg(func=reset_office, mode="reset"),
    },
    rewards={
      "track_trail": RewardTermCfg(func=track_trail, weight=2.0, params={"std": 0.25}),
      "track_trail_coarse": RewardTermCfg(func=track_trail, weight=1.0, params={"std": 1.0}),   # pays for recovering
      "heading": RewardTermCfg(func=heading_match, weight=0.3),
      "too_close": RewardTermCfg(func=too_close, weight=-10.0, params={"min_gap": 0.3}),
      "bump": RewardTermCfg(func=bump_penalty, weight=-20.0),
      "lost": RewardTermCfg(func=lost_penalty, weight=-20.0, params={"max_distance": 1.5}),
      "action_rate": RewardTermCfg(func=action_rate_l2, weight=-0.05),
    },
    terminations={
      "time_out": TerminationTermCfg(func=time_out, time_out=True),
      "bump": TerminationTermCfg(func=bumped),
      "lost_trail": TerminationTermCfg(func=lost_trail, params={"max_distance": 1.5}),
    },
    viewer=ViewerConfig(
      origin_type=ViewerConfig.OriginType.ASSET_BODY, entity_name="car", body_name="chassis",
      distance=3.0, elevation=55.0, azimuth=180.0,     # viser convention: + looks down from above
    ),
    sim=SimulationCfg(
      nconmax=96, njmax=256,
      mujoco=MujocoCfg(timestep=0.005, integrator="implicitfast"),
    ),
    decimation=10,               # 20 Hz policy
    episode_length_s=30.0,
  )
  if play:
    cfg.episode_length_s = 120.0
    cfg.observations["actor"].enable_corruption = False
  return cfg


def office_ppo_runner_cfg() -> RslRlOnPolicyRunnerCfg:
  return RslRlOnPolicyRunnerCfg(
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
    experiment_name="office",
    logger="tensorboard",
    save_interval=50,
    num_steps_per_env=32,
    max_iterations=600,
  )
