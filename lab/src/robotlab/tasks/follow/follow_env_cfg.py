"""Follow task: a MentorPi A1 learns to hold a slot 0.6 m behind a scripted leader car.

First task of the lab, built the mjlab way (manager-based env, as in Microduck's microduck_rl):
scene with two entities, a custom Ackermann action term, observation/reward/termination/event
terms, and an rsl_rl PPO runner config. Two learning cars (MAPPO) come next.
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
from mjlab.sim import MujocoCfg, SimulationCfg
from mjlab.terrains import TerrainEntityCfg
from mjlab.utils.noise import UniformNoiseCfg
from mjlab.viewer import ViewerConfig

from robotlab.tasks.common import (
  STEER_MAX, WHEELBASE, AckermannActionCfg, car_cfg, car_motion, car_pose, leader_cfg, place_car, wrap,
  write_leader_pose,
)

if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv

LEADER_SPEED = 0.4
SLOT = (-0.6, 0.0)               # target slot in the leader frame (m)


# Leader script: shared per-env state, advanced once per policy step.


@dataclass
class LeaderState:
  xy: torch.Tensor      # [N, 2] in the env frame
  yaw: torch.Tensor     # [N]
  steer: torch.Tensor   # [N]
  freq: torch.Tensor    # [N] Hz of the weaving steering pattern
  phase: torch.Tensor   # [N]
  t: torch.Tensor       # [N] seconds since reset


def leader_state(env: ManagerBasedRlEnv) -> LeaderState:
  if not hasattr(env, "_robotlab_leader"):
    z = torch.zeros(env.num_envs, device=env.device)
    env._robotlab_leader = LeaderState(torch.zeros(env.num_envs, 2, device=env.device), z.clone(), z.clone(), z.clone(), z.clone(), z.clone())
  return env._robotlab_leader


def _write_leader(env: ManagerBasedRlEnv, env_ids: torch.Tensor) -> None:
  s = leader_state(env)
  write_leader_pose(env, env_ids, s.xy[env_ids], s.yaw[env_ids])


def slot_pose(env: ManagerBasedRlEnv) -> tuple[torch.Tensor, torch.Tensor]:
  """Target slot (xy in env frame, yaw) behind the leader."""
  s = leader_state(env)
  c, sn = torch.cos(s.yaw), torch.sin(s.yaw)
  off = torch.stack([SLOT[0] * c - SLOT[1] * sn, SLOT[0] * sn + SLOT[1] * c], -1)
  return s.xy + off, s.yaw


def advance_leader(env: ManagerBasedRlEnv) -> None:
  """Weaving leader: kinematic bicycle with a sinusoidal steering pattern."""
  s, dt = leader_state(env), env.step_dt
  s.steer = 0.45 * STEER_MAX * torch.sin(2 * math.pi * s.freq * s.t + s.phase)
  s.yaw = wrap(s.yaw + LEADER_SPEED / WHEELBASE * torch.tan(s.steer) * dt)
  s.xy = s.xy + LEADER_SPEED * dt * torch.stack([torch.cos(s.yaw), torch.sin(s.yaw)], -1)
  s.t = s.t + dt
  _write_leader(env, torch.arange(env.num_envs, device=env.device))


# Events.


def reset_pair(env: ManagerBasedRlEnv, env_ids: torch.Tensor, start_radius: float = 1.0) -> None:
  """New leader heading and weaving pattern; follower placed near its slot."""
  s, n, dev = leader_state(env), len(env_ids), env.device
  s.xy[env_ids] = 0.0
  s.yaw[env_ids] = (torch.rand(n, device=dev) * 2 - 1) * math.pi
  s.freq[env_ids] = 0.15 + 0.35 * torch.rand(n, device=dev)
  s.phase[env_ids] = torch.rand(n, device=dev) * 2 * math.pi
  s.t[env_ids] = 0.0
  s.steer[env_ids] = 0.0
  _write_leader(env, env_ids)

  slot_xy, slot_yaw = slot_pose(env)
  r = torch.rand(n, device=dev) * start_radius
  a = (torch.rand(n, device=dev) * 2 - 1) * math.pi
  xy = slot_xy[env_ids] + torch.stack([r * torch.cos(a), r * torch.sin(a)], -1)
  yaw = slot_yaw[env_ids] + (torch.rand(n, device=dev) * 2 - 1)
  place_car(env, env_ids, xy, yaw)


# Observations (car frame, so the policy is position-independent).


def slot_error(env: ManagerBasedRlEnv) -> torch.Tensor:
  """Slot position error in the car frame (2) and heading error as sin/cos (2)."""
  xy, yaw = car_pose(env)
  slot_xy, slot_yaw = slot_pose(env)
  d = slot_xy - xy
  c, s = torch.cos(yaw), torch.sin(yaw)
  ex, ey = c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]
  dyaw = wrap(slot_yaw - yaw)
  return torch.stack([ex, ey, torch.sin(dyaw), torch.cos(dyaw)], -1)


def leader_motion(env: ManagerBasedRlEnv) -> torch.Tensor:
  s = leader_state(env)
  return torch.stack([torch.full_like(s.yaw, LEADER_SPEED), s.steer / STEER_MAX], -1)


# Rewards and terminations.


def slot_distance(env: ManagerBasedRlEnv) -> torch.Tensor:
  xy, _ = car_pose(env)
  return torch.linalg.norm(slot_pose(env)[0] - xy, dim=-1)


def track_slot(env: ManagerBasedRlEnv, std: float) -> torch.Tensor:
  return torch.exp(-(slot_distance(env) / std) ** 2)


def heading_match(env: ManagerBasedRlEnv) -> torch.Tensor:
  _, yaw = car_pose(env)
  return torch.cos(wrap(slot_pose(env)[1] - yaw))


def too_close(env: ManagerBasedRlEnv, min_gap: float) -> torch.Tensor:
  xy, _ = car_pose(env)
  return (torch.linalg.norm(leader_state(env).xy - xy, dim=-1) < min_gap).float()


def lost_slot(env: ManagerBasedRlEnv, max_distance: float) -> torch.Tensor:
  return slot_distance(env) > max_distance


# Config.


def follow_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  actor_terms = {
    "slot_error": ObservationTermCfg(func=slot_error, noise=UniformNoiseCfg(n_min=-0.02, n_max=0.02)),
    "car_motion": ObservationTermCfg(func=car_motion, noise=UniformNoiseCfg(n_min=-0.02, n_max=0.02)),
    "leader_motion": ObservationTermCfg(func=leader_motion),
  }
  cfg = ManagerBasedRlEnvCfg(
    scene=SceneCfg(
      terrain=TerrainEntityCfg(terrain_type="plane"),
      entities={"car": car_cfg(), "leader": leader_cfg()},
      num_envs=1,
      env_spacing=12.0,
    ),
    observations={
      "actor": ObservationGroupCfg(actor_terms, enable_corruption=True),
      "critic": ObservationGroupCfg({**actor_terms}),
    },
    actions={"drive": AckermannActionCfg(entity_name="car", leader_step=advance_leader)},
    events={
      "reset_joints": EventTermCfg(
        func=reset_joints_by_offset, mode="reset",
        params={"position_range": (0.0, 0.0), "velocity_range": (0.0, 0.0), "asset_cfg": SceneEntityCfg("car")},
      ),
      "reset_pair": EventTermCfg(func=reset_pair, mode="reset", params={"start_radius": 1.0}),
    },
    rewards={
      "track_slot": RewardTermCfg(func=track_slot, weight=2.0, params={"std": 0.3}),
      "heading": RewardTermCfg(func=heading_match, weight=0.3),
      "too_close": RewardTermCfg(func=too_close, weight=-10.0, params={"min_gap": 0.25}),
      "action_rate": RewardTermCfg(func=action_rate_l2, weight=-0.05),
    },
    terminations={
      "time_out": TerminationTermCfg(func=time_out, time_out=True),
      "lost_slot": TerminationTermCfg(func=lost_slot, params={"max_distance": 3.0}),
    },
    viewer=ViewerConfig(
      origin_type=ViewerConfig.OriginType.ASSET_BODY, entity_name="car", body_name="chassis",
      distance=2.0, elevation=40.0, azimuth=180.0,     # viser convention: + looks down from above
    ),
    sim=SimulationCfg(
      nconmax=64, njmax=128,
      mujoco=MujocoCfg(timestep=0.005, integrator="implicitfast"),
    ),
    decimation=10,               # 20 Hz policy, like the planned Pi 5 policy node
    episode_length_s=20.0,
  )
  if play:
    cfg.episode_length_s = 60.0
    cfg.observations["actor"].enable_corruption = False
  return cfg


def follow_ppo_runner_cfg() -> RslRlOnPolicyRunnerCfg:
  return RslRlOnPolicyRunnerCfg(
    actor=RslRlModelCfg(
      hidden_dims=(128, 128), activation="elu", obs_normalization=True,
      distribution_cfg={"class_name": "GaussianDistribution", "init_std": 0.6, "std_type": "scalar"},
    ),
    critic=RslRlModelCfg(hidden_dims=(128, 128), activation="elu", obs_normalization=True),
    algorithm=RslRlPpoAlgorithmCfg(
      value_loss_coef=1.0, use_clipped_value_loss=True, clip_param=0.2, entropy_coef=0.005,
      num_learning_epochs=5, num_mini_batches=4, learning_rate=1.0e-3, schedule="adaptive",
      gamma=0.99, lam=0.95, desired_kl=0.01, max_grad_norm=1.0,
    ),
    experiment_name="follow",
    logger="tensorboard",
    save_interval=25,
    num_steps_per_env=32,
    max_iterations=300,
  )
