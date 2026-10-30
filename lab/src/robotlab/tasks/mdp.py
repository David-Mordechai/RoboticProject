"""Shared MDP terms for the MentorPi tasks: Ackermann action, pose helpers, observations, LiDAR pattern."""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING

import mujoco
import torch

from mjlab.entity import Entity
from mjlab.managers.action_manager import ActionTerm, ActionTermCfg

from robotlab.robot.mentorpi_a1_constants import STEER_MAX, STEER_RATE, TRACK, V_MAX, WHEEL_RADIUS, WHEELBASE

if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv


# LiDAR: horizontal rays all around the scan centre (mjlab's patterns are all parallel rays).


@dataclass
class LidarPatternCfg:
  num_rays: int = 36

  def generate_rays(self, mj_model: mujoco.MjModel | None, device: str) -> tuple[torch.Tensor, torch.Tensor]:
    del mj_model
    a = torch.arange(self.num_rays, device=device, dtype=torch.float32) * (2 * math.pi / self.num_rays)
    dirs = torch.stack([torch.cos(a), torch.sin(a), torch.zeros_like(a)], -1)
    return torch.zeros(self.num_rays, 3, device=device), dirs


# Math.


def yaw_quat(yaw: torch.Tensor) -> torch.Tensor:
  return torch.stack([torch.cos(yaw / 2), torch.zeros_like(yaw), torch.zeros_like(yaw), torch.sin(yaw / 2)], -1)


def quat_yaw(q: torch.Tensor) -> torch.Tensor:
  w, x, y, z = q.unbind(-1)
  return torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def wrap(a: torch.Tensor) -> torch.Tensor:
  return torch.atan2(torch.sin(a), torch.cos(a))


def to_frame(d: torch.Tensor, yaw: torch.Tensor) -> torch.Tensor:
  """Rotate world-frame xy vectors [N, 2] into a frame with heading yaw."""
  c, s = torch.cos(yaw), torch.sin(yaw)
  return torch.stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]], -1)


# Car and leader state.


def write_leader_pose(env: ManagerBasedRlEnv, env_ids: torch.Tensor, xy: torch.Tensor, yaw: torch.Tensor) -> None:
  """Move the kinematic leader; xy is in the env frame."""
  leader: Entity = env.scene["leader"]
  pos = torch.cat([xy, torch.full_like(yaw, WHEEL_RADIUS)[:, None]], -1) + env.scene.env_origins[env_ids]
  leader.write_mocap_pose_to_sim(torch.cat([pos, yaw_quat(yaw)], -1), env_ids=env_ids)


def place_car(env: ManagerBasedRlEnv, env_ids: torch.Tensor, xy: torch.Tensor, yaw: torch.Tensor) -> None:
  """Teleport the learning car to xy (env frame), at rest."""
  root = torch.zeros(len(env_ids), 13, device=env.device)
  root[:, :2] = xy + env.scene.env_origins[env_ids, :2]
  root[:, 2] = WHEEL_RADIUS + 0.002
  root[:, 3:7] = yaw_quat(yaw)
  env.scene["car"].write_root_state_to_sim(root, env_ids=env_ids)


def car_pose(env: ManagerBasedRlEnv) -> tuple[torch.Tensor, torch.Tensor]:
  """Learning car xy (env frame) and yaw."""
  car: Entity = env.scene["car"]
  return car.data.root_link_pos_w[:, :2] - env.scene.env_origins[:, :2], quat_yaw(car.data.root_link_quat_w)


def car_motion(env: ManagerBasedRlEnv) -> torch.Tensor:
  """Forward/lateral speed (body frame), yaw rate, current steering angle."""
  car: Entity = env.scene["car"]
  steer_ids, _ = car.find_joints(("steer_fl",))
  return torch.cat([car.data.root_link_lin_vel_b[:, :2], car.data.root_link_ang_vel_b[:, 2:3],
                    car.data.joint_pos[:, steer_ids] / STEER_MAX], -1)


# Action: (speed, steering angle) -> rear-wheel velocity targets + both steering targets.
# Same interface the real car will get (motor rev/s + servo pulse via the RRC Lite). The two rear wheels
# get different speeds in turns, exactly as Hiwonder's controller does (manual 9.2):
#   V_L = V * (1 - D tan(d) / 2H),  V_R = V * (1 + D tan(d) / 2H)
# leader_step, if set, advances the task's scripted leader once per policy step.
# v_reverse > 0 lets the car back up (a[0] in [-1, 0) -> up to v_reverse m/s backwards), as the real
# car can; a car-like robot that overshoots a corner can't recover otherwise. With 0, speed is forward only.
# steer_max_range: each car's steering limit is drawn from this range at every reset (our two cars may
# be the 29 deg 2024 build or the 40 deg 2025 build, capped at 37 deg by Hiwonder's code).


@dataclass(kw_only=True)
class AckermannActionCfg(ActionTermCfg):
  leader_step: Callable[..., None] | None = None   # (env) -> None; loose hint so tyro can resolve it
  v_reverse: float = 0.0
  steer_max_range: tuple[float, float] = (STEER_MAX, STEER_MAX)

  def build(self, env: ManagerBasedRlEnv) -> AckermannAction:
    return AckermannAction(self, env)


class AckermannAction(ActionTerm):
  cfg: AckermannActionCfg

  def __init__(self, cfg: AckermannActionCfg, env: ManagerBasedRlEnv):
    super().__init__(cfg, env)
    ids = lambda names: torch.tensor(self._entity.find_joints(names, preserve_order=True)[0], device=self.device)
    self._steer_ids, self._drive_ids = ids(("steer_fl", "steer_fr")), ids(("wheel_rl", "wheel_rr"))
    self._raw = torch.zeros(self.num_envs, 2, device=self.device)
    self._steer = torch.zeros(self.num_envs, 2, device=self.device)
    self._omega = torch.zeros(self.num_envs, 2, device=self.device)
    self.steer_max = torch.full((self.num_envs,), cfg.steer_max_range[1], device=self.device)
    self._steer_now = torch.zeros(self.num_envs, 2, device=self.device)   # servo position, rate-limited
    self._steer_step = STEER_RATE * env.physics_dt

  @property
  def action_dim(self) -> int:
    return 2

  @property
  def raw_action(self) -> torch.Tensor:
    return self._raw

  def process_actions(self, actions: torch.Tensor) -> None:
    self._raw[:] = actions
    a = actions.clamp(-1.0, 1.0)
    if self.cfg.v_reverse > 0:
      v = torch.where(a[:, 0] >= 0, a[:, 0] * V_MAX, a[:, 0] * self.cfg.v_reverse)
    else:
      v = (a[:, 0] + 1.0) / 2.0 * V_MAX
    delta = a[:, 1] * self.steer_max
    k = TRACK * torch.tan(delta) / (2 * WHEELBASE)
    self._omega[:] = torch.stack([v * (1 - k), v * (1 + k)], -1) / WHEEL_RADIUS    # left, right rear
    self._steer[:] = delta[:, None]
    if self.cfg.leader_step is not None:
      self.cfg.leader_step(self._env)

  def apply_actions(self) -> None:
    # The servo moves toward the commanded angle at its rated speed, not instantly.
    self._steer_now += (self._steer - self._steer_now).clamp(-self._steer_step, self._steer_step)
    self._entity.set_joint_position_target(self._steer_now, joint_ids=self._steer_ids)
    self._entity.set_joint_velocity_target(self._omega, joint_ids=self._drive_ids)

  def reset(self, env_ids: torch.Tensor | slice | None = None) -> None:
    self._raw[env_ids] = 0.0
    self._steer_now[env_ids] = 0.0
    lo, hi = self.cfg.steer_max_range
    n = self.num_envs if env_ids is None or isinstance(env_ids, slice) else len(env_ids)
    self.steer_max[env_ids] = lo + (hi - lo) * torch.rand(n, device=self.device)
