"""Shared MentorPi A1 pieces for all tasks: car/leader entities, Ackermann action, pose helpers."""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

import mujoco
import torch

from mjlab.actuator.xml_actuator import XmlActuatorCfg
from mjlab.entity import Entity, EntityArticulationInfoCfg, EntityCfg
from mjlab.managers.action_manager import ActionTerm, ActionTermCfg

if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv

_ASSETS = Path(__file__).resolve().parents[1] / "assets"
WHEEL_RADIUS = 0.0325
WHEELBASE = 0.145
STEER_MAX = math.radians(35.0)
V_MAX = 0.8                      # m/s, training cap (not the hardware limit)


# Entities.


def car_cfg() -> EntityCfg:
  return EntityCfg(
    spec_fn=lambda: mujoco.MjSpec.from_file(str(_ASSETS / "mentorpi_a1.xml")),
    articulation=EntityArticulationInfoCfg(
      actuators=(
        XmlActuatorCfg(target_names_expr=("steer_.*",)),
        XmlActuatorCfg(target_names_expr=("wheel_rl", "wheel_rr")),
      ),
    ),
    init_state=EntityCfg.InitialStateCfg(pos=(0.0, 0.0, WHEEL_RADIUS + 0.002)),
  )


def leader_cfg() -> EntityCfg:
  return EntityCfg(spec_fn=lambda: mujoco.MjSpec.from_file(str(_ASSETS / "leader_car.xml")))


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
# Same interface the real car will get (motor rev/s + servo pulse via the RRC Lite).
# leader_step, if set, advances the task's scripted leader once per policy step.
# v_reverse > 0 lets the car back up (a[0] in [-1, 0) -> up to v_reverse m/s backwards), as the real
# car can; a car-like robot that overshoots a corner can't recover otherwise. With 0, speed is forward only.


@dataclass(kw_only=True)
class AckermannActionCfg(ActionTermCfg):
  leader_step: Callable[..., None] | None = None   # (env) -> None; loose hint so tyro can resolve it
  v_reverse: float = 0.0

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
    self._omega[:] = (v / WHEEL_RADIUS)[:, None]
    self._steer[:] = (a[:, 1] * STEER_MAX)[:, None]
    if self.cfg.leader_step is not None:
      self.cfg.leader_step(self._env)

  def apply_actions(self) -> None:
    self._entity.set_joint_position_target(self._steer, joint_ids=self._steer_ids)
    self._entity.set_joint_velocity_target(self._omega, joint_ids=self._drive_ids)

  def reset(self, env_ids: torch.Tensor | slice | None = None) -> None:
    self._raw[env_ids] = 0.0
