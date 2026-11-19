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
# v_max: forward speed at a[0] = 1 (m/s). Default: the stock joystick limit; the explore task uses the stock
# driver's /cmd_vel clip of 0.2 m/s (research/02), which is the speed the real cars are driven at.
# steer_max_range: each car's steering limit is drawn from this range at every reset (our two cars may
# be the 29 deg 2024 build or the 40 deg 2025 build, capped at 37 deg by Hiwonder's code).
# Per-car actuation randomization, drawn at every reset (tasks/randomization.py sets the ranges):
#   latency_range_s:        policy output -> motors and servo (ROS, serial, firmware); delays both.
#   steer_deadtime_range_s: extra steering delay: the servo reads a new pulse once per 20 ms PWM period.
#   steer_rate_scale_range: servo speed relative to its 0.16 s/60 deg rating.
# Delays are whole physics steps (5 ms).
# Low-speed behaviour of the real drive (off by default; the turnaround task turns it on):
#   v_dead, v_min: |v| < v_dead -> stop; other commands below v_min are raised to v_min, exactly what the car-side
#     node does (car/policy/explore_policy.py V_DEAD / V_MIN: car #1's motors do not follow < 0.03 m/s, measured
#     2026-11-03).
#   steer_cmd_rate: max change of the normalized steering command a[1] per policy step (0 = off). Applied to the
#     policy output before anything else, here and in the car-side node (car/policy/explore_policy.py), so the
#     policy learns to drive with it and the car can never steer more jittery than this. Design choice (explore v4e).
#   shield: (env, v [N] m/s, steering [N] rad) -> [N] bool; where True the speed command is set to 0 (the car-side
#     node's obstacle stop, applied in the sim too so the policy trains with it; explore v5). self.shielded holds the
#     mask of the last step for a penalty term.
#   unstick: (held_s, back_s, v_frac, window_s, dist_m) or None: the car-side node's back-and-forth rule
#     (car/policy/explore_policy.py UNSTICK_*, 2026-11-05), applied in the sim too: once the shield has held the car
#     held_s, or the car has moved less than dist_m in the last window_s (turning in place without getting anywhere),
#     it backs up for back_s at v_frac x v_reverse with the steering opposite to the policy's, then the policy drives
#     again (a blocked back-up ends at once). Floor goto11-15: the policy never chose reverse by itself; sim v6h 7350
#     in 1.0-1.3 m halls: turned left / right in place, never reversed.
#   steer_at_rest_prob: per car and episode, the chance that the steering can be set while stopped. Hiwonder's
#     ackermann.py steers by atan(L w / v) (research/02), so through /cmd_vel the servo cannot be moved at v = 0;
#     whether a node can drive the servo directly is not verified. Cars that cannot hold the last steering command.


# Back-up escalation (goto v7, ours; floor goto17 2026-11-08: 10 of 14 back-ups ended with the car on the same spot and
# heading ~1 s later, then held again): a back-up that starts within ESCALATE_M / ESCALATE_DEG of where the last one
# started backs up 1 + n times as long (n repeats, at most ESCALATE_MAX). Car side: explore_policy.py ESCALATE_*.
ESCALATE_M, ESCALATE_DEG, ESCALATE_MAX = 0.3, 30.0, 2
REAR_WAIT_S = 1.5   # s a back-up waits for a clear rear before going anyway (ours: a passer-by walks on, a wall stays)


@dataclass(kw_only=True)
class AckermannActionCfg(ActionTermCfg):
  leader_step: Callable[..., None] | None = None   # (env) -> None; loose hint so tyro can resolve it
  v_reverse: float = 0.0
  v_max: float = V_MAX
  steer_max_range: tuple[float, float] = (STEER_MAX, STEER_MAX)
  latency_range_s: tuple[float, float] = (0.0, 0.0)
  steer_deadtime_range_s: tuple[float, float] = (0.0, 0.0)
  steer_rate_scale_range: tuple[float, float] = (1.0, 1.0)
  v_dead: float = 0.0
  v_min: float = 0.0
  steer_at_rest_prob: float = 1.0
  steer_cmd_rate: float = 0.0
  shield: Callable[..., torch.Tensor] | None = None
  unstick: tuple[float, float, float, float, float] | None = None
  unstick_away: Callable[..., torch.Tensor] | None = None   # (env) -> [N] bool: also back up when this is True
  unstick_rear_clear: float = 0.0   # m: start a back-up only when the LiDAR's rear quarter is clear this far (0 = off),
                                    # or after waiting REAR_WAIT_S for it
  unstick_escalate: bool = False    # goto v7: repeated back-ups at one spot back up longer (ESCALATE_*)
  # 2026-11-08 (imitation students): exponential smoothing of the steering command, steer_cmd <- steer_cmd + steer_ema x
  # (target - steer_cmd) before the rate limit (0 or 1 = off). The car node applies the same (explore_policy STEER_EMA).
  steer_ema: float = 0.0
  # 2026-11-08: the policy drives forward only; every reverse comes from the scripted back-up (unstick). Students imitate
  # the teacher's multi-point-turn reverses without seeing its mode: 6-7 reverse blips/min vs the teacher's 2.
  forward_only: bool = False

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
    self._steer_cmd = torch.zeros(self.num_envs, device=self.device)   # normalized, after steer_cmd_rate
    self._omega = torch.zeros(self.num_envs, 2, device=self.device)
    self.steer_max = torch.full((self.num_envs,), cfg.steer_max_range[1], device=self.device)
    self._steer_now = torch.zeros(self.num_envs, 2, device=self.device)   # servo position, rate-limited
    self._steer_step = STEER_RATE * env.physics_dt
    self._dt = env.physics_dt
    steps = lambda s: round(s / env.physics_dt)
    self._hist_len = steps(cfg.latency_range_s[1]) + steps(cfg.steer_deadtime_range_s[1]) + 1
    self._hist = torch.zeros(self.num_envs, self._hist_len, 3, device=self.device)  # (steer, omega_l, omega_r)
    self._head = 0
    self._lag = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)        # physics steps
    self._steer_lag = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
    self._rate_scale = torch.ones(self.num_envs, device=self.device)
    self._steer_at_rest = torch.ones(self.num_envs, dtype=torch.bool, device=self.device)
    self.v_cmd = torch.zeros(self.num_envs, device=self.device)        # m/s, after the dead band (for tasks)
    self.shielded = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
    self.held = torch.zeros(self.num_envs, device=self.device)                # s the shield has held the car (unstick)
    self.back_left = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)   # policy steps of back-up left
    self.back_steer = torch.zeros(self.num_envs, device=self.device)
    self._pos_hist = None                                                     # [N, K, 2] car positions (unstick)
    self._pos_n = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
    self.back_try = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
    self.rear_wait = torch.zeros(self.num_envs, device=self.device)               # s a back-up waited for the rear
    self.backing = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)   # the back-up drives this step
    self.back_xy = torch.full((self.num_envs, 2), 1e3, device=self.device)       # where the last back-up started
    self.back_yaw = torch.zeros(self.num_envs, device=self.device)
    self.back_rep = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)

  @property
  def action_dim(self) -> int:
    return 2

  @property
  def raw_action(self) -> torch.Tensor:
    return self._raw

  def process_actions(self, actions: torch.Tensor) -> None:
    hook = getattr(self._env, "_carskill_hook", None)
    if hook is not None:          # RL fine-tuning with the car's skill layer in the loop (robotlab/tasks/carskill.py)
      actions = hook(actions)
    self._raw[:] = actions
    a = actions.clamp(-1.0, 1.0)
    if self.cfg.v_reverse > 0:
      v = torch.where(a[:, 0] >= 0, a[:, 0] * self.cfg.v_max, a[:, 0] * self.cfg.v_reverse)
    else:
      v = (a[:, 0] + 1.0) / 2.0 * self.cfg.v_max
    if self.cfg.forward_only:     # not during a manoeuvre (env._robotlab_manoeuvre: the multi-point-turn skill reverses)
      man = getattr(self._env, "_robotlab_manoeuvre", None)
      v = torch.where(man, v, v.clamp(min=0.0)) if man is not None else v.clamp(min=0.0)
    if self.cfg.v_dead > 0 or self.cfg.v_min > 0:
      mag = v.abs()
      v = torch.where(mag < self.cfg.v_dead, 0.0, torch.sign(v) * mag.clamp(min=self.cfg.v_min))
    self.v_cmd[:] = v
    tgt = a[:, 1]
    if 0.0 < self.cfg.steer_ema < 1.0:
      tgt = self._steer_cmd + self.cfg.steer_ema * (tgt - self._steer_cmd)
    if self.cfg.steer_cmd_rate > 0:
      r = self.cfg.steer_cmd_rate
      self._steer_cmd += (tgt - self._steer_cmd).clamp(-r, r)
    else:
      self._steer_cmd[:] = a[:, 1]
    delta = self._steer_cmd * self.steer_max
    if self.cfg.steer_at_rest_prob < 1.0:
      delta = torch.where((v == 0) & ~self._steer_at_rest, self._steer[:, 0], delta)
    if self.cfg.unstick is not None:
      held_s, back_s, v_frac, window_s, dist_m = self.cfg.unstick
      pos = self._entity.data.root_link_pos_w[:, :2]
      K = max(round(window_s / self._env.step_dt), 1)
      if self._pos_hist is None:
        self._pos_hist = pos[:, None].repeat(1, K, 1)
      self._pos_hist = torch.cat([self._pos_hist[:, 1:], pos[:, None]], 1)
      self._pos_n += 1
      stalled = (self._pos_n >= K) & (torch.linalg.norm(pos - self._pos_hist[:, 0], dim=-1) < dist_m)
      start = ((self.held > held_s) | stalled) & (self.back_left == 0)
      # no scripted back-up while a multi-point turn runs (2026-11-08; = car node: unstick off while kturn_on). Teacher
      # v17: 9 of 11 chair hits were back-ups / reverse legs, the back-up firing inside the turn and fighting it
      man = getattr(self._env, "_robotlab_manoeuvre", None)
      if man is not None:
        start &= ~man
      if self.cfg.unstick_away is not None:
        start |= self.cfg.unstick_away(self._env) & (self.back_left == 0) & (~man if man is not None else True)
      if self.cfg.unstick_rear_clear > 0:      # v6n: no back-up towards someone behind (v6m 8300: 10% of person
        from robotlab.tasks import mentorpi_navigate_env_cfg as _N   # contacts during a back-up, 11% behind)
        sec = _N.lidar_sectors(self._env) * _N.LIDAR_RANGE                       # 36 x 10 deg, sector i at 5 + 10 i
        rear_ok = sec[:, 13:23].amin(-1) > self.cfg.unstick_rear_clear          # 135-225 deg: the rear quarter
        rear_ok |= self.rear_wait > REAR_WAIT_S       # still not clear after the wait: a wall, not a passer-by
        self.rear_wait = torch.where(start & ~rear_ok, self.rear_wait + self._env.step_dt,
                                     torch.where(start, self.rear_wait, 0.0))      # no back-up wanted: start over
        start &= rear_ok
      self._pos_n = torch.where(start, 0, self._pos_n)                           # a fresh window after each back-up
      steps = torch.full_like(self.back_left, round(back_s / self._env.step_dt))
      if self.cfg.unstick_escalate:
        yaw = quat_yaw(self._entity.data.root_link_quat_w)
        same = (torch.linalg.norm(pos - self.back_xy, dim=-1) < ESCALATE_M) & (
          wrap(yaw - self.back_yaw).abs() < math.radians(ESCALATE_DEG))
        self.back_rep = torch.where(start, torch.where(same, self.back_rep + 1, 0), self.back_rep)
        self.back_xy = torch.where(start[:, None], pos, self.back_xy)
        self.back_yaw = torch.where(start, yaw, self.back_yaw)
        steps = steps * (1 + self.back_rep.clamp(max=ESCALATE_MAX))
      self.back_left = torch.where(start, steps, self.back_left)
      # consecutive blocked attempts cycle the back-up steering: opposite, straight, same side as the policy (v6m: 5% of
      # blocked starts stayed boxed in, every back-up with the same full opposite lock blocked at once)
      side = torch.tensor([-1.0, 0.0, 1.0], device=self.device)[self.back_try % 3]
      self.back_steer = torch.where(start, side * torch.where(delta >= 0, 1.0, -1.0) * self.steer_max, self.back_steer)
      self.back_try = torch.where(start, self.back_try + 1, self.back_try)
      self.held = torch.where(start, 0.0, self.held)   # = car: held restarts at each back-up (audit 2026-11-09)
      backing = self.back_left > 0
      self.backing[:] = backing
      v = torch.where(backing, -self.cfg.v_reverse * v_frac, v)
      delta = torch.where(backing, self.back_steer, delta)
      self.back_left = (self.back_left - 1).clamp(min=0)
    if self.cfg.shield is not None:
      self.shielded = self.cfg.shield(self._env, v, delta)
      v = torch.where(self.shielded, 0.0, v)
      self.v_cmd[:] = v
    if self.cfg.unstick is not None:
      self.back_left = torch.where(backing & self.shielded, 0, self.back_left)   # blocked back-up: policy again
      active = backing & ~self.shielded
      self.back_try = torch.where(active, 0, self.back_try)                       # it moved: start the cycle again
      self.rear_wait = torch.where(active, 0.0, self.rear_wait)                   # v6q: and wait for the rear again
      self.held = torch.where(active, 0.0, torch.where(self.shielded, self.held + self._env.step_dt, 0.0))
    k = TRACK * torch.tan(delta) / (2 * WHEELBASE)
    self._omega[:] = torch.stack([v * (1 - k), v * (1 + k)], -1) / WHEEL_RADIUS    # left, right rear
    self._steer[:] = delta[:, None]
    if self.cfg.leader_step is not None:
      self.cfg.leader_step(self._env)

  def apply_actions(self) -> None:
    # Commands reach the car late (per-car latency); the servo then moves toward the commanded angle at
    # its own speed, not instantly.
    self._head = (self._head + 1) % self._hist_len
    self._hist[:, self._head] = torch.cat([self._steer[:, :1], self._omega], -1)
    ar = torch.arange(self.num_envs, device=self.device)
    steer = self._hist[ar, (self._head - self._steer_lag) % self._hist_len, 0][:, None]
    omega = self._hist[ar, (self._head - self._lag) % self._hist_len, 1:]
    step = (self._steer_step * self._rate_scale)[:, None]
    self._steer_now += torch.clamp(steer - self._steer_now, -step, step)
    self._entity.set_joint_position_target(self._steer_now, joint_ids=self._steer_ids)
    self._entity.set_joint_velocity_target(omega, joint_ids=self._drive_ids)

  def reset(self, env_ids: torch.Tensor | slice | None = None) -> None:
    self._raw[env_ids] = 0.0
    self._steer_now[env_ids] = 0.0
    self._steer[env_ids] = 0.0
    self._steer_cmd[env_ids] = 0.0
    self.v_cmd[env_ids] = 0.0
    self.shielded[env_ids] = False
    self.held[env_ids] = 0.0
    self.back_left[env_ids] = 0
    self._pos_n[env_ids] = 0
    self.back_try[env_ids] = 0
    self.rear_wait[env_ids] = 0.0
    self.backing[env_ids] = False
    self.back_xy[env_ids] = 1e3
    self.back_rep[env_ids] = 0
    lo, hi = self.cfg.steer_max_range
    n = self.num_envs if env_ids is None or isinstance(env_ids, slice) else len(env_ids)
    u = lambda r: r[0] + (r[1] - r[0]) * torch.rand(n, device=self.device)
    self.steer_max[env_ids] = u((lo, hi))
    self._hist[env_ids] = 0.0
    self._lag[env_ids] = torch.round(u(self.cfg.latency_range_s) / self._dt).long()
    self._steer_lag[env_ids] = self._lag[env_ids] + torch.round(u(self.cfg.steer_deadtime_range_s) / self._dt).long()
    self._rate_scale[env_ids] = u(self.cfg.steer_rate_scale_range)
    self._steer_at_rest[env_ids] = torch.rand(n, device=self.device) < self.cfg.steer_at_rest_prob
