"""Domain randomization for the MentorPi tasks (sim-to-real), Microduck style: one ENABLE_* flag per term.

apply_domain_randomization(cfg) adds the terms to a task config. Each car draws its own values at every
reset (mass at startup), so the policy learns to drive any car in the range, not one ideal car.

Every range cites its source. Where no source gives the value it says so: the range is then deliberately
wide and must be narrowed once the real cars are measured (docs/open-issues.md #3).

Applied in play mode too (evaluate_office.py then measures robustness on randomized cars), except the
observation noise, which mjlab switches off with enable_corruption=False.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

import torch

from mjlab.envs.mdp import dr
from mjlab.managers.event_manager import EventTermCfg, requires_model_fields
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.utils.noise.noise_cfg import NoiseCfg, NoiseModelCfg
from mjlab.utils.noise.noise_model import NoiseModel

if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv, ManagerBasedRlEnvCfg

ENABLE_TYRE_FRICTION = True
ENABLE_MASS_INERTIA = True
ENABLE_DRIVE_MOTOR = True
ENABLE_STEERING_SERVO = True
ENABLE_LATENCY = True
ENABLE_LIDAR_NOISE = True
ENABLE_LEADER_NOISE = True
ENABLE_SPEED_SCALE = True
ENABLE_LOCALIZATION_NOISE = True      # navigation tasks: error of the own pose the path is expressed in

# Tyre-floor friction coefficient. NO SOURCE: Hiwonder publishes no tyre data and our floors are unknown.
# Wide on purpose; narrow after a tilt-plane slip test on the real floors.
TYRE_FRICTION_RANGE = (0.5, 1.2)

# Mass and inertia scale, ±5% as Microduck (microduck_velocity_env_cfg.py, MASS_INERTIA_RANDOMIZATION_RANGE).
# Nominal mass ~1.2 kg (Hiwonder product page, research/01 §2); per-unit spread not published.
MASS_SCALE_RANGE = (0.95, 1.05)

# Drive motors: max wheel torque and the STM32 speed-loop stiffness, as scales of the model's values
# (forcerange 0.3 N m, kv 0.05, mentorpi_a1.xml). NO SOURCE for either: the 310 motor's stall torque and
# the RRC Lite PID gains are unpublished (research/01 §3, research/02 firmware). Wide on purpose; narrow
# after measuring acceleration and top speed (open-issues #3).
DRIVE_TORQUE_SCALE_RANGE = (0.5, 2.0)
DRIVE_GAIN_SCALE_RANGE = (0.5, 2.0)

# Steering servo (LD-1501MG manual, docs/vendor/hiwonder/datasheets/):
# - dead time 0-20 ms: the servo takes a new pulse once per 20 ms PWM period ("Pulse Period 20ms").
# - speed 0.16 s/60 deg is rated at 7.4 V, no load. The RRC Lite can feed the servo 5 V or VIN (research/01
#   §1.2), and the wheels load it, so it can only be slower: scale 0.6-1.0. The lower bound has NO SOURCE.
STEER_DEADTIME_RANGE_S = (0.0, 0.020)
STEER_RATE_SCALE_RANGE = (0.6, 1.0)

# Policy output -> motors. NO SOURCE for the end-to-end value: the serial link itself is fast (1 Mbaud,
# a motor packet is ~20 bytes = 0.2 ms; research/01 §1.2), the rest is ROS 2 scheduling and the RRC
# firmware loop, both unmeasured. 0-40 ms is an estimate; measure with a step command on the real car.
LATENCY_RANGE_S = (0.0, 0.040)

# LiDAR. LD19 datasheet v2.5: ranging standard deviation 10 mm and average accuracy ±45 mm (0.3-12 m, 70%
# reflectivity); 10 Hz scan (LD19 development manual §1, docs/vendor/hiwonder/datasheets/). Which LiDAR our
# cars carry is unresolved (LD19 / MS200 / COIN-D6, research/01 §5), so the noise std spans 0.5-2x the
# LD19 figure. Dropout rate: NO SOURCE (dark and glass surfaces, open-issues #11); 0-5% per ray is an estimate.
LIDAR_BIAS_M = 0.045
LIDAR_STD_RANGE_M = (0.005, 0.020)
LIDAR_DROPOUT_RANGE = (0.0, 0.05)
LIDAR_REFRESH_STEPS = 2          # 10 Hz scan / 20 Hz policy: each scan is seen for two policy steps
# Scan rate per car for the navigation tasks: LD19 defaults to 10 Hz, Hiwonder's stock MS200 launch spins it at
# 15 Hz (research/03 §1.1). At a 20 Hz policy a fresh scan arrives with probability rate / 20 per step.
LIDAR_SCAN_HZ_RANGE = (10.0, 15.0)

# Own speed reading (car_motion vx, vy): per-car scale error. Stock speed is the commanded one (open-loop,
# research/03 §1.4); wheel diameter 65 vs 67 mm alone is ±3% (research/01 §3; research/03 §4.3, derived).
SPEED_SCALE_RANGE = (0.97, 1.03)

# Own pose in the shared map (navigation tasks: the planned path reaches the policy through it). NO SOURCE:
# AMCL / slam_toolbox error on these cars is unmeasured (research/03 §3.4, G9). Estimates: a per-episode bias
# plus a correction jump at the stock update cadence, 0.5 s (research/03 §4.3). Measure in Phase 2.
LOC_BIAS_M = 0.05
LOC_BIAS_YAW = math.radians(3.0)
LOC_JUMP_M = 0.03                # std of each jump
LOC_JUMP_YAW = math.radians(1.5)
LOC_JUMP_STEPS = 10              # 0.5 s at 20 Hz

# Leader relative pose. NO SOURCE: on the real cars it comes from localization + the network, still open
# (open-issues #7, research/03). Estimates until #7 measures the real noise; then feed it back here.
LEADER_POS_STD_M = 0.03          # per step
LEADER_POS_BIAS_M = 0.05         # per episode
LEADER_YAW_STD = math.radians(2.0)
LEADER_YAW_BIAS = math.radians(3.0)
LEADER_LAG_STEPS = (0, 2)        # 0-100 ms at 20 Hz: Wi-Fi + localization latency, estimate


# Events.


@requires_model_fields("actuator_forcerange", "actuator_gainprm", "actuator_biasprm")
def randomize_drive_motor(
  env: ManagerBasedRlEnv,
  env_ids: torch.Tensor | None,
  torque_scale_range: tuple[float, float],
  gain_scale_range: tuple[float, float],
  asset_cfg: SceneEntityCfg,
) -> None:
  """Per car: one torque limit and one speed-loop gain for both rear motors (velocity servos,
  MuJoCo: force = kv * (ctrl - qvel), gainprm[0] = kv, biasprm[2] = -kv)."""
  car = env.scene[asset_cfg.name]
  ids = torch.cat([a.global_ctrl_ids for a in car.actuators if any(t.startswith("wheel_") for t in a.target_names)])
  assert len(ids) == 2, f"expected the two rear drive actuators, got {len(ids)}"
  if env_ids is None:
    env_ids = torch.arange(env.num_envs, device=env.device)
  n, m = env.sim.model, len(env_ids)
  u = lambda r: (r[0] + (r[1] - r[0]) * torch.rand(m, 1, device=env.device))
  torque, gain = u(torque_scale_range), u(gain_scale_range)
  rows = env_ids[:, None]
  n.actuator_forcerange[rows, ids] = env.sim.get_default_field("actuator_forcerange")[ids] * torque[..., None]
  n.actuator_gainprm[rows, ids, 0] = env.sim.get_default_field("actuator_gainprm")[ids, 0] * gain
  n.actuator_biasprm[rows, ids, 2] = env.sim.get_default_field("actuator_biasprm")[ids, 2] * gain


# Observation noise models (active only when the group has enable_corruption=True).


@dataclass(kw_only=True)
class _NoiseModelCfg(NoiseModelCfg, class_type=NoiseModel):
  noise_cfg: NoiseCfg | None = None   # unused: these models sample their own noise


class LidarNoiseModel(NoiseModel):
  """Range noise (per-car bias + per-ray std), dropouts (no return = max range) and a 10 Hz scan held for
  LIDAR_REFRESH_STEPS policy steps. Works on the normalized scan (range / max_range)."""

  def __init__(self, cfg: LidarNoiseCfg, num_envs: int, device: str):
    self.cfg, self._n, self._dev = cfg, num_envs, device
    z = lambda: torch.zeros(num_envs, device=device)
    self._bias, self._std, self._drop = z(), z(), z()
    self._phase = torch.randint(0, cfg.refresh_steps, (num_envs,), device=device)
    self._held: torch.Tensor | None = None
    self.reset()

  def reset(self, env_ids: torch.Tensor | slice | None = None) -> None:
    ids = slice(None) if env_ids is None else env_ids
    m = self._bias[ids].shape[0]
    u = lambda r: r[0] + (r[1] - r[0]) * torch.rand(m, device=self._dev)
    self._bias[ids] = u((-self.cfg.bias_m, self.cfg.bias_m))
    self._std[ids] = u(self.cfg.std_range_m)
    self._drop[ids] = u(self.cfg.dropout_range)
    if self._held is not None:
      self._held[ids] = -1.0                 # take a fresh scan on the first step

  def __call__(self, data: torch.Tensor) -> torch.Tensor:
    r = self.cfg.max_range
    hit = data < 1.0
    d = data * r + self._bias[:, None] + self._std[:, None] * torch.randn_like(data)
    d = torch.where(hit, d.clamp(0.0, r), r)
    d = torch.where(torch.rand_like(data) < self._drop[:, None], r, d)
    scan = d / r
    if self._held is None:
      self._held = scan.clone()
    self._phase = (self._phase + 1) % self.cfg.refresh_steps
    fresh = (self._phase == 0) | (self._held[:, 0] < 0)
    self._held = torch.where(fresh[:, None], scan, self._held)
    return self._held.clone()


@dataclass(kw_only=True)
class LidarNoiseCfg(_NoiseModelCfg, class_type=LidarNoiseModel):
  max_range: float
  bias_m: float = LIDAR_BIAS_M
  std_range_m: tuple[float, float] = LIDAR_STD_RANGE_M
  dropout_range: tuple[float, float] = LIDAR_DROPOUT_RANGE
  refresh_steps: int = LIDAR_REFRESH_STEPS


class LeaderPoseNoiseModel(NoiseModel):
  """Noise on the leader observation [x, y, sin(dyaw), cos(dyaw), speed, steer]: position and heading get
  a per-episode bias plus per-step noise; heading noise rotates the (sin, cos) pair so it stays a unit
  vector. Speed and steering are the leader's own commands, sent over the network: small uniform noise."""

  def __init__(self, cfg: LeaderPoseNoiseCfg, num_envs: int, device: str):
    self.cfg, self._dev = cfg, device
    self._pos_bias = torch.zeros(num_envs, 2, device=device)
    self._yaw_bias = torch.zeros(num_envs, device=device)
    self.reset()

  def reset(self, env_ids: torch.Tensor | slice | None = None) -> None:
    ids = slice(None) if env_ids is None else env_ids
    m = self._yaw_bias[ids].shape[0]
    self._pos_bias[ids] = self.cfg.pos_bias_m * (2 * torch.rand(m, 2, device=self._dev) - 1)
    self._yaw_bias[ids] = self.cfg.yaw_bias * (2 * torch.rand(m, device=self._dev) - 1)

  def __call__(self, data: torch.Tensor) -> torch.Tensor:
    out = data.clone()
    out[:, :2] += self._pos_bias + self.cfg.pos_std_m * torch.randn_like(data[:, :2])
    e = torch.atan2(data[:, 2], data[:, 3]) + self._yaw_bias + self.cfg.yaw_std * torch.randn_like(data[:, 2])
    out[:, 2], out[:, 3] = torch.sin(e), torch.cos(e)
    out[:, 4:] += self.cfg.cmd_noise * (2 * torch.rand_like(data[:, 4:]) - 1)
    return out


@dataclass(kw_only=True)
class LeaderPoseNoiseCfg(_NoiseModelCfg, class_type=LeaderPoseNoiseModel):
  pos_std_m: float = LEADER_POS_STD_M
  pos_bias_m: float = LEADER_POS_BIAS_M
  yaw_std: float = LEADER_YAW_STD
  yaw_bias: float = LEADER_YAW_BIAS
  cmd_noise: float = 0.02


class SpeedScaleNoiseModel(NoiseModel):
  """Per-car scale error on the first two values of car_motion (forward and lateral speed)."""

  def __init__(self, cfg: SpeedScaleNoiseCfg, num_envs: int, device: str):
    self.cfg, self._dev = cfg, device
    self._scale = torch.ones(num_envs, device=device)
    self.reset()

  def reset(self, env_ids: torch.Tensor | slice | None = None) -> None:
    ids = slice(None) if env_ids is None else env_ids
    lo, hi = self.cfg.scale_range
    self._scale[ids] = lo + (hi - lo) * torch.rand_like(self._scale[ids])

  def __call__(self, data: torch.Tensor) -> torch.Tensor:
    out = data.clone()
    out[:, :2] *= self._scale[:, None]
    return out


@dataclass(kw_only=True)
class SpeedScaleNoiseCfg(_NoiseModelCfg, class_type=SpeedScaleNoiseModel):
  scale_range: tuple[float, float] = SPEED_SCALE_RANGE


# Raw-ray LiDAR noise for the navigation tasks. Unlike LidarNoiseModel (which works on the 36 values the policy
# sees) this acts on every raw ray before the min-pooling into sectors, as the real scan would; the task calls
# it from its observation term. State lives on the env.


@dataclass
class RayNoiseState:
  bias: torch.Tensor      # [N] m
  std: torch.Tensor       # [N] m
  drop: torch.Tensor      # [N] probability per ray
  fresh_p: torch.Tensor   # [N] probability that a new scan arrived this step
  held: torch.Tensor      # [N, R] last scan, m (-1 = none yet)


def ray_noise_state(env: ManagerBasedRlEnv, num_rays: int) -> RayNoiseState:
  if not hasattr(env, "_robotlab_ray_noise"):
    z = lambda: torch.zeros(env.num_envs, device=env.device)
    env._robotlab_ray_noise = RayNoiseState(z(), z(), z(), z(), torch.full((env.num_envs, num_rays), -1.0, device=env.device))
  return env._robotlab_ray_noise


def reset_ray_noise(env: ManagerBasedRlEnv, env_ids: torch.Tensor, num_rays: int) -> None:
  s, m = ray_noise_state(env, num_rays), len(env_ids)
  u = lambda r: r[0] + (r[1] - r[0]) * torch.rand(m, device=env.device)
  s.bias[env_ids] = u((-LIDAR_BIAS_M, LIDAR_BIAS_M))
  s.std[env_ids] = u(LIDAR_STD_RANGE_M)
  s.drop[env_ids] = u(LIDAR_DROPOUT_RANGE)
  s.fresh_p[env_ids] = u(LIDAR_SCAN_HZ_RANGE) * env.step_dt
  s.held[env_ids] = -1.0


def noisy_rays(env: ManagerBasedRlEnv, dist: torch.Tensor, max_range: float) -> torch.Tensor:
  """dist [N, R] in m, max_range where nothing was hit. Bias + per-ray noise + dropouts, then sample-and-hold
  at the car's scan rate."""
  s = ray_noise_state(env, dist.shape[1])
  hit = dist < max_range
  d = dist + s.bias[:, None] + s.std[:, None] * torch.randn_like(dist)
  d = torch.where(hit, d.clamp(0.0, max_range), max_range)
  d = torch.where(torch.rand_like(dist) < s.drop[:, None], max_range, d)
  fresh = (torch.rand(env.num_envs, device=env.device) < s.fresh_p) | (s.held[:, 0] < 0)
  s.held = torch.where(fresh[:, None], d, s.held)
  return s.held.clone()


# Config.


def apply_domain_randomization(cfg: ManagerBasedRlEnvCfg, lidar_range: float | None = None) -> None:
  """Add the enabled terms to a task config with a "car" entity, a "drive" AckermannAction and, if present,
  actor observation terms "lidar" (normalized by lidar_range) and "leader" (leader_relative layout)."""
  car = SceneEntityCfg("car")
  if ENABLE_TYRE_FRICTION:
    cfg.events["tyre_friction"] = EventTermCfg(
      func=dr.geom_friction, mode="reset",
      params={"asset_cfg": SceneEntityCfg("car", geom_names=("tyre_.*",)), "operation": "abs",
              "ranges": TYRE_FRICTION_RANGE, "shared_random": True},    # same rubber on all four tyres
    )
  if ENABLE_MASS_INERTIA:
    # Microduck: alpha scales mass and inertia by e^(2 alpha), CoM unchanged; fixed per car for the run.
    lo, hi = MASS_SCALE_RANGE
    cfg.events["mass_inertia"] = EventTermCfg(
      func=dr.pseudo_inertia, mode="startup",
      params={"asset_cfg": SceneEntityCfg("car", body_names=("chassis",)),
              "alpha_range": (math.log(lo) / 2, math.log(hi) / 2)},
    )
  if ENABLE_DRIVE_MOTOR:
    cfg.events["drive_motor"] = EventTermCfg(
      func=randomize_drive_motor, mode="reset",
      params={"torque_scale_range": DRIVE_TORQUE_SCALE_RANGE, "gain_scale_range": DRIVE_GAIN_SCALE_RANGE,
              "asset_cfg": car},
    )
  drive = cfg.actions["drive"]
  if ENABLE_STEERING_SERVO:
    drive.steer_deadtime_range_s = STEER_DEADTIME_RANGE_S
    drive.steer_rate_scale_range = STEER_RATE_SCALE_RANGE
  if ENABLE_LATENCY:
    drive.latency_range_s = LATENCY_RANGE_S

  actor = cfg.observations["actor"].terms
  if ENABLE_SPEED_SCALE and "car_motion" in actor:
    actor["car_motion"].noise = SpeedScaleNoiseCfg()
  if ENABLE_LIDAR_NOISE and "lidar" in actor:
    assert lidar_range is not None, "lidar term needs lidar_range"
    actor["lidar"].noise = LidarNoiseCfg(max_range=lidar_range)
  if ENABLE_LEADER_NOISE and "leader" in actor:
    actor["leader"].noise = LeaderPoseNoiseCfg()
    actor["leader"].delay_min_lag, actor["leader"].delay_max_lag = LEADER_LAG_STEPS
    actor["leader"].delay_hold_prob = 0.5    # latency drifts rather than jumping every step

