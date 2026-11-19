"""Turnaround task: one MentorPi A1 faces into a dead-end corridor and must turn around, quickly and precisely: a single
U-turn where the corridor is wide enough, a multi-point turn (forward / reverse, steering swapped) where it is not.

Scene (per env, redrawn at every reset; kinematic mocap walls so every episode gets its own width):
- A corridor of width W along x: dead-end wall face at x = 0, side walls at y = +-W/2, a back wall at x = -L (L 2.5-4 m,
  beyond the 3 m LiDAR clip from most start poses).
- In LOW_OBSTACLE_PROB of the episodes a low obstacle near the dead end, BELOW the LiDAR plane, so only the depth camera
  sees it: an office-chair star base (5 legs) or a low box. Real cars were pinned on chair bases (open-issues §0).
- The car starts facing into the dead end, 0.3-1.5 m from the nearest thing ahead, yaw +-20 deg, random lateral
  offset where it fits. Target heading: the corridor axis pointing out (yaw pi).

Width range, derived (CLAUDE.md: geometry from sources, then measured in sim, lab/tests/test_turnaround.py):
- Turning radius of the rear-axle centre R = H / tan(theta) (Hiwonder manual 9.2; H = WHEELBASE 0.145 m):
  0.262 m at 29 deg, 0.192 m at 37 deg.
- Body: 213 x 159 mm (research/01 §3), centred between the axles in our model (mentorpi_a1.xml), so it reaches
  BODY_FRONT = 0.179 m ahead of and BODY_REAR = 0.034 m behind the rear axle, BODY_HALF_W = 0.0795 m to each side.
- A single 180 deg turn at full lock, starting along the corridor, sweeps the band between the outer front corner's
  circle (radius sqrt(BODY_FRONT^2 + (R + b)^2), it passes the top of the circle) and the outer rear corner's circle
  (radius sqrt(BODY_REAR^2 + (R + b)^2), it passes the bottom): W_1 = R_of + R_or = 0.600 m (37 deg), 0.728 m
  (29 deg) (uturn_width_formula below). The sim circles wider than H / tan(theta) (TURN_RADIUS_SIM, parallel-steering
  scrub, flagged below), so a sweep needs 0.675 m (37 deg) / 0.805 m (29 deg) there (SINGLE_TURN_MEASURED).
- Turning around at all needs W > the body diagonal sqrt(0.213^2 + 0.159^2) = 0.266 m (the rectangle's width across the
  corridor passes through the diagonal while its heading sweeps 180 deg); W_MIN = 0.40 m leaves 0.13 m for the
  multi-point turn: a scripted K-turn (ground truth, 0.12 m/s, switching 2 cm before contact) turns around there at 29
  and 37 deg in ~9-11 s with ~28 switches; at 0.45 m it needs 6-8, at 0.50 m 4, from 0.60 m 2.
- W is drawn half from [W_MIN, W_SPLIT] (multi-point turn needed for most cars) and half from [W_SPLIT, W_MAX]
  (one sweep is enough), W_SPLIT = the measured single-turn width between the 29 and 37 deg builds.

Observations (actor; same sensor terms and noise as the explore task so car/policy/explore_obs.py is reused):
  car_motion 4 | lidar_sectors 36 | depth 12 | heading_error 2 (sin, cos of target - yaw) | last_action 2  = 56.
Critic: the same without noise + privileged corridor geometry, obstacle pose and clearance.

Success: |heading error| < 10 deg, every body corner >= SUCCESS_CLEARANCE inside the corridor, |speed| < 0.05 m/s,
held SUCCESS_HOLD_S -> terminate with a bonus that grows with the time left. Contact -> penalty + terminate. 20 s limit.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

import mujoco
import torch

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs.mdp import action_rate_l2, reset_joints_by_offset, time_out
from mjlab.entity import EntityCfg
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
from mjlab.utils.noise.noise_model import NoiseModel
from mjlab.viewer import ViewerConfig

from robotlab.robot.mentorpi_a1_constants import STEER_2024, STEER_CODE, WHEELBASE, car_cfg
from robotlab.tasks import camera as C
from robotlab.tasks import randomization as R
from robotlab.tasks.lidar_rate import cast_lidar_every_other_step
from robotlab.tasks.mdp import AckermannActionCfg, LidarPatternCfg, car_motion, car_pose, place_car, to_frame, wrap, \
  yaw_quat
from robotlab.tasks.mentorpi_explore_env_cfg import arc_blocked, shield_stop
from robotlab.tasks.mentorpi_navigate_env_cfg import LIDAR_RANGE, LIDAR_RAYS, lidar_sectors
from robotlab.tasks.office_terrain import WALL_HEIGHT

if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv

# Speeds as the explore task: stock joystick limit 0.7 m/s forward (research/01 §3), 0.3 m/s reverse.
TURN_V_MAX = 0.7
TURN_V_REVERSE = 0.3
# Low-speed drive behaviour of the real car (mdp.AckermannActionCfg): car-side node V_DEAD / V_MIN, car #1's motors do
# not follow below 0.03 m/s (measured 2026-11-03, docs/open-issues.md §0, car/policy/explore_policy.py).
V_DEAD, V_MIN = 0.005, 0.03
STEER_AT_REST_PROB = 0.5         # unknown whether the car-side node can set the servo at v = 0 (research/02): both

# Car body relative to the rear-axle centre (model: chassis centred between the axles, mentorpi_a1.xml; 213 x 159 mm,
# research/01 §3; wheelbase 0.145 m, Hiwonder ackermann.py).
BODY_HALF_L, BODY_HALF_W = 0.1065, 0.0795
AXLE_X = WHEELBASE / 2           # rear axle behind the body origin
BODY_FRONT = BODY_HALF_L + AXLE_X
BODY_REAR = BODY_HALF_L - AXLE_X
BODY_DIAGONAL = 2 * math.hypot(BODY_HALF_L, BODY_HALF_W)    # 0.266 m


def turning_radius(steer: float) -> float:
  """Rear-axle-centre turning radius, Hiwonder manual 9.2: R = H / tan(theta)."""
  return WHEELBASE / math.tan(steer)


def uturn_width_formula(steer: float, radius: float | None = None) -> float:
  """Narrowest corridor for one full-lock 180 deg turn from a start along the corridor (see module docstring), with
  Hiwonder's radius or a given (e.g. measured) rear-axle radius."""
  r = (turning_radius(steer) if radius is None else radius) + BODY_HALF_W
  return math.hypot(BODY_FRONT, r) + math.hypot(BODY_REAR, r)


# Measured in sim on the nominal car (lab/tests/test_turnaround.py, 2026-11-03):
# - test_turning_radius: steady full-lock circles at 0.2 m/s; the steering joints reach the command, yet the rear-axle
#   centre circles 13% (29 deg) / 19% (37 deg) wider than H / tan(theta): the parallel-steered front wheels scrub
#   (both at the same angle, research/01 §3). CONFLICT with Hiwonder's idealised formula; the real radius is
#   unmeasured, so the plausible range is [H / tan(theta), TURN_RADIUS_SIM]. Measure on the car (open-issues #3).
# - test_single_uturn_width: narrowest corridor a full-lock sweep clears from the best start (1 cm off the outer wall,
#   steering set at rest, 0.2 m/s); matches uturn_width_formula with TURN_RADIUS_SIM + the 1 cm.
TURN_RADIUS_SIM = {29: 0.296, 37: 0.228}       # m, rear-axle centre
SINGLE_TURN_MEASURED = {29: 0.805, 37: 0.675}  # m
W_MIN = 0.40                     # m: > BODY_DIAGONAL + 0.13 m, multi-point turn measured feasible (test_turnaround.py)
W_MAX = 1.40                     # m: one sweep is enough for every build (> 0.805 m)
W_SPLIT = (SINGLE_TURN_MEASURED[29] + SINGLE_TURN_MEASURED[37]) / 2
CORRIDOR_LENGTH = (2.5, 4.0)     # m, dead end to back wall (our design)
START_GAP = (0.3, 1.5)           # m, car front to the nearest thing ahead (task spec)
START_YAW = math.radians(20.0)
START_SIDE_MARGIN = 0.03         # m, body to side wall at the start (our design)
TARGET_YAW = math.pi             # corridor axis, pointing out

# Walls: same height as the office walls (office_terrain.WALL_HEIGHT, above the LiDAR plane and the depth band top).
WALL_T = 0.10                    # m thick
SIDE_HALF_LEN = 2.6              # side walls span x in [-4.8, 0.4]
END_HALF_W = 0.85                # end / back walls span y in [-0.85, 0.85] (> W_MAX / 2 + WALL_T)
PARK = (-50.0, -50.0)            # env frame: unused obstacles

# Low obstacles. LiDAR scan plane: 0.14 m in our model (mentorpi_a1.xml, estimate) vs 0.1625 m in Hiwonder's URDF
# (docs/measurements/car1-2026-11-02.md §1): conflict, not measured; obstacle tops stay below the lower value. They
# must reach above the chassis bottom (0.0425 m in the model) so the chassis touches them (wheels do not collide).
POST_R = 0.015                   # m: v3 open-room post (chair / table leg) radius, ESTIMATE (floor object not measured)
LOW_OBSTACLE_PROB = 0.30         # task spec: ~30% of episodes
CHAIR_SHARE = 0.5                # of those: chair base, else low box
# Office-chair star base: ESTIMATE, no source (no standard fixes it; typical bases are ~0.6-0.7 m across): 5 legs
# to radius CHAIR_R, 0.05 m wide, top CHAIR_TOP minus a random sink of 0-CHAIR_SINK (top 0.07-0.11 m).
CHAIR_R, CHAIR_LEG_W, CHAIR_TOP, CHAIR_SINK = 0.32, 0.05, 0.11, 0.04
# Low box (a shoe box, a fallen bag): ESTIMATE, 0.24 x 0.34 m footprint, top 0.06-0.10 m.
BOX_HALF, BOX_TOP, BOX_SINK = (0.12, 0.17), 0.10, 0.04
OBSTACLE_SETBACK = (0.0, 0.4)    # m from the dead-end wall to the obstacle's far edge (our design)

# Success (task spec): heading within 10 deg, still (< 0.05 m/s) for 0.3 s, inside the corridor clear of the walls.
SUCCESS_YAW = math.radians(10.0)
SUCCESS_SPEED = 0.05
SUCCESS_HOLD_S = 0.3
SUCCESS_CLEARANCE = 0.03         # m, every body corner from every wall (our design)
PROXIMITY = 0.05                 # m: clearance below which the proximity penalty ramps up (our design)
SWITCH_V = V_MIN                 # m/s: commanded speeds at least this count as a direction (forward / reverse)

# Depth fan: the explore task's 12 bands over the measured 56.6 deg FOV (camera.MEASURED_HFOV), sampled denser so thin
# chair legs are not missed between rays (the car takes the minimum over every pixel column of a band,
# car/perception/depth_scan.py): DEPTH_SUB rays per band, elevations down to the measured FOV bottom (-21.95 deg).
DEPTH_SUB = 3
DEPTH_ELEVATIONS = tuple(math.radians(e) for e in (0.0, -2.0, -4.0, -6.0, -8.0, -10.0, -13.0, -16.0, -19.0, -21.95))

# Heading error noise: on the car the target is (yaw when the turn starts) + pi from odometry yaw. NO SOURCE for its
# error over a 20 s turn (IMU / odom drift unmeasured): per-episode bias R.LOC_BIAS_YAW (3 deg, estimate) + per-step
# std 0.5 deg (ESTIMATE).
HEADING_STD = math.radians(0.5)


# Entities: kinematic (mocap) walls and obstacles, collision group 2 like the office walls (car chassis only).


def _xml(name: str, geoms: str) -> str:
  return f'<mujoco model="{name}"><worldbody><body name="{name}">{geoms}</body></worldbody></mujoco>'


def _box(name: str, half: tuple[float, float, float], pos=(0.0, 0.0, 0.0), yaw_deg: float = 0.0,
         rgba: str = "0.94 0.93 0.90 1") -> str:
  # MJCF euler is in degrees (MuJoCo default angle unit).
  return (f'<geom name="{name}" type="box" size="{half[0]} {half[1]} {half[2]}" pos="{pos[0]} {pos[1]} {pos[2]}" '
          f'euler="0 0 {yaw_deg}" contype="2" conaffinity="2" rgba="{rgba}"/>')


def _chair_xml() -> str:
  leg = CHAIR_R / 2
  g = "".join(_box(f"leg{k}", (leg, CHAIR_LEG_W / 2, CHAIR_TOP / 2),
                   (leg * math.cos(math.radians(72 * k)), leg * math.sin(math.radians(72 * k)), CHAIR_TOP / 2),
                   72.0 * k, "0.12 0.12 0.12 1") for k in range(5))
  return _xml("chair", g)


_ENTITY_XML = {
  "wall_l": _xml("wall_l", _box("wall_l", (SIDE_HALF_LEN, WALL_T / 2, WALL_HEIGHT / 2), (0, 0, WALL_HEIGHT / 2))),
  "wall_r": _xml("wall_r", _box("wall_r", (SIDE_HALF_LEN, WALL_T / 2, WALL_HEIGHT / 2), (0, 0, WALL_HEIGHT / 2))),
  "wall_end": _xml("wall_end", _box("wall_end", (WALL_T / 2, END_HALF_W, WALL_HEIGHT / 2), (0, 0, WALL_HEIGHT / 2))),
  "wall_back": _xml("wall_back", _box("wall_back", (WALL_T / 2, END_HALF_W, WALL_HEIGHT / 2), (0, 0, WALL_HEIGHT / 2))),
  "chair": _chair_xml(),
  "lowbox": _xml("lowbox", _box("lowbox", (BOX_HALF[0], BOX_HALF[1], BOX_TOP / 2), (0, 0, BOX_TOP / 2),
                                rgba="0.74 0.55 0.32 1")),
  # v3: a thin post (chair / table leg) behind the car in open-room episodes
  "post": _xml("post", f'<geom name="post" type="cylinder" size="{POST_R} {WALL_HEIGHT / 2}" pos="0 0 {WALL_HEIGHT / 2}" '
                       f'contype="2" conaffinity="2" rgba="0.12 0.12 0.12 1"/>'),
}
NONE, CHAIR, BOX = 0, 1, 2


def _entity(name: str) -> EntityCfg:
  return EntityCfg(spec_fn=lambda: mujoco.MjSpec.from_string(_ENTITY_XML[name]))


@dataclass
class DenseDepthFanPatternCfg:
  """camera.DepthFanPatternCfg with `sub` rays per column band: ray order [band, sub, elevation], so
  camera.depth_sectors(elevations=elevations * sub) pools each band over all its rays."""
  hfov: float = C.MEASURED_HFOV
  elevations: tuple[float, ...] = DEPTH_ELEVATIONS
  sub: int = DEPTH_SUB

  def generate_rays(self, mj_model: mujoco.MjModel | None, device: str) -> tuple[torch.Tensor, torch.Tensor]:
    del mj_model
    n = C.DEPTH_COLUMNS * self.sub
    az = (torch.arange(n, device=device, dtype=torch.float32) + 0.5) / n * self.hfov - self.hfov / 2
    el = torch.tensor(self.elevations, device=device, dtype=torch.float32)
    a, e = torch.meshgrid(az, el, indexing="ij")
    dirs = torch.stack([torch.cos(e) * torch.cos(a), torch.cos(e) * torch.sin(a), torch.sin(e)], -1).reshape(-1, 3)
    return torch.zeros_like(dirs), dirs


# State.


@dataclass
class Turn:
  width: torch.Tensor      # [N] m
  length: torch.Tensor     # [N] m, dead end to back wall
  kind: torch.Tensor       # [N] NONE / CHAIR / BOX
  obs_xy: torch.Tensor     # [N, 2] obstacle centre, env frame
  obs_yaw: torch.Tensor    # [N]
  prev_err: torch.Tensor   # [N] |heading error| at the start of the step
  last_dir: torch.Tensor   # [N] last commanded direction, +1 / -1 (0 = none yet)
  switched: torch.Tensor   # [N] bool: this step's command reversed the direction
  still: torch.Tensor      # [N] consecutive steps meeting the success condition
  success: torch.Tensor    # [N] bool


def turn(env: ManagerBasedRlEnv) -> Turn:
  if not hasattr(env, "_robotlab_turn"):
    n, dev = env.num_envs, env.device
    z = lambda *s, dt=torch.float32: torch.zeros(n, *s, device=dev, dtype=dt)
    env._robotlab_turn = Turn(torch.full((n,), W_MAX, device=dev), torch.full((n,), CORRIDOR_LENGTH[1], device=dev),
                              z(dt=torch.long), z(2), z(), z(), z(), z(dt=torch.bool), z(dt=torch.long), z(dt=torch.bool))
  return env._robotlab_turn


def _put(env: ManagerBasedRlEnv, name: str, env_ids: torch.Tensor, xy: torch.Tensor, yaw: torch.Tensor,
         z: torch.Tensor | None = None) -> None:
  zz = torch.zeros_like(yaw) if z is None else z
  pos = torch.cat([xy, zz[:, None]], -1) + env.scene.env_origins[env_ids]
  env.scene[name].write_mocap_pose_to_sim(torch.cat([pos, yaw_quat(yaw)], -1), env_ids=env_ids)


def set_scene(env: ManagerBasedRlEnv, env_ids: torch.Tensor, width: torch.Tensor, length: torch.Tensor,
              car_xy: torch.Tensor, car_yaw: torch.Tensor, kind: torch.Tensor | None = None,
              obs_xy: torch.Tensor | None = None, obs_yaw: torch.Tensor | None = None,
              obs_sink: torch.Tensor | None = None, post_xy: torch.Tensor | None = None) -> None:
  """Move the walls, the obstacle and the car (all env frame) and clear the episode state. Also used by the tests."""
  t, m, dev = turn(env), len(env_ids), env.device
  zero = torch.zeros(m, device=dev)
  kind = torch.zeros(m, dtype=torch.long, device=dev) if kind is None else kind
  obs_xy = torch.zeros(m, 2, device=dev) if obs_xy is None else obs_xy
  obs_yaw = zero if obs_yaw is None else obs_yaw
  obs_sink = zero if obs_sink is None else obs_sink
  t.width[env_ids], t.length[env_ids], t.kind[env_ids] = width, length, kind
  t.obs_xy[env_ids], t.obs_yaw[env_ids] = obs_xy, obs_yaw
  side_x = torch.full((m,), 0.4 - SIDE_HALF_LEN, device=dev)
  _put(env, "wall_l", env_ids, torch.stack([side_x, width / 2 + WALL_T / 2], -1), zero)
  _put(env, "wall_r", env_ids, torch.stack([side_x, -width / 2 - WALL_T / 2], -1), zero)
  _put(env, "wall_end", env_ids, torch.stack([zero + WALL_T / 2, zero], -1), zero)
  _put(env, "wall_back", env_ids, torch.stack([-length - WALL_T / 2, zero], -1), zero)
  park = torch.tensor(PARK, device=dev).expand(m, 2)
  _put(env, "chair", env_ids, torch.where((kind == CHAIR)[:, None], obs_xy, park), obs_yaw, -obs_sink)
  _put(env, "lowbox", env_ids, torch.where((kind == BOX)[:, None], obs_xy, park), obs_yaw, -obs_sink)
  _put(env, "post", env_ids, park if post_xy is None else post_xy, zero)
  place_car(env, env_ids, car_xy, car_yaw)
  car = env.scene["car"]
  q = torch.zeros(m, car.data.joint_pos.shape[1], device=dev)
  car.write_joint_state_to_sim(q, q, env_ids=env_ids)          # wheels and steering at rest, straight
  t.prev_err[env_ids] = wrap(TARGET_YAW - car_yaw).abs()
  t.last_dir[env_ids] = 0.0
  t.switched[env_ids] = False
  t.still[env_ids] = 0
  t.success[env_ids] = False


# v2: the car node's obstacle stop in turn mode (explore_policy.py TURN_STOP_* / TURN_HALF_W), in the sim too: v1 never
# trained with it and on the floor it blocked v1's reverse-first turns (2026-11-04). Same arc rule as explore's.
TURN_STOP_FRONT, TURN_STOP_REAR, TURN_STOP_TIME, TURN_HALF_W = 0.22, 0.20, 0.3, 0.13
TRAIN_W_MIN = 0.75               # m, v2 (v1: W_MIN 0.40). With the car's turn-mode stop a scripted K-turn turns 0% at
                                 # 0.6 m, 62-87% at 0.7 m, 100% from 0.8 m (tmp_kturn_shield.py, 2026-11-04)
REAR_BLOCKED_PROB = 0.5          # v2: share of episodes (width >= REAR_BLOCKED_MIN_W) with the back wall close behind
REAR_BLOCKED_FRONT_GAP = 3.0     # m, v2c: with the rear blocked the front may be open (floor, drive 3: 1.7 m free ahead,
                                 # 0.1 m behind; v2/v2b only had a dead end <= 1.5 m ahead and kept reversing)
REAR_BLOCKED_MIN_W = 0.6         # m: narrower corridors keep the long free space behind (a box too small to turn in)
REAR_GAP = (0.05, 0.8)           # m from the car's rear to the back wall. v2b (v2: 0.25): on the floor the object was ~0.1 m behind


def turn_shield(env: ManagerBasedRlEnv, v: torch.Tensor, steer: torch.Tensor) -> torch.Tensor:
  return arc_blocked(env, v, steer, TURN_STOP_FRONT, TURN_STOP_REAR, TURN_STOP_TIME, TURN_HALF_W) & (v != 0)


# v3 open room (floor, drive 3, 2026-11-04: open floor around the car, ~1.7 m free ahead, one thin chair leg ~0.1 m
# behind; v2c, trained only in corridors, still reversed into it in the replay): side walls OPEN_W apart, back wall far,
# no dead-end obstacle, a POST_R post POST_GAP behind the car's rear.
OPEN_ROOM_PROB = 0.3             # share of episodes (design choice)
OPEN_W = 3.0                     # m between the side walls: open floor
OPEN_FRONT_GAP = (0.8, 3.0)      # m free ahead (drive 3: 1.7 m)
OPEN_Y = 0.5                     # m: lateral start within the end wall's span (END_HALF_W)
POST_GAP = (0.05, 0.30)          # m from the body's rear face to the post (drive 3: ~0.1 m)
POST_LAT = 0.08                  # m: lateral offset of the post from the car's axis, +-


def _uniform(lo, hi, m: int, dev) -> torch.Tensor:
  return lo + (hi - lo) * torch.rand(m, device=dev)


def reset_turnaround(env: ManagerBasedRlEnv, env_ids: torch.Tensor) -> None:
  """New corridor (width, length), maybe a low obstacle at the dead end, car facing in."""
  m, dev = len(env_ids), env.device
  width = _uniform(TRAIN_W_MIN, W_MAX, m, dev)      # v2 (v1: half below / half above W_SPLIT from W_MIN)
  # Obstacle near the dead end: chair (bounding radius CHAIR_R) or box (any yaw), clamped laterally into the corridor.
  r = torch.rand(m, device=dev)
  kind = torch.where(r < LOW_OBSTACLE_PROB * CHAIR_SHARE, CHAIR, torch.where(r < LOW_OBSTACLE_PROB, BOX, NONE))
  oyaw = _uniform(-math.pi, math.pi, m, dev)
  ext_x = torch.where(kind == CHAIR, CHAIR_R, BOX_HALF[0] * oyaw.cos().abs() + BOX_HALF[1] * oyaw.sin().abs())
  ext_y = torch.where(kind == CHAIR, CHAIR_R, BOX_HALF[0] * oyaw.sin().abs() + BOX_HALF[1] * oyaw.cos().abs())
  ox = -ext_x - _uniform(*OBSTACLE_SETBACK, m, dev)
  ylim = (width / 2 - ext_y).clamp(min=0.0)
  oy = ylim * (2 * torch.rand(m, device=dev) - 1)
  front = torch.where(kind == NONE, 0.0, ox - ext_x)          # nearest thing ahead of the car
  sink = torch.where(kind == CHAIR, _uniform(0.0, CHAIR_SINK, m, dev), _uniform(0.0, BOX_SINK, m, dev))
  # Car: front START_GAP from it, yaw +-START_YAW, lateral offset where the body fits with START_SIDE_MARGIN.
  yaw = _uniform(-START_YAW, START_YAW, m, dev)
  cx = BODY_HALF_L * yaw.cos() + BODY_HALF_W * yaw.sin().abs()
  cy = BODY_HALF_L * yaw.sin().abs() + BODY_HALF_W * yaw.cos()
  rear = (torch.rand(m, device=dev) < REAR_BLOCKED_PROB) & (width >= REAR_BLOCKED_MIN_W)
  gap = torch.where(rear, _uniform(START_GAP[0], REAR_BLOCKED_FRONT_GAP, m, dev), _uniform(*START_GAP, m, dev))
  x = front - gap - cx
  y = (width / 2 - cy - START_SIDE_MARGIN).clamp(min=0.0) * (2 * torch.rand(m, device=dev) - 1)
  length = torch.maximum(_uniform(*CORRIDOR_LENGTH, m, dev), -(x - cx) + 0.4)
  length = torch.where(rear, -(x - cx) + _uniform(*REAR_GAP, m, dev), length)
  # v3 open room: wide, no dead-end obstacle, free ahead, far back wall, a thin post just behind the car.
  open_ = torch.rand(m, device=dev) < OPEN_ROOM_PROB
  width = torch.where(open_, OPEN_W, width)
  kind = torch.where(open_, NONE, kind)
  rear = rear & ~open_
  x = torch.where(open_, -_uniform(*OPEN_FRONT_GAP, m, dev) - cx, x)
  y = torch.where(open_, OPEN_Y * (2 * torch.rand(m, device=dev) - 1), y)
  length = torch.where(open_, -(x - cx) + 1.5, length)
  u = torch.stack([yaw.cos(), yaw.sin()], -1)
  nrm = torch.stack([-yaw.sin(), yaw.cos()], -1)
  back = -(BODY_HALF_L + _uniform(*POST_GAP, m, dev) + POST_R)
  lat = POST_LAT * (2 * torch.rand(m, device=dev) - 1)
  post = torch.stack([x, y], -1) + back[:, None] * u + lat[:, None] * nrm
  post = torch.where(open_[:, None], post, torch.tensor(PARK, device=dev).expand(m, 2))
  t = turn(env)
  if not hasattr(t, "rear_blocked"):
    t.rear_blocked = torch.zeros(env.num_envs, dtype=torch.bool, device=dev)
    t.open_room = torch.zeros(env.num_envs, dtype=torch.bool, device=dev)
  t.rear_blocked[env_ids] = rear
  t.open_room[env_ids] = open_
  set_scene(env, env_ids, width, length, torch.stack([x, y], -1), yaw, kind, torch.stack([ox, oy], -1), oyaw, sink,
            post)
  R.reset_ray_noise(env, env_ids, LIDAR_RAYS)
  C.reset_camera(env, env_ids)


# Geometry helpers (ground truth).


def heading_error(env: ManagerBasedRlEnv) -> torch.Tensor:
  """wrap(target - yaw), rad."""
  return wrap(TARGET_YAW - car_pose(env)[1])


def body_corners(xy: torch.Tensor, yaw: torch.Tensor) -> torch.Tensor:
  """[N, 4, 2] body corners, env frame."""
  c, s = yaw.cos()[:, None], yaw.sin()[:, None]
  lx = torch.tensor([1.0, 1.0, -1.0, -1.0], device=xy.device) * BODY_HALF_L
  ly = torch.tensor([1.0, -1.0, 1.0, -1.0], device=xy.device) * BODY_HALF_W
  return torch.stack([xy[:, 0:1] + c * lx - s * ly, xy[:, 1:2] + s * lx + c * ly], -1)


def wall_clearance(env: ManagerBasedRlEnv, xy: torch.Tensor | None = None, yaw: torch.Tensor | None = None):
  """Smallest distance from a body corner to a wall face (negative = through it). For a rectangle in an axis-aligned
  corridor the corners are the extreme points."""
  t = turn(env)
  if xy is None:
    xy, yaw = car_pose(env)
  p = body_corners(xy, yaw)
  d = torch.stack([t.width[:, None] / 2 - p[..., 1].abs(), -p[..., 0], p[..., 0] + t.length[:, None]], -1)
  return d.flatten(1).amin(-1)


def obstacle_clearance(env: ManagerBasedRlEnv) -> torch.Tensor:
  """Distance from the body rectangle to the obstacle's bounding circle (1e3 without obstacle). Conservative."""
  t, (xy, yaw) = turn(env), car_pose(env)
  p = to_frame(t.obs_xy - xy, yaw).abs()
  half = torch.tensor([BODY_HALF_L, BODY_HALF_W], device=env.device)
  d = torch.linalg.norm((p - half).clamp(min=0.0), dim=-1)
  r = torch.where(t.kind == CHAIR, CHAIR_R, math.hypot(*BOX_HALF))
  return torch.where(t.kind == NONE, 1e3, d - r)


# Per policy step, before physics (AckermannAction leader_step hook).


def turnaround_step(env: ManagerBasedRlEnv) -> None:
  """Heading error at the start of the step (progress reward) and forward / reverse switches of the command."""
  t = turn(env)
  t.prev_err = heading_error(env).abs()
  v = env.action_manager.get_term("drive").v_cmd
  d = torch.where(v >= SWITCH_V, 1.0, torch.where(v <= -SWITCH_V, -1.0, 0.0))
  t.switched = (d != 0) & (t.last_dir != 0) & (d != t.last_dir)
  t.last_dir = torch.where(d != 0, d, t.last_dir)


# Observations.


def depth(env: ManagerBasedRlEnv, noisy: bool = False) -> torch.Tensor:
  return C.depth_sectors(env, noisy, elevations=DEPTH_ELEVATIONS * DEPTH_SUB)


def heading_obs(env: ManagerBasedRlEnv) -> torch.Tensor:
  """(sin, cos) of target heading - car heading. The car-side node: target = yaw when the turn is requested + pi."""
  e = heading_error(env)
  return torch.stack([e.sin(), e.cos()], -1)


def last_action(env: ManagerBasedRlEnv) -> torch.Tensor:
  """The previous policy output, clipped to [-1, 1] (the car-side node keeps its last command), with the speed the car
  executed: 0 where the obstacle stop held it (v2c; otherwise a blocked command looked unblocked and the policy
  repeated it forever, drive 3 replay 2026-11-04)."""
  a = env.action_manager.action.clamp(-1.0, 1.0).clone()
  a[:, 0] = torch.where(env.action_manager.get_term("drive").shielded, 0.0, a[:, 0])
  return a


def privileged(env: ManagerBasedRlEnv) -> torch.Tensor:
  """Critic only: corridor half-width, car position (dead end at x = 0), obstacle in the car frame and kind, clearances,
  the car's steering limit and whether it can steer at rest, last commanded direction."""
  t, (xy, yaw) = turn(env), car_pose(env)
  drive = env.action_manager.get_term("drive")
  rel = to_frame(t.obs_xy - xy, yaw).clamp(-3, 3) * (t.kind != NONE)[:, None]
  return torch.cat([
    t.width[:, None] / 2, xy.clamp(-5, 1), rel, (t.kind == CHAIR).float()[:, None], (t.kind == BOX).float()[:, None],
    wall_clearance(env).clamp(-0.1, 1)[:, None], obstacle_clearance(env).clamp(-0.1, 1)[:, None],
    drive.steer_max[:, None], drive._steer_at_rest.float()[:, None], t.last_dir[:, None],
  ], -1)


class HeadingNoiseModel(NoiseModel):
  """Rotates the (sin, cos) heading error by a per-episode bias plus per-step noise; stays a unit vector."""

  def __init__(self, cfg: HeadingNoiseCfg, num_envs: int, device: str):
    self.cfg, self._dev = cfg, device
    self._bias = torch.zeros(num_envs, device=device)
    self.reset()

  def reset(self, env_ids: torch.Tensor | slice | None = None) -> None:
    ids = slice(None) if env_ids is None else env_ids
    self._bias[ids] = self.cfg.bias * (2 * torch.rand_like(self._bias[ids]) - 1)

  def __call__(self, data: torch.Tensor) -> torch.Tensor:
    e = torch.atan2(data[:, 0], data[:, 1]) + self._bias + self.cfg.std * torch.randn_like(data[:, 0])
    return torch.stack([e.sin(), e.cos()], -1)


@dataclass(kw_only=True)
class HeadingNoiseCfg(R._NoiseModelCfg, class_type=HeadingNoiseModel):
  bias: float = R.LOC_BIAS_YAW
  std: float = HEADING_STD


# Rewards and terminations. mjlab multiplies every reward by the step dt (0.05 s); one-off events below divide by it,
# so their weight is the reward per event.


def bumped(env: ManagerBasedRlEnv) -> torch.Tensor:
  return env.scene["bump"].data.found[:, 0] > 0


def succeeded(env: ManagerBasedRlEnv) -> torch.Tensor:
  """Success condition held SUCCESS_HOLD_S. Called once per step by the termination manager (before the rewards)."""
  t = turn(env)
  ok = (heading_error(env).abs() < SUCCESS_YAW) & (env.scene["car"].data.root_link_lin_vel_b[:, 0].abs() < SUCCESS_SPEED) \
    & (wall_clearance(env) >= SUCCESS_CLEARANCE)
  t.still = torch.where(ok, t.still + 1, 0)
  t.success = t.still >= round(SUCCESS_HOLD_S / env.step_dt)
  return t.success


def heading_progress(env: ManagerBasedRlEnv) -> torch.Tensor:
  """Decrease of |heading error| this step, rad/s (weight = reward per rad)."""
  return (turn(env).prev_err - heading_error(env).abs()) / env.step_dt


def constant(env: ManagerBasedRlEnv) -> torch.Tensor:
  return torch.ones(env.num_envs, device=env.device)


def success_bonus(env: ManagerBasedRlEnv) -> torch.Tensor:
  """Per success: 1 + the fraction of the episode time left (1-2), per event."""
  left = 1.0 - env.episode_length_buf.float() / env.max_episode_length
  return turn(env).success.float() * (1.0 + left) / env.step_dt


def contact_event(env: ManagerBasedRlEnv) -> torch.Tensor:
  return bumped(env).float() / env.step_dt


def switch_event(env: ManagerBasedRlEnv) -> torch.Tensor:
  return turn(env).switched.float() / env.step_dt


def proximity(env: ManagerBasedRlEnv) -> torch.Tensor:
  """0 beyond PROXIMITY of every wall and the obstacle's bounding circle, 1 at contact."""
  d = torch.minimum(wall_clearance(env), obstacle_clearance(env))
  return ((PROXIMITY - d) / PROXIMITY).clamp(0.0, 1.0)


# Config.


def make_mentorpi_turnaround_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  actor_terms = {
    "car_motion": ObservationTermCfg(func=car_motion),
    "lidar_sectors": ObservationTermCfg(func=lidar_sectors, params={"noisy": True}),
    "depth": ObservationTermCfg(func=depth, params={"noisy": True}),
    "heading_error": ObservationTermCfg(func=heading_obs, noise=HeadingNoiseCfg()),
    "last_action": ObservationTermCfg(func=last_action),
  }
  critic_terms = {
    "car_motion": ObservationTermCfg(func=car_motion),
    "lidar_sectors": ObservationTermCfg(func=lidar_sectors),
    "depth": ObservationTermCfg(func=depth),
    "heading_error": ObservationTermCfg(func=heading_obs),
    "last_action": ObservationTermCfg(func=last_action),
    "privileged": ObservationTermCfg(func=privileged),
  }
  lidar = RayCastSensorCfg(
    name="lidar", frame=ObjRef(type="site", name="lidar", entity="car"),
    pattern=LidarPatternCfg(num_rays=LIDAR_RAYS), ray_alignment="yaw", max_distance=LIDAR_RANGE, debug_vis=play,
    include_geom_groups=(0, 1),
  )
  depth_cam = RayCastSensorCfg(
    name="depth", frame=ObjRef(type="site", name="depth_cam", entity="car"),
    pattern=DenseDepthFanPatternCfg(), ray_alignment="base", max_distance=C.DEPTH_MAX, debug_vis=False,
    include_geom_groups=(0, 1),
  )
  bump = ContactSensorCfg(   # the chassis touches only walls and obstacles (collision group 2)
    name="bump", primary=ContactMatch(mode="geom", pattern="chassis", entity="car"), secondary=None,
    fields=("found",), reduce="none", num_slots=1,
  )
  cfg = ManagerBasedRlEnvCfg(
    scene=SceneCfg(
      terrain=TerrainEntityCfg(terrain_type="plane"),
      entities={"car": car_cfg(), **{k: _entity(k) for k in _ENTITY_XML}},
      sensors=(lidar, depth_cam, bump),
      num_envs=1,
      env_spacing=6.0,
    ),
    observations={
      "actor": ObservationGroupCfg(actor_terms, enable_corruption=True),
      "critic": ObservationGroupCfg(critic_terms),
    },
    actions={"drive": AckermannActionCfg(entity_name="car", leader_step=turnaround_step, v_reverse=TURN_V_REVERSE,
                                         v_max=TURN_V_MAX, steer_max_range=(STEER_2024, STEER_CODE),
                                         v_dead=V_DEAD, v_min=V_MIN, steer_at_rest_prob=STEER_AT_REST_PROB,
                                         shield=turn_shield)},
    events={
      "reset_joints": EventTermCfg(
        func=reset_joints_by_offset, mode="reset",
        params={"position_range": (0.0, 0.0), "velocity_range": (0.0, 0.0), "asset_cfg": SceneEntityCfg("car")},
      ),
      "lidar_rate": EventTermCfg(func=cast_lidar_every_other_step, mode="startup"),   # real MS200: 10 Hz
      "reset_turnaround": EventTermCfg(func=reset_turnaround, mode="reset"),
    },
    rewards={
      # Effective values (after mjlab's dt scaling): a full 180 deg turn earns 3 * pi = 9.4; success 10-20 (more when
      # faster); time -0.3 per s; contact -20 (= weight -400 in the explore task's convention, where -50 is -2.5).
      "heading_progress": RewardTermCfg(func=heading_progress, weight=3.0),
      "success": RewardTermCfg(func=success_bonus, weight=10.0),
      "time": RewardTermCfg(func=constant, weight=-0.6),               # v2 (v1 -0.3): faster turns
      "contact": RewardTermCfg(func=contact_event, weight=-20.0),
      "switch": RewardTermCfg(func=switch_event, weight=-1.0),          # per forward <-> reverse switch; v2 (v1 -0.5)
      "shield_stop": RewardTermCfg(func=shield_stop, weight=-3.0),       # v2: as explore v5 (CBF-RL: penalise interventions)
      "proximity": RewardTermCfg(func=proximity, weight=-1.0),
      "action_rate": RewardTermCfg(func=action_rate_l2, weight=-0.5),   # as explore v3 (real car dithered at -0.05)
    },
    terminations={
      "time_out": TerminationTermCfg(func=time_out, time_out=True),
      "bump": TerminationTermCfg(func=bumped),
      "success": TerminationTermCfg(func=succeeded),
    },
    viewer=ViewerConfig(
      origin_type=ViewerConfig.OriginType.ASSET_BODY, entity_name="car", body_name="chassis",
      distance=2.0, elevation=60.0, azimuth=180.0,
    ),
    sim=SimulationCfg(nconmax=64, njmax=160, mujoco=MujocoCfg(timestep=0.005, integrator="implicitfast")),
    decimation=10,               # 20 Hz policy
    episode_length_s=20.0,
  )
  R.apply_domain_randomization(cfg)
  if play:
    cfg.observations["actor"].enable_corruption = False
  return cfg


MentorPiTurnaroundRlCfg = RslRlOnPolicyRunnerCfg(
  actor=RslRlModelCfg(
    hidden_dims=(256, 256, 128), activation="elu", obs_normalization=True,
    distribution_cfg={"class_name": "GaussianDistribution", "init_std": 0.6, "std_type": "scalar"},
  ),
  critic=RslRlModelCfg(hidden_dims=(256, 256, 128), activation="elu", obs_normalization=True),
  algorithm=RslRlPpoAlgorithmCfg(
    value_loss_coef=1.0, use_clipped_value_loss=True, clip_param=0.2, entropy_coef=0.005,
    num_learning_epochs=5, num_mini_batches=4, learning_rate=1.0e-3, schedule="adaptive",
    gamma=0.99, lam=0.95, desired_kl=0.01, max_grad_norm=1.0,
  ),
  experiment_name="turnaround",
  logger="tensorboard",
  save_interval=50,
  num_steps_per_env=32,
  max_iterations=1500,
)
