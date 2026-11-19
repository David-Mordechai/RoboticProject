"""Explore task: one MentorPi A1 explores an unknown office floor on its own, building a map as it drives, among walking
people. It is rewarded for newly seen floor and must not touch walls, clutter or people.

What the car sees (the real car can build all of it): the LiDAR (36 min-pooled sectors), the depth camera (12 column
bands, feet and low clutter below the LiDAR plane), its own motion, and two car-frame crops of the map it has built so
far, at its ESTIMATED pose: a fine crop around the car and a coarse one that shows where unexplored space is. On the
real car the map is slam_toolbox's /map (0.05 m, -1 unknown / 0 free / 100 occupied) in the SLAM frame, sampled the
same way. There is no frontier hint: the policy learns where to go from the coarse crop.

Built map (the sim's stand-in for SLAM), per env on a SEEN_RES grid over the floor patch:
- Every LiDAR cast, along every 2nd of the 360 rays, cells before the return are marked free, the return's cell
  occupied (no return: free up to LIDAR_RANGE). Line of sight comes from the ray itself: a ray stops at the first
  wall, so nothing behind a wall is marked. APPROXIMATIONS: noise-free ranges (SLAM averages noise out); samples every
  SEEN_RES along the ray, so a cell grazed by a ray can be missed; occupied wins over free once set (no log-odds
  decay); a person in the beam marks an occupied cell (as on the real map).
- Reward: newly free cells, as area (m^2), after the first scan of an episode (the start scan is free).
- Coverage (evaluation): free cells seen / floor cells of the patch (cells not in the ground-truth occupancy).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

import torch

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs.mdp import reset_joints_by_offset, time_out
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

from robotlab.robot.mentorpi_a1_constants import STEER_2024, STEER_CODE, WHEELBASE, car_cfg, person_cfg
from robotlab.tasks import camera as C
from robotlab.tasks import localization as Lz
from robotlab.tasks import randomization as R
from robotlab.tasks.lidar_rate import cast_lidar_every_other_step
from robotlab.tasks.mdp import LidarPatternCfg, AckermannActionCfg, car_motion, car_pose, place_car, to_frame
from robotlab.tasks.mentorpi_formation_env_cfg import HALL_WIDTH_RANGE
from robotlab.tasks.mentorpi_navigate_env_cfg import CAR_RADIUS, LIDAR_RANGE, LIDAR_RAYS, PERSON_RADIUS, \
  PERSONAL_SPACE, PERSONAL_ZONE, lidar_sectors
from robotlab.tasks.office_terrain import NODES, OCC_N, OCC_RES, PATCH_SIZE, LayoutTensors, OfficeTerrainCfg, \
  layout_tensors
from robotlab.tasks.people import write_walkers, MAX_PEOPLE, STAND_PROB, people_size_events, reset_walkers, step_walkers, walkers

if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv

OFFICE_ROWS = OFFICE_COLS = 4
# Speed: the stock joystick limit, 0.7 m/s (research/01 sec. 3); the car-side node drives /controller/cmd_vel, which is
# uncapped (research/02; only the app's /cmd_vel is clipped to 0.2 m/s). v2 trained at 0.2 m/s and averaged 0.12 m/s:
# far too slow (user, 2026-11-03). v3 = v1's speeds with harsher contact penalties (v1: ~5% of runs ended in contact).
EXPLORE_V_MAX = 0.7
EXPLORE_V_REVERSE = 0.3
# v4e: max change of the normalized steering command per policy step (20 Hz): 0.1 -> centre to full lock in 0.5 s,
# lock to lock in 1 s. Design choice, not a spec (the servo itself is far faster: 0.16 s/60 deg). v3-v4c steered
# 0.11-0.15 per step on average while driving straight (sim), which looks like constant wobbling.
EXPLORE_STEER_CMD_RATE = 0.1
# v5: the LIDAR_HIST scans before the current one in the actor observation (scans are 0.1 s apart: the MS200 runs at
# 10 Hz, lidar_rate.py), so with the current scan the policy sees 0.5 s. DRL-VO (Xie & Dames, T-RO 2023,
# arXiv 2301.06512): a 0.5 s window of 10 scans worked best; the current scan alone lacks motion, older data did not
# help. v4e/v4f (current scan only) kept touching people in ~16-22% of runs whatever the penalty (-60 or -300).
LIDAR_HIST = 4
# v5 shield: the car-side node's obstacle stop (car/policy/explore_policy.py blocked(), explore mode), applied in the
# sim so the policy trains with the stop it will meet on the car. Same values as there (ESTIMATES, see that file).
SHIELD_HALF_W, SHIELD_FRONT, SHIELD_REAR, SHIELD_TIME = 0.14, 0.30, 0.35, 0.4
# SHIELD_HALF_W 0.14 since goto v6p (2026-11-06; was 0.10 = half car width + 2 cm): goto v6o 8600 at-fault person contacts
# were 58% standing people at 30-90 deg clipped by the car's corner, the stop silent (LiDAR sees legs, feet reach up to
# 0.19 m further, sim person.xml). Eval on 8600: at-fault contacts 1.4% -> 0.6-0.9%. Design choice; car: HALF_W.
SHIELD_HALF_W_REV = 0.10   # m when reversing (goto v6q): with 0.14 both ways boxed-in cars could not back up at all (v6p
                           # 8750: 2% of blocked starts held up to 29 s); contacts during back-ups were 5% (v6o 8600)
SHIELD_MIN_Z = 0.02     # m: depth-fan hits lower than this are the floor, not obstacles
# v5 approaching-object stop (part of the shield, same on the car): within SHIELD_APPROACH_HALF_ANGLE of the driving
# direction and closer than SHIELD_APPROACH_RANGE, an object that itself approaches faster than SHIELD_APPROACH_SPEED
# (range rate between the last two scans minus the car's own v cos(bearing)) stops the car; a person then walks around
# the stopped car. Design choices: v5w person contacts stayed at ~9-12% with the learned policy alone (65% head-on).
SHIELD_APPROACH_RANGE = 1.5         # m (1.2 until v5w 1650: the stop fired with the car still at 0.3-0.55 m/s)
SHIELD_APPROACH_SPEED = 0.6         # m/s (0.4 until v5w 1750); must show in two scans in a row (see approaching())
SHIELD_APPROACH_HALF_ANGLE = 60.0   # deg (40: side approaches at ~50 deg got through)
SCAN_DT = 0.1                       # s between LiDAR scans (10 Hz, lidar_rate.py)
# v7: the approach stop only for objects in the car's path (within SHIELD_APPROACH_HALF_W of its axis), no hold. v6 2649
# with the v5 cone + 5 s hold: 85% of the idle time with a clear path was this stop, firing 20% of the time with no
# person within 2 m (walls while turning); people beside cut speed to 0.85 (tmp_idle_cause.py). The policy has to keep
# clear of standing people itself (hit_person, person_ttc).
SHIELD_APPROACH_HALF_W = 0.35       # m: car half width 0.08 + person 0.20 + margin; design choice
# v5 anticipatory person reward (DRL-VO's velocity-obstacle idea): constant-velocity prediction over TTC_HORIZON;
# penalised when the car (moving) is on course to pass a person closer than TTC_MARGIN, more the sooner.
TTC_HORIZON = 2.0       # s: design choice
TTC_MARGIN = 0.15       # m clearance: design choice
SEEN_RES = 0.10                  # m per built-map cell (our design; slam_toolbox uses 0.05, slam.yaml)
SEEN_N = int(round(PATCH_SIZE / SEEN_RES))
# Depth obstacles in the built map (goto v7, 2026-11-08). The LiDAR plane misses low objects (chair bases: floor goto17
# hit a caster) and the depth camera only sees 57 deg ahead, so the policy forgot a chair leg as soon as it turned. Depth
# hits (FLOOR_Z < z < BAND_TOP, within DEPTH_MAP_RANGE) mark their cell occupied for DEPTH_MAP_S; the car-side node does
# the same (car/policy/explore_obs.DepthLayer). Design choices: 8 s covers a back-and-forth past an obstacle; 1.5 m is
# where the HP60C is best (< 2 m, research/03) and the car's old stop memory range (explore_policy.MEM_RANGE).
DEPTH_MAP_S, DEPTH_MAP_RANGE = 8.0, 1.5
DEPTH_SUB = 4                    # depth fan columns per band (camera.DepthFanPatternCfg.sub)
RAY_STRIDE = 2                   # mark along every 2nd LiDAR ray: 2 deg apart, 0.10 m apart at 3 m
_SAMPLES = int(round(LIDAR_RANGE / SEEN_RES))
# Map crops (our design). Fine: like the formation task's saved-map crop (localization.py MAP_*), more ahead than behind.
FINE_RES, FINE_X, FINE_Y = 0.15, (-0.9, 2.1), (-1.5, 1.5)
FINE_NX = int(round((FINE_X[1] - FINE_X[0]) / FINE_RES))      # 20
FINE_NY = int(round((FINE_Y[1] - FINE_Y[0]) / FINE_RES))      # 20
# Coarse: 12 x 12 cells of 0.6 m centred on the car (7.2 m square); each cell = fraction of known-free and of
# known-occupied over a 3 x 3 grid of sample points.
COARSE_RES, COARSE_N, COARSE_SUB = 0.6, 12, 3
_DIR_VEC = ((1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0))
_DIR_YAW = (0.0, math.pi, math.pi / 2, -math.pi / 2)


@dataclass
class Explore:
  maps: LayoutTensors
  layout: torch.Tensor     # [N] office floor of each env
  floor: torch.Tensor      # [L, SEEN_N, SEEN_N] bool: no wall / clutter in the cell (ground truth, coarsened)
  free: torch.Tensor       # [N, SEEN_N, SEEN_N] bool: seen free
  occ: torch.Tensor        # [N, SEEN_N, SEEN_N] bool: seen occupied
  free_count: torch.Tensor # [N] cells seen free
  new_area: torch.Tensor   # [N] m^2 newly seen free in the last update (0 on the first scan of an episode)
  primed: torch.Tensor     # [N] bool: the episode's first scan has been taken
  dseen: torch.Tensor      # [N, SEEN_N, SEEN_N] int16: episode step a depth hit last marked the cell (goto v7)


def explore(env: ManagerBasedRlEnv) -> Explore:
  if not hasattr(env, "_robotlab_explore"):
    terrain, n, dev = env.scene.terrain, env.num_envs, env.device
    num_cols = terrain.terrain_origins.shape[1]
    maps = layout_tensors(terrain.terrain_origins.shape[0] * num_cols, dev)
    # A built-map cell is floor if none of the 0.05 m occupancy cells inside it is occupied.
    k = int(round(SEEN_RES / OCC_RES))
    occ = maps.occ[:, :SEEN_N * k, :SEEN_N * k].float()
    floor = torch.nn.functional.max_pool2d(occ[:, None], k)[:, 0] < 0.5
    z = lambda: torch.zeros(n, SEEN_N, SEEN_N, dtype=torch.bool, device=dev)
    env._robotlab_explore = Explore(maps, terrain.terrain_levels * num_cols + terrain.terrain_types, floor, z(), z(),
                                    torch.zeros(n, device=dev), torch.zeros(n, device=dev),
                                    torch.zeros(n, dtype=torch.bool, device=dev),
                                    torch.full((n, SEEN_N, SEEN_N), _NEVER, dtype=torch.int16, device=dev))
    Lz.localization(env, agents=1)       # one car: create the localization state with one agent
  return env._robotlab_explore


_NEVER = -30000


def update_depth_map(env: ManagerBasedRlEnv) -> None:
  """Mark the cells of this step's depth hits (true ranges, as the LiDAR map) with the episode step (DEPTH_MAP_S)."""
  e, n = explore(env), env.num_envs
  data = env.scene["depth"].data
  d, hw = data.distances, data.hit_pos_w
  rel = hw[..., :2] - env.scene.env_origins[:, None, :2]
  xy, _ = car_pose(env)
  ok = (d >= C.DEPTH_MIN) & (hw[..., 2] > C.FLOOR_Z) & (hw[..., 2] < C.BAND_TOP)
  ok &= torch.linalg.norm(rel - xy[:, None], dim=-1) <= DEPTH_MAP_RANGE
  ok &= (env.episode_length_buf >= 2)[:, None]       # as update_built_map: no stale pre-teleport hits
  idx, inside = _cells(rel)
  ok &= inside
  rows = torch.arange(n, device=env.device)[:, None].expand_as(idx)
  step = env.episode_length_buf.to(torch.int16)[:, None].expand_as(idx)
  e.dseen.view(n, -1)[rows[ok], idx[ok]] = step[ok]


def _cells(xy: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
  """Env-frame points -> flat built-map cell index and inside-the-patch mask."""
  ij = (xy / SEEN_RES).floor().long()
  inside = (ij >= 0).all(-1) & (ij < SEEN_N).all(-1)
  ij = ij.clamp(0, SEEN_N - 1)
  return ij[..., 0] * SEEN_N + ij[..., 1], inside


def update_built_map(env: ManagerBasedRlEnv) -> None:
  """Mark free / occupied cells along the LiDAR rays (true ranges) from the car's true pose."""
  e, n = explore(env), env.num_envs
  xy, yaw = car_pose(env)
  d = env.scene["lidar"].data.distances[:, ::RAY_STRIDE]                          # [N, R], -1 = no return
  hit = (d > 0) & (d < LIDAR_RANGE)
  reach = torch.where(hit, d, LIDAR_RANGE)
  a = yaw[:, None] + torch.arange(0, LIDAR_RAYS, RAY_STRIDE, device=env.device) * (2 * math.pi / LIDAR_RAYS)
  u = torch.stack([torch.cos(a), torch.sin(a)], -1)                               # [N, R, 2]
  s = (torch.arange(_SAMPLES, device=env.device) + 0.5) * SEEN_RES                # [S]
  pts = xy[:, None, None] + s[None, None, :, None] * u[:, :, None]               # [N, R, S, 2]
  idx, inside = _cells(pts)
  # The first steps after a reset may still hold the scan from before the car was teleported (the LiDAR is cast
  # every other step): map only from the second step of an episode on.
  valid = (env.episode_length_buf >= 2)[:, None]
  free_m = inside & (s[None, None] < reach[..., None] - SEEN_RES / 2) & valid[..., None]
  rows = torch.arange(n, device=env.device)[:, None, None].expand_as(idx)
  e.free.view(n, -1)[rows[free_m], idx[free_m]] = True
  hidx, hin = _cells(xy[:, None] + (reach + SEEN_RES / 4)[..., None] * u)          # just inside the obstacle
  hm = hit & hin & valid
  hrows = torch.arange(n, device=env.device)[:, None].expand_as(hidx)
  e.occ.view(n, -1)[hrows[hm], hidx[hm]] = True
  e.free &= ~e.occ
  count = e.free.view(n, -1).sum(-1).float()
  e.new_area = torch.where(e.primed, (count - e.free_count).clamp(min=0.0) * SEEN_RES ** 2, 0.0)
  e.free_count = count
  e.primed |= count > 0


def explore_step(env: ManagerBasedRlEnv) -> None:
  """Per policy step, before physics: localization drift, people, built map (when a new scan was cast)."""
  e = explore(env)
  xy, yaw = car_pose(env)
  Lz.step_localization(env, torch.cat([xy, yaw[:, None]], -1)[:, None])
  step_walkers(env, walkers(env, e.maps, e.layout), xy, pass_by=True,   # v4e: people don't stop for an approaching car
               car_vel=env.scene["car"].data.root_link_lin_vel_w[:, :2])
  if getattr(env, "_robotlab_lidar_cast", True):
    update_built_map(env)
  else:
    e.new_area = torch.zeros_like(e.new_area)
  if getattr(env, "_robotlab_depth_map", False):    # goto v7 (the camera is fresh every step, camera.DEPTH_FRESH_P)
    update_depth_map(env)


# Events.


def reset_explore(env: ManagerBasedRlEnv, env_ids: torch.Tensor) -> None:
  """Car 0.6 m into a random hallway, heading along it +- 0.3 rad; empty built map; new people."""
  e, n, dev = explore(env), len(env_ids), env.device
  k = e.layout[env_ids]
  a = torch.randint(0, NODES * NODES, (n,), device=dev)
  nbr = e.maps.nbr[k, a]
  d = torch.multinomial((nbr >= 0).float(), 1)[:, 0]
  xy = e.maps.node_xy[k, a] + 0.6 * torch.tensor(_DIR_VEC, device=dev)[d]
  yaw = torch.tensor(_DIR_YAW, device=dev)[d] + 0.3 * (2 * torch.rand(n, device=dev) - 1)
  from robotlab.tasks import office_twin as _TW
  if _TW.TWIN_PATH:                       # office twin (2026-11-09): the charger start of the floor runs
    xy, yaw = _TW.start_pose(n, dev)
  place_car(env, env_ids, xy, yaw)
  env._robotlab_start = (a, d, xy)       # start node, hallway direction, position (the goto task plans from them)
  Lz.reset_localization(env, env_ids, torch.cat([xy, yaw[:, None]], -1)[:, None])
  e.free[env_ids] = False
  e.occ[env_ids] = False
  e.free_count[env_ids] = 0.0
  e.new_area[env_ids] = 0.0
  e.primed[env_ids] = False
  e.dseen[env_ids] = _NEVER
  car_xy = torch.zeros(env.num_envs, 2, device=dev)
  car_xy[env_ids] = xy
  reset_walkers(env, walkers(env, e.maps, e.layout), env_ids, car_xy, stand_prob=STAND_PROB)
  if _TW.TWIN_PATH:                       # no people in the twin (none on the floor route in goto17-22)
    w_ = walkers(env, e.maps, e.layout)
    w_.active[env_ids] = False
    write_walkers(env, w_, env_ids)
  R.reset_ray_noise(env, env_ids, LIDAR_RAYS)
  C.reset_camera(env, env_ids)


# Observations (car frame).


def own_pose(env: ManagerBasedRlEnv, est: bool) -> tuple[torch.Tensor, torch.Tensor]:
  xy, yaw = car_pose(env)
  return Lz.estimated(env, 0, xy, yaw) if est else (xy, yaw)


def _lookup(e: Explore, xy: torch.Tensor, yaw: torch.Tensor, px: torch.Tensor, py: torch.Tensor,
            ep: torch.Tensor | None = None):
  """Built-map state at car-frame points (px, py) [G]: known-free, known-occupied, [N, G] each (outside = occupied).
  ep [N] (episode step, goto v7): depth hits of the last DEPTH_MAP_S count as occupied, and not free."""
  c, s = torch.cos(yaw)[:, None], torch.sin(yaw)[:, None]
  w = torch.stack([xy[:, 0:1] + c * px - s * py, xy[:, 1:2] + s * px + c * py], -1)
  idx, inside = _cells(w)
  n = xy.shape[0]
  free = torch.gather(e.free.view(n, -1), 1, idx) & inside
  occ = torch.gather(e.occ.view(n, -1), 1, idx) | ~inside
  if ep is not None:
    age = ep[:, None].to(torch.int32) - torch.gather(e.dseen.view(n, -1), 1, idx).to(torch.int32)
    recent = (age >= 0) & (age < round(DEPTH_MAP_S / 0.05)) & inside      # 0.05 s policy step (decimation 10 x 5 ms)
    occ, free = occ | recent, free & ~recent
  return free.float(), occ.float()


def _ep(env: ManagerBasedRlEnv) -> torch.Tensor | None:
  return env.episode_length_buf if getattr(env, "_robotlab_depth_map", False) else None


def built_map_fine(env: ManagerBasedRlEnv, est: bool = False) -> torch.Tensor:
  """FINE_NX x FINE_NY car-frame crop of the built map: [known-free..., known-occupied...] (unknown = 0, 0)."""
  e, dev = explore(env), env.device
  gx = FINE_X[0] + FINE_RES * (torch.arange(FINE_NX, device=dev) + 0.5)
  gy = FINE_Y[0] + FINE_RES * (torch.arange(FINE_NY, device=dev) + 0.5)
  px, py = torch.meshgrid(gx, gy, indexing="ij")
  free, occ = _lookup(e, *own_pose(env, est), px.reshape(-1), py.reshape(-1), _ep(env))
  return torch.cat([free, occ], -1)


def built_map_coarse(env: ManagerBasedRlEnv, est: bool = False) -> torch.Tensor:
  """COARSE_N x COARSE_N car-frame cells of COARSE_RES centred on the car: fraction known-free, fraction known-occupied
  (each over COARSE_SUB x COARSE_SUB sample points). Unexplored space = cells with both fractions low."""
  e, dev = explore(env), env.device
  sub = (torch.arange(COARSE_SUB, device=dev) + 0.5) / COARSE_SUB
  g = (torch.arange(COARSE_N, device=dev)[:, None] + sub[None]).reshape(-1) * COARSE_RES - COARSE_N * COARSE_RES / 2
  px, py = torch.meshgrid(g, g, indexing="ij")                                     # [N*S, N*S]
  free, occ = _lookup(e, *own_pose(env, est), px.reshape(-1), py.reshape(-1), _ep(env))
  pool = lambda t: t.view(-1, COARSE_N, COARSE_SUB, COARSE_N, COARSE_SUB).mean((2, 4)).reshape(t.shape[0], -1)
  return torch.cat([pool(free), pool(occ)], -1)


def depth(env: ManagerBasedRlEnv, noisy: bool = False) -> torch.Tensor:
  return C.depth_sectors(env, noisy, elevations=C.MEASURED_ELEVATIONS)


def lidar_history(env: ManagerBasedRlEnv, noisy: bool = False) -> torch.Tensor:
  """v5: the LIDAR_HIST scans before the current one, newest first, each as lidar_sectors (36 values). A scan is
  pushed when the LiDAR casts a new one (every other policy step, 10 Hz); right after a reset every slot holds the
  current scan. The car-side node keeps the same buffer of /scan_raw messages (car/policy/explore_obs.py)."""
  key = "_robotlab_lidar_hist_noisy" if noisy else "_robotlab_lidar_hist"
  cur = lidar_sectors(env, noisy)
  st = getattr(env, key, None)
  if st is None:
    st = {"buf": cur[:, None].repeat(1, LIDAR_HIST + 1, 1), "step": -1}
    setattr(env, key, st)
  if st["step"] != env.common_step_counter:
    st["step"] = env.common_step_counter
    if getattr(env, "_robotlab_lidar_cast", True):
      st["buf"] = torch.cat([cur[:, None], st["buf"][:, :-1]], 1)
  fresh = env.episode_length_buf <= 1
  st["buf"][fresh] = cur[fresh, None]
  return st["buf"][:, 1:].flatten(1)


def _car_frame_points(env: ManagerBasedRlEnv, sensor: str, min_z: float | None = None) -> tuple[torch.Tensor, torch.Tensor]:
  """Hit points of a ray sensor in the car frame [N, R, 2] and their valid mask [N, R]."""
  d = env.scene[sensor].data
  xy, yaw = car_pose(env)
  origin = env.scene.env_origins[:, :2]
  rel = d.hit_pos_w[..., :2] - origin[:, None] - xy[:, None]
  c, s = torch.cos(yaw)[:, None], torch.sin(yaw)[:, None]
  pts = torch.stack([c * rel[..., 0] + s * rel[..., 1], -s * rel[..., 0] + c * rel[..., 1]], -1)
  ok = d.distances > 0
  if min_z is not None:
    ok &= d.hit_pos_w[..., 2] > min_z
  return pts, ok


def arc_blocked(env: ManagerBasedRlEnv, v: torch.Tensor, steer: torch.Tensor, front: float, rear: float,
                time_s: float, half_w: float) -> torch.Tensor:
  """True where a LiDAR or depth point lies within half_w of the arc the car drives at this steering, from the car out
  to front (+ time_s * v) ahead, or rear behind when reversing: explore_policy.blocked() (also the turnaround's)."""
  lp, lok = _car_frame_points(env, "lidar")
  dp, dok = _car_frame_points(env, "depth", SHIELD_MIN_Z)
  pts, ok = torch.cat([lp, dp], 1), torch.cat([lok, dok], 1)
  x, y = pts[..., 0] * torch.where(v < 0, -1.0, 1.0)[:, None], pts[..., 1]
  reach = (torch.where(v > 0, front, rear) + time_s * v.abs())[:, None]
  k = (torch.tan(steer) / WHEELBASE)[:, None]
  straight = (x > 0) & (x < reach) & (y.abs() < half_w)
  r = 1.0 / torch.where(k.abs() < 1e-3, 1e-3, k)
  dc = torch.hypot(x, y - r)
  s = r.abs() * torch.remainder(torch.atan2(x, torch.sign(r) * (r - y)), 2 * math.pi)
  arc = ((dc - r.abs()).abs() < half_w) & (s > 0) & (s < reach)
  return (torch.where(k.abs() < 1e-3, straight, arc) & ok).any(-1)


# Manoeuvre mode (2026-11-08): while a slow multi-point turn runs (env._robotlab_manoeuvre, set by the teacher / the
# manoeuvre skill), the stop uses the car node's turn-mode reaches (car/policy/explore_policy.py TURN_STOP_FRONT /
# TURN_STOP_REAR / TURN_STOP_TIME / TURN_HALF_W, 2026-11-04) instead of the driving ones; without it, U-turns in 1.0-1.2 m
# halls took 11 s in tiny legs and 18% did not finish (tmp_teacher_uturn.py). No approaching-object stop in the mode,
# as the car node's turn mode.
TURN_FRONT, TURN_REAR, TURN_TIME, TURN_HALF_W = 0.22, 0.20, 0.3, 0.13


# Reversing into remembered low obstacles (2026-11-08): behind the car only the LiDAR sees, and chair bases are under its
# plane; the depth camera saw them on the way in. When reversing, the stop also checks the depth map (DEPTH_MAP_S) on a
# grid along the straight rear corridor. Teacher v15: most chair hits were back-ups into legs. Car: explore_policy blocked().
_REAR_X = torch.arange(1, 8) * -0.05          # m behind base_footprint, to 0.35 (= SHIELD_REAR)
_REAR_Y = torch.tensor([-0.08, 0.0, 0.08])    # m, the car's half width 0.08 (research/01 sec. 2)


def rear_depth_blocked(env: ManagerBasedRlEnv) -> torch.Tensor:
  if not getattr(env, "_robotlab_depth_map", False):
    return torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
  e, dev = explore(env), env.device
  px, py = torch.meshgrid(_REAR_X.to(dev), _REAR_Y.to(dev), indexing="ij")
  xy, yaw = car_pose(env)
  c, s_ = torch.cos(yaw)[:, None], torch.sin(yaw)[:, None]
  w = torch.stack([xy[:, 0:1] + c * px.reshape(-1) - s_ * py.reshape(-1), xy[:, 1:2] + s_ * px.reshape(-1) + c * py.reshape(-1)], -1)
  idx, inside = _cells(w)
  n = env.num_envs
  age = env.episode_length_buf[:, None].to(torch.int32) - torch.gather(e.dseen.view(n, -1), 1, idx).to(torch.int32)
  return ((age >= 0) & (age < round(DEPTH_MAP_S / 0.05)) & inside).any(-1)


# Reverse arc against remembered depth hits (2026-11-09, car skill): the depth camera looks forward only and the LiDAR
# plane is above chair legs, so a reverse leg of the multi-point turn is blind to them; 28 of 50 chair hits of the car
# skill were in the turn. Rear body points (base_footprint frame, ESTIMATE: car half width 0.08 m, research/01 sec. 2;
# rear overhang unmeasured, 0.05-0.15 m sampled) along the reverse arc out to MEM_REV_REACH. = car explore_policy.
MEM_REV_REACH = 0.30
_MEM_S = torch.linspace(0.05, MEM_REV_REACH, 6)
_MEM_B = torch.tensor([[-0.05, -0.08], [-0.05, 0.08], [-0.15, -0.08], [-0.15, 0.0], [-0.15, 0.08]])


_MEM_BF = torch.tensor([[0.10, -0.08], [0.10, 0.08], [0.18, -0.08], [0.18, 0.0], [0.18, 0.08]])   # front body points


def depth_mem_rev_blocked(env: ManagerBasedRlEnv, steer: torch.Tensor, forward: bool = False) -> torch.Tensor:
  """True where the reverse (or, forward=True, forward) arc at this steering runs into a depth hit remembered in the
  depth map (DEPTH_MAP_S). Forward (2026-11-09, user: "plan ahead, not react at 0.2 m"): a chair leg seen 1-1.5 m
  ahead is still known when it is closer than the camera's 0.2 m minimum range or out of its view in a tight turn.
  Front body points: car front ~0.10-0.18 m ahead of base_footprint (ESTIMATE, unmeasured)."""
  if not getattr(env, "_robotlab_depth_map", False):
    return torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
  e, dev, n = explore(env), env.device, env.num_envs
  s = _MEM_S.to(dev)[None]                                         # [1, S]
  k = (torch.tan(steer) / WHEELBASE)[:, None]
  kk = torch.where(k.abs() < 1e-3, torch.full_like(k, 1e-3), k)
  if forward:
    ax = torch.sin(kk * s) / kk; ay = (1 - torch.cos(kk * s)) / kk; ayaw = kk * s
    b = _MEM_BF.to(dev)
  else:
    ax = -torch.sin(kk * s) / kk; ay = (1 - torch.cos(kk * s)) / kk; ayaw = -kk * s   # reversing pose along the arc
    b = _MEM_B.to(dev)
  cy, sy = torch.cos(ayaw)[..., None], torch.sin(ayaw)[..., None]
  px = (ax[..., None] + cy * b[:, 0] - sy * b[:, 1]).reshape(n, -1)
  py = (ay[..., None] + sy * b[:, 0] + cy * b[:, 1]).reshape(n, -1)
  xy, yaw = car_pose(env)
  c, s_ = torch.cos(yaw)[:, None], torch.sin(yaw)[:, None]
  w = torch.stack([xy[:, 0:1] + c * px - s_ * py, xy[:, 1:2] + s_ * px + c * py], -1)
  idx, inside = _cells(w)
  age = env.episode_length_buf[:, None].to(torch.int32) - torch.gather(e.dseen.view(n, -1), 1, idx).to(torch.int32)
  return ((age >= 0) & (age < round(DEPTH_MAP_S / 0.05)) & inside).any(-1)


def shield(env: ManagerBasedRlEnv, v: torch.Tensor, steer: torch.Tensor, pure: bool = False) -> torch.Tensor:
  """v5: the arc stop (SHIELD_*) or the approaching-object stop, as the car node. pure: no state change (approaching)."""
  half_w = torch.where(v < 0, SHIELD_HALF_W_REV, SHIELD_HALF_W)[:, None]   # v6q: the wide corridor forward only
  hit = arc_blocked(env, v, steer, SHIELD_FRONT, SHIELD_REAR, SHIELD_TIME, half_w)
  # rear_depth_blocked is NOT used (v16, 2026-11-08: chair hits 1.3 -> 1.4%, wall-start hold 0.96 -> 1.70 s: the depth
  # map also holds wall bases behind the car)
  man = getattr(env, "_robotlab_manoeuvre", None)
  if man is not None and man.any():
    hit_t = arc_blocked(env, v, steer, TURN_FRONT, TURN_REAR, TURN_TIME, TURN_HALF_W)
    appr = approaching(env, v, pure)                 # keep its two-scan state advancing every step
    return torch.where(man, hit_t, hit | appr) & (v != 0)
  return (hit | approaching(env, v, pure)) & (v != 0)


def _cone(env: ManagerBasedRlEnv, v: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor] | None:
  """Current and previous scan sectors (m) and the mask of sectors within SHIELD_APPROACH_HALF_ANGLE of the driving
  direction (rear when reversing), from the actor's noisy scan buffer, as the car sees it."""
  st = getattr(env, "_robotlab_lidar_hist_noisy", None)
  if st is None:
    return None
  cur, prev = st["buf"][:, 0] * LIDAR_RANGE, st["buf"][:, 1] * LIDAR_RANGE
  bearing = torch.deg2rad(torch.arange(LIDAR_RAYS // 10, device=v.device) * 10.0 + 5.0)
  bearing = torch.atan2(torch.sin(bearing), torch.cos(bearing))
  ahead = torch.where(v[:, None] < 0, math.pi - bearing.abs()[None], bearing.abs()[None]) < math.radians(SHIELD_APPROACH_HALF_ANGLE)
  return cur, prev, ahead, bearing


def approaching(env: ManagerBasedRlEnv, v: torch.Tensor, pure: bool = False) -> torch.Tensor:
  """The approaching-object stop (v7: in the path, no hold), from the actor's (noisy) scan buffer, as the car sees it.
  pure (2026-11-08, for the teacher's look-ahead): the same verdict without advancing the two-scans-in-a-row state;
  the drive's own call (once per step) must stay the only one that advances it."""
  c = _cone(env, v)
  if c is None:
    return torch.zeros_like(v, dtype=torch.bool)
  cur, prev, ahead, bearing = c
  # Compensate the car's own motion with its MEASURED speed and yaw rate (the car: /odom): while turning, static
  # points move to other sectors between scans (up to ~2 rad/s x 0.1 s = 11 deg at full lock), so the previous scan is
  # rotated by the yaw change first; then a static point's range changes by -vx cos(bearing).
  vel = env.scene["car"].data
  vx, wz = vel.root_link_lin_vel_b[:, 0], vel.root_link_ang_vel_b[:, 2]
  k = torch.round(wz * SCAN_DT / math.radians(10.0)).long()                     # sectors the scene turned by
  idx = (torch.arange(cur.shape[1], device=v.device)[None] + k[:, None]) % cur.shape[1]
  prev = torch.gather(prev, 1, idx)
  rate = (prev - cur) / SCAN_DT - vx[:, None] * torch.cos(bearing)[None]         # object's own approach speed
  ahead = ahead & ((cur * torch.sin(bearing)[None]).abs() < SHIELD_APPROACH_HALF_W)    # v7: in the path only
  fresh = (env.episode_length_buf > 4)[:, None]                                  # history is filled
  raw = ((cur < SHIELD_APPROACH_RANGE) & (prev < LIDAR_RANGE) & (rate > SHIELD_APPROACH_SPEED) & ahead & fresh).any(-1)
  # Two scans in a row: single-scan detections were mostly LiDAR noise and walls shifting between sectors while the car
  # turned (v5w 1750: 77% of stops/holds with no person within 2 m). raw only changes when a new scan arrives.
  prev_raw = getattr(env, "_robotlab_appr_raw", None)
  if prev_raw is None:
    prev_raw = env._robotlab_appr_raw = {"last": torch.zeros_like(raw), "prev": torch.zeros_like(raw)}
  if pure:                                       # what the drive's call this step will answer (it shifts last -> prev)
    return raw & (prev_raw["last"] if getattr(env, "_robotlab_lidar_cast", True) else prev_raw["prev"])
  if getattr(env, "_robotlab_lidar_cast", True):
    prev_raw["prev"], prev_raw["last"] = prev_raw["last"], raw
  appr = raw & prev_raw["prev"]
  return appr


def shield_stop(env: ManagerBasedRlEnv) -> torch.Tensor:
  return env.action_manager.get_term("drive").shielded.float()


# v6: people count only in the car's path (CrowdNav++, Liu et al. ICRA 2023); the corridor ahead is the goal check's.
PATH_LENGTH, PATH_HALF_W = 1.2, 0.5     # m; our design
STALL_SPEED = 0.1                       # m/s (Rudin et al. IROS 2022: r_stall below 0.1 m/s)


def person_in_path(env: ManagerBasedRlEnv) -> torch.Tensor:
  """An active person within PATH_LENGTH (clearance) ahead of the car and PATH_HALF_W of its axis (centre)."""
  e, (xy, yaw) = explore(env), car_pose(env)
  w = walkers(env, e.maps, e.layout)
  rel = w.xy - xy[:, None]
  c, s = torch.cos(yaw)[:, None], torch.sin(yaw)[:, None]
  ax, ay = c * rel[..., 0] + s * rel[..., 1], -s * rel[..., 0] + c * rel[..., 1]
  return (w.active & (ax > 0) & (ax < PATH_LENGTH + CAR_RADIUS + PERSON_RADIUS) & (ay.abs() < PATH_HALF_W)).any(-1)


def stall(env: ManagerBasedRlEnv) -> torch.Tensor:
  """v6 anti-freeze: standing still is penalised unless a person is in the path or the shield holds the car (Rudin et
  al. 2022, gated on a clear path). Walls are no excuse: turn or reverse. Explore v5 froze ~25 s facing a door
  (car #1, 2026-11-04)."""
  v = env.scene["car"].data.root_link_lin_vel_b[:, 0].abs()
  held = env.action_manager.get_term("drive").shielded
  return ((v < STALL_SPEED) & ~person_in_path(env) & ~held).float()


def person_ttc(env: ManagerBasedRlEnv) -> torch.Tensor:
  """v5: 1 - t/TTC_HORIZON for the soonest person the MOVING car would pass closer than TTC_MARGIN within TTC_HORIZON
  (both at constant velocity), else 0. Not while the car stands: a person walks around a stopped car (people.py)."""
  e, (xy, _) = explore(env), car_pose(env)
  w = walkers(env, e.maps, e.layout)
  cv = env.scene["car"].data.root_link_lin_vel_w[:, :2]
  p, u = w.xy - xy[:, None], w.vel - cv[:, None]
  t = (-(p * u).sum(-1) / (u * u).sum(-1).clamp(min=1e-6)).clamp(0.0, TTC_HORIZON)
  dmin = torch.linalg.norm(p + u * t[..., None], dim=-1) - CAR_RADIUS - PERSON_RADIUS
  risk = ((dmin < TTC_MARGIN) & w.active).float() * (1.0 - t / TTC_HORIZON)
  return risk.amax(-1) * (cv.norm(dim=-1) > 0.05)


def people_privileged(env: ManagerBasedRlEnv) -> torch.Tensor:
  """Critic only: each walker's position and velocity in the car frame, and whether it is active."""
  e, (xy, yaw) = explore(env), car_pose(env)
  w = walkers(env, e.maps, e.layout)
  out = []
  for k in range(MAX_PEOPLE):
    act = w.active[:, k, None].float()
    out += [to_frame(w.xy[:, k] - xy, yaw).clamp(-5, 5) * act, to_frame(w.vel[:, k], yaw) * act, act]
  return torch.cat(out, -1)


# Rewards and terminations.


def new_area(env: ManagerBasedRlEnv) -> torch.Tensor:
  return explore(env).new_area


def forward_speed(env: ManagerBasedRlEnv) -> torch.Tensor:
  return env.scene["car"].data.root_link_lin_vel_b[:, 0].clamp(min=0.0) / EXPLORE_V_MAX


def person_clearance(env: ManagerBasedRlEnv) -> torch.Tensor:
  e, (xy, _) = explore(env), car_pose(env)
  w = walkers(env, e.maps, e.layout)
  d = torch.linalg.norm(w.xy - xy[:, None], dim=-1) - CAR_RADIUS - PERSON_RADIUS
  return torch.where(w.active, d, 1e3).amin(-1)


def steer_rate(env: ManagerBasedRlEnv) -> torch.Tensor:
  """v4: squared change of the steering command per step. In v3 action_rate cost 1.6% of new_area, and the steering
  command changed 0.15 (of +-1) per step while driving straight (sim, 64 cars x 90 s).
  v7d: on the EXECUTED steering command (after the steer_cmd_rate limit), not the sampled action: on the raw samples
  the policy's own exploration noise was penalised, and PPO shrank the steering std to 0.0016 (v7b 2900), so it could
  no longer learn to steer around people whatever the contact penalty."""
  cmd = env.action_manager.get_term("drive")._steer_cmd
  prev = getattr(env, "_robotlab_prev_steer_cmd", None)
  if prev is None:
    prev = env._robotlab_prev_steer_cmd = cmd.clone()
  d = torch.where(env.episode_length_buf <= 1, 0.0, cmd - prev)
  prev.copy_(cmd)
  return d ** 2


def speed_rate(env: ManagerBasedRlEnv) -> torch.Tensor:
  """v7d: action_rate on the speed action only (steering: steer_rate, executed command)."""
  am = env.action_manager
  return (am.action[:, 0] - am.prev_action[:, 0]) ** 2


def bumped(env: ManagerBasedRlEnv) -> torch.Tensor:
  return env.scene["bump"].data.found[:, 0] > 0


def hit_person(env: ManagerBasedRlEnv) -> torch.Tensor:
  return torch.stack([env.scene[f"touch{k}"].data.found[:, 0] > 0 for k in range(MAX_PEOPLE)], -1).any(-1)


# v7c: Constraints as Terminations (CaT; Chane-Sane et al., IROS 2024, arXiv 2403.18765): while the car drives toward a
# person closer than CAT_MARGIN (clearance with CAR_RADIUS + PERSON_RADIUS, which counts the feet), the episode ends
# with probability CAT_P_MAX x violation / CAT_MARGIN per step: a dense constraint signal before contact, plain PPO.
# Training only. v7b 2850 (tmp_contact_geom.py): 98% of contacts were feet (below the LiDAR plane), legs still ~0.17 m
# from the chassis; -15 / -100 per contact did not move the contact rate (~9-10%).
CAT_MARGIN = 0.10      # m: design choice
CAT_P_MAX = 0.25       # CaT paper's p_max for hard constraints
# v7f: the margin grows with the speed toward the person (a braking distance): CAT_MARGIN + CAT_TIME x closing speed.
# v7e 3200 (tmp_contact_geom.py): 0.5 s before contact the car still drove 0.65 m/s at 0.8 m, the shield engaged at 76%
# of contacts but too late; contacts flat at ~5.5% for 150 iterations with the fixed 0.10 m margin.
CAT_TIME = 0.5         # s: design choice (the shield's look-ahead is 0.4 s)


def person_close(env: ManagerBasedRlEnv) -> torch.Tensor:
  e, (xy, yaw) = explore(env), car_pose(env)
  w = walkers(env, e.maps, e.layout)
  rel = w.xy - xy[:, None]
  ax = torch.cos(yaw)[:, None] * rel[..., 0] + torch.sin(yaw)[:, None] * rel[..., 1]
  v = env.scene["car"].data.root_link_lin_vel_b[:, 0]
  d = torch.linalg.norm(rel, dim=-1) - CAR_RADIUS - PERSON_RADIUS
  toward = (v[:, None] * ax > 0) & (v.abs() > 0.05)[:, None]
  closing = (v[:, None] * ax / torch.linalg.norm(rel, dim=-1).clamp(min=1e-3)).clamp(min=0)   # m/s toward the person
  m = CAT_MARGIN + CAT_TIME * closing
  p = torch.where(w.active & toward, (m - d) / m, 0.0).clamp(0, 1).amax(-1) * CAT_P_MAX
  return torch.rand_like(p) < p


def as_float(func):
  def f(env, **kw):
    return func(env, **kw).float()
  f.__name__ = func.__name__ + "_penalty"
  return f


def coverage(env: ManagerBasedRlEnv) -> torch.Tensor:
  """Fraction of the floor patch's floor cells seen free (evaluation)."""
  e = explore(env)
  fl = e.floor[e.layout]
  return (e.free & fl).flatten(1).sum(-1).float() / fl.flatten(1).sum(-1).float()


# Config.


def make_mentorpi_explore_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  actor_terms = {
    "car_motion": ObservationTermCfg(func=car_motion),
    "lidar_sectors": ObservationTermCfg(func=lidar_sectors, params={"noisy": True}),
    "depth": ObservationTermCfg(func=depth, params={"noisy": True}),
    "map_fine": ObservationTermCfg(func=built_map_fine, params={"est": True}),
    "map_coarse": ObservationTermCfg(func=built_map_coarse, params={"est": True}),
    "lidar_hist": ObservationTermCfg(func=lidar_history, params={"noisy": True}),   # v5, last so offsets stay
  }
  critic_terms = {
    "car_motion": ObservationTermCfg(func=car_motion),
    "lidar_sectors": ObservationTermCfg(func=lidar_sectors),
    "depth": ObservationTermCfg(func=depth),
    "map_fine": ObservationTermCfg(func=built_map_fine),
    "map_coarse": ObservationTermCfg(func=built_map_coarse),
    "people": ObservationTermCfg(func=people_privileged),
    "lidar_hist": ObservationTermCfg(func=lidar_history),
  }
  lidar = RayCastSensorCfg(
    name="lidar", frame=ObjRef(type="site", name="lidar", entity="car"),
    pattern=LidarPatternCfg(num_rays=LIDAR_RAYS), ray_alignment="yaw", max_distance=LIDAR_RANGE, debug_vis=play,
    include_geom_groups=(0, 1),  # group 2 = visual-only parts (people's torsos)
  )
  depth_cam = RayCastSensorCfg(
    name="depth", frame=ObjRef(type="site", name="depth_cam", entity="car"),
    pattern=C.DepthFanPatternCfg(hfov=C.MEASURED_HFOV, elevations=C.MEASURED_ELEVATIONS), ray_alignment="base",
    max_distance=C.DEPTH_MAX, debug_vis=False, include_geom_groups=(0, 1),
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
          size=(PATCH_SIZE, PATCH_SIZE), num_rows=OFFICE_ROWS, num_cols=OFFICE_COLS, seed=17,
          sub_terrains={"office": OfficeTerrainCfg(hall_width=HALL_WIDTH_RANGE)}, color_scheme="height",
        ),
      ),
      entities={"car": car_cfg(), **{f"person{k}": person_cfg() for k in range(MAX_PEOPLE)}},
      sensors=(lidar, depth_cam, bump, *touch),
      num_envs=1,
    ),
    observations={
      "actor": ObservationGroupCfg(actor_terms, enable_corruption=True),
      "critic": ObservationGroupCfg(critic_terms),
    },
    actions={"drive": AckermannActionCfg(entity_name="car", leader_step=explore_step, v_reverse=EXPLORE_V_REVERSE,
                                         v_max=EXPLORE_V_MAX, steer_max_range=(STEER_2024, STEER_CODE),
                                         steer_cmd_rate=EXPLORE_STEER_CMD_RATE, shield=shield)},
    events={
      "reset_joints": EventTermCfg(
        func=reset_joints_by_offset, mode="reset",
        params={"position_range": (0.0, 0.0), "velocity_range": (0.0, 0.0), "asset_cfg": SceneEntityCfg("car")},
      ),
      "lidar_rate": EventTermCfg(func=cast_lidar_every_other_step, mode="startup"),   # real MS200: 10 Hz
      "reset_explore": EventTermCfg(func=reset_explore, mode="reset"),
      **people_size_events(),
    },
    rewards={
      # ~1 m^2 of new floor per second while exploring a hallway at ~0.2 m/s: weight 10 -> ~0.5 per step.
      "new_area": RewardTermCfg(func=new_area, weight=10.0),
      "forward": RewardTermCfg(func=forward_speed, weight=0.1),
      # v6: personal_space (-10) and speed_near_people (-3) removed: they counted the nearest person in any direction,
      # so people beside / behind slowed the car (floor test 2026-11-04). person_ttc covers people in the path.
      "stall": RewardTermCfg(func=stall, weight=-2.0),                   # v6
      "bump": RewardTermCfg(func=as_float(bumped), weight=-50.0),        # v1/v2: -20
      "hit_person": RewardTermCfg(func=as_float(hit_person), weight=-2000.0),  # v7b: x dt = -100 per contact, ~3x a run's new_area (~36). v7 (-300, no shield hold): contacts flat at ~10% for 150 iterations; v4f -300; v4e -60: flat at ~16%; v1/v2: -30
      "action_rate": RewardTermCfg(func=speed_rate, weight=-0.5),       # v7d: speed only (was action_rate_l2, both)   # v1/v2: -0.05; v2 on car #1 flipped
      "person_ttc": RewardTermCfg(func=person_ttc, weight=-20.0),         # v5 (-5 until iteration 1400: person contacts flat at ~9%)
      "shield_stop": RewardTermCfg(func=shield_stop, weight=-3.0),       # v5: the stop engaged (safe-RL shield + penalty); -1 until 1650
      "steer_rate": RewardTermCfg(func=steer_rate, weight=-20.0),       # v7e (v7d -10 on the executed command: 0.031 -> 0.070 per step in 200 it); v4c -10 (v4/v4b: -3)
      # forward/reverse every step in a tight spot and went nowhere (2026-11-03)
    },
    terminations={
      "time_out": TerminationTermCfg(func=time_out, time_out=True),
      "bump": TerminationTermCfg(func=bumped),
      "hit_person": TerminationTermCfg(func=hit_person),
    },
    viewer=ViewerConfig(
      origin_type=ViewerConfig.OriginType.ASSET_BODY, entity_name="car", body_name="chassis",
      distance=3.5, elevation=55.0, azimuth=180.0,
    ),
    sim=SimulationCfg(nconmax=128, njmax=320, mujoco=MujocoCfg(timestep=0.005, integrator="implicitfast")),
    decimation=10,               # 20 Hz policy
    episode_length_s=90.0,
  )
  R.apply_domain_randomization(cfg)
  if play:
    cfg.episode_length_s = 180.0
    cfg.observations["actor"].enable_corruption = False
  else:
    cfg.terminations["person_close"] = TerminationTermCfg(func=person_close)   # v7c CaT, training only
  return cfg


MentorPiExploreRlCfg = RslRlOnPolicyRunnerCfg(
  actor=RslRlModelCfg(
    hidden_dims=(512, 256, 128), activation="elu", obs_normalization=True,
    distribution_cfg={"class_name": "GaussianDistribution", "init_std": 0.6, "std_type": "scalar"},
  ),
  critic=RslRlModelCfg(hidden_dims=(512, 256, 128), activation="elu", obs_normalization=True),
  algorithm=RslRlPpoAlgorithmCfg(
    value_loss_coef=1.0, use_clipped_value_loss=True, clip_param=0.2, entropy_coef=0.001,   # v7e (0.005: std grew 0.25 -> 0.38 in 200 it once steer_rate stopped penalising the noise)
    num_learning_epochs=5, num_mini_batches=4, learning_rate=1.0e-3, schedule="adaptive",
    gamma=0.995, lam=0.95, desired_kl=0.01, max_grad_norm=1.0,   # longer horizon: exploring pays off seconds later
  ),
  experiment_name="explore",
  logger="tensorboard",
  save_interval=50,
  num_steps_per_env=32,
  max_iterations=1500,
)
