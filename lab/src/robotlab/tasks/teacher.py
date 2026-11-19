"""Privileged teacher for the goto task (2026-11-08): a scripted driver that sees the true map, the true pose and the
people, plans a short path along the route around obstacles, and tracks it with pure pursuit. Purpose: the expert for
imitation learning (the student copies it from the car's own sensors) and a quick test of the plan: if the teacher alone
does not meet the goto goals (tmp_goto_check.py teacher), a student that copies it cannot either.

Output: the drive action of mdp.AckermannAction ([speed, steering], each in [-1, 1]) and the planned path (car frame) as
the student's training label. Reversing is left to the drive's scripted back-up (mdp unstick), as for the policy.

All values marked "design choice" are ours (no source); car values cite mentorpi_a1_constants.
"""

from __future__ import annotations

import math
import os
from typing import TYPE_CHECKING

import torch

from robotlab.robot.mentorpi_a1_constants import WHEELBASE
from robotlab.tasks import mentorpi_explore_env_cfg as X
from robotlab.tasks import mentorpi_navigate_env_cfg as N
from robotlab.tasks.mdp import car_pose, to_frame
from robotlab.tasks.office_terrain import OCC_N, OCC_RES
from robotlab.tasks.people import walkers

if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv

PATH_STEP, PATH_N = 0.1, 20          # planned path: 20 points, 0.1 m apart (2 m), the student's label
OFFSETS = (0.0, 0.1, -0.1, 0.2, -0.2, 0.3, -0.3, 0.4, -0.4)   # m sideways off the route, tried in this order
CLEAR = 0.20                         # m from the path centre to an obstacle: half the car's 159 mm width + 12 cm
PERSON_CLEAR = 0.45                  # m from the path centre to a person's centre (legs + feet ~0.2 m + margin)
PLAN_FROM, PLAN_TO = 0.2, 1.4        # m of path that must be clear for an offset to be chosen
# Pure-pursuit look-ahead = LOOKAHEAD_T x speed, clamped: Nav2 Regulated Pure Pursuit's defaults (lookahead_time 1.5 s,
# min_lookahead_dist 0.3, max 0.9, lookahead_dist 0.6; docs.nav2.org configuring-regulated-pp; Macenski et al. 2023,
# arXiv 2305.20026: a short look-ahead oscillates). v1 used 0.3-0.7 m: steering std on straights 8.3 deg. The minimum
# is 0.6 m here (Nav2's fixed default) because our car starts from rest at every goal (design choice).
LOOKAHEAD_T, LOOKAHEAD = 1.5, (0.6, 0.9)
A_LAT = 0.8                          # m/s^2 lateral acceleration in turns (design choice; no tyre data)
STOP_FREE = 0.15                     # m of free arc at which the speed reaches 0 (see act)
V_CREEP = 0.15                       # m/s: slowest forward speed; slower counts as "stalled" (mdp unstick: < 0.25 m in 2 s)
                                     # and the scripted back-up fires (trace 2026-11-08: 0.06 m/s creep -> back-up loops)
# Local choice among steering candidates (the Dynamic Window Approach idea, Fox, Burgard & Thrun 1997, IEEE RAM 4(1);
# Nav2's DWB controller): when the pure-pursuit move is blocked by the true map or would be stopped by the drive's own
# shield (mentorpi_explore_env_cfg.shield, the car's stop rule), take the allowed candidate closest to it; if no forward
# move is allowed, reverse with the opposite lock. 2026-11-08 idle breakdown (tmp_teacher_idle.py): 48% of the
# teacher's idle time the shield held a move the teacher wanted, 25% the teacher waited at a blocked arc.
CAND = (-1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0)   # x steer_max
REV_V = 0.25                         # m/s reverse when nothing forward is allowed (design choice)
REV_CLEAR = 0.60                     # m of reverse arc clear on the map before a reverse starts: REV_MIN_S x REV_V
                                     # + the shield's rear reach 0.35 + 0.4 x 0.25 (2026-11-08: reverses checked only
                                     # at the start point were stopped by the shield after a few cm: blips)
REV_MIN_S = 0.6                      # s: a reverse lasts at least this unless the rear blocks (floor blips were < 0.5 s)
KTURN_STUCK_S = 1.0                  # s with both legs blocked (either lock) before the turn hands over (design choice)
KTURN_COOLDOWN = 1.0                 # s after a multi-point turn before another may start (trace: restarted at once, circled)
SLOW_GAIN = 1.2                      # 1/s: v <= SLOW_GAIN * (free arc - STOP_FREE) (design choice)
# Multi-point turn (ours: a scripted stand-in for the forward / reverse moves a Hybrid A* + Reeds-Shepp planner finds).
# When the path's look-ahead point is more than KTURN_ON off the heading, the car turns at full lock towards it, forward
# while the true map shows the arc ahead clear for KTURN_ARC, else in reverse with the opposite lock while the arc
# behind is clear, switching at each blocked end; it hands back to pure pursuit within KTURN_OFF.
KTURN_ON, KTURN_OFF = math.radians(100.0), math.radians(40.0)
KTURN_ARC = 0.22                     # m of arc that must be clear to keep going (design choice; shield reach 0.30)
# reverse legs of the multi-point turn need only the shield's rear reach clear (0.35 + 0.4 x 0.25 = 0.45 m); 0.6 m
# (REV_CLEAR) blocked most reverse legs in 1.0-1.2 m halls: 82% done, 12 s per turn (tmp_teacher_uturn.py, 2026-11-08)
KTURN_REV_CLEAR = 0.275              # manoeuvre mode (2026-11-08): the stop's turn-mode rear reach 0.20 + 0.3 x 0.25
# v17 (2026-11-08, env ROBOTLAB_TEACHER_PERSON_SLOW=1 until chosen): at most PERSON_V while a person is within
# PERSON_AHEAD m ahead and +-0.6 m sideways. Teacher v15 mean of 3: at-fault person contacts 1.1% (goal 1%).
PERSON_SLOW = os.environ.get("ROBOTLAB_TEACHER_PERSON_SLOW", "0") == "1"
PERSON_V, PERSON_AHEAD = 0.35, 1.2
EXIT_V = 0.30                        # m/s just after a multi-point turn (design choice; stop reach 0.30 + 0.4 x 0.3)
KTURN_V = 0.25                       # m/s during the turn (design choice; the drive's back-up is 0.8 x 0.3 m/s)
FOOT = ((0.06, 0.0), (-0.06, 0.0))   # car-frame disc centres covering the 213 x 159 mm chassis (research/01 sec. 2)
FOOT_R = 0.10                        # m: a corner (0.1065, 0.0795) is 0.092 m from the nearer centre
_DISC = torch.tensor([[0.0, 0.0], [1.0, 0.0], [-1.0, 0.0], [0.0, 1.0], [0.0, -1.0],
                      [0.7, 0.7], [0.7, -0.7], [-0.7, 0.7], [-0.7, -0.7]])


# Which map the teacher plans on (2026-11-08): "true" = the generator's occupancy (walls, clutter, chair bases, 5 cm);
# "seen" = true obstacles the car has already seen (no never-seen obstacles, no ghost trails of people);
# "built" = what the car has seen, as the student's map inputs: the LiDAR built map (SEEN_RES 0.10 m, occupied cells)
# plus depth hits of the last DEPTH_MAP_S. A teacher that plans on information the student cannot have is
# "unrealizable" and the copy plateaus (student v1 / v2: |student - teacher| 0.17-0.19); see CritiQ, arXiv 2505.09546.
TEACHER_MAP = os.environ.get("ROBOTLAB_TEACHER_MAP", "true")


def _occupied(env, pts: torch.Tensor, r: float) -> torch.Tensor:
  """Any of 9 points on a disc of radius r around each point [N, ..., 2] (env frame) is occupied on TEACHER_MAP."""
  e = X.explore(env)
  d = _DISC.to(pts.device) * r
  q = pts[..., None, :] + d                                                  # [N, ..., 9, 2]
  if TEACHER_MAP == "seen":
    # real obstacles the car has already seen (LiDAR built-map hit or a depth hit, ever this episode), true geometry.
    # "built" alone stops for ghosts: the sim's built map never clears cells where people walked (2026-11-08: idle 18.5%)
    n = e.occ.shape[0]
    ij = (q / OCC_RES).floor().long().clamp(0, OCC_N - 1)
    lay = e.layout.view(-1, *([1] * (q.dim() - 2)))
    true_occ = e.maps.occ[lay.expand(ij.shape[:-1]), ij[..., 0], ij[..., 1]]
    sj = (q / X.SEEN_RES).floor().long().clamp(0, X.SEEN_N - 1)
    flat = (sj[..., 0] * X.SEEN_N + sj[..., 1]).reshape(n, -1)
    seen = torch.gather(e.occ.view(n, -1), 1, flat) | (torch.gather(e.dseen.view(n, -1), 1, flat) > -30000)
    return (true_occ & seen.view(sj.shape[:-1])).any(-1)
  if TEACHER_MAP == "built":
    n = e.occ.shape[0]
    ij = (q / X.SEEN_RES).floor().long().clamp(0, X.SEEN_N - 1)
    flat = (ij[..., 0] * X.SEEN_N + ij[..., 1]).reshape(n, -1)
    occ = torch.gather(e.occ.view(n, -1), 1, flat)
    if getattr(env, "_robotlab_depth_map", False):
      age = env.episode_length_buf[:, None].to(torch.int32) - torch.gather(e.dseen.view(n, -1), 1, flat).to(torch.int32)
      occ = occ | ((age >= 0) & (age < round(X.DEPTH_MAP_S / env.step_dt)))
    return occ.view(ij.shape[:-1]).any(-1)
  ij = (q / OCC_RES).floor().long().clamp(0, OCC_N - 1)
  lay = e.layout.view(-1, *([1] * (q.dim() - 2)))
  return e.maps.occ[lay.expand(ij.shape[:-1]), ij[..., 0], ij[..., 1]].any(-1)


def plan(env) -> tuple[torch.Tensor, torch.Tensor]:
  """Planned path [N, PATH_N, 2] (env frame) from the progress point, shifted sideways around obstacles and people,
  and the free length along it [N] (m to the first blocked point)."""
  nv = N.nav(env)
  dev = env.device
  s = nv.s[:, None] + PATH_STEP * (torch.arange(PATH_N + 1, device=dev) + 1)
  route = N._points_at(nv, s)                                                # [N, P+1, 2]
  tang = route[:, 1:] - route[:, :-1]
  tang = tang / torch.linalg.norm(tang, dim=-1, keepdim=True).clamp(min=1e-6)
  normal = torch.stack([-tang[..., 1], tang[..., 0]], -1)                    # left of the route
  base = route[:, :-1]
  off = torch.tensor(OFFSETS, device=dev)
  cand = base[:, None] + off[None, :, None, None] * normal[:, None]          # [N, O, P, 2]
  blocked = _occupied(env, cand, CLEAR)                                      # [N, O, P]
  e = X.explore(env)
  w = walkers(env, e.maps, e.layout)
  pd = torch.linalg.norm(cand[:, :, :, None] - w.xy[:, None, None], dim=-1)  # [N, O, P, K]
  blocked |= ((pd < PERSON_CLEAR) & w.active[:, None, None]).any(-1)
  dist = PATH_STEP * (torch.arange(PATH_N, device=dev) + 1)
  window = (dist >= PLAN_FROM) & (dist <= PLAN_TO)
  ok = ~(blocked & window).any(-1)                                           # [N, O]
  first = torch.where(ok.any(-1), ok.float().argmax(-1), 0)                  # first clear offset in OFFSETS order
  # hysteresis (2026-11-08): keep the last offset while it stays clear; a fresh pick every step made the path jump
  # between offsets (steering std on straights 5.4 deg)
  last = getattr(env, "_teacher_off", None)
  if last is None:
    last = env._teacher_off = torch.zeros(env.num_envs, dtype=torch.long, device=dev)
  last = torch.where(env.episode_length_buf <= 1, 0, last)
  keep = torch.gather(ok, 1, last[:, None])[:, 0] & (last != 0) & ok[:, 0].logical_not()
  first = torch.where(keep, last, first)                                     # back to the route centre when it clears
  env._teacher_off = first.clone()
  ar = torch.arange(env.num_envs, device=dev)
  path = cand[ar, first]                                                     # [N, P, 2]
  b = blocked[ar, first]
  free = torch.where(b.any(-1), dist[b.float().argmax(-1)], dist[-1] + 1.0)
  return path, free


def _arc_clear(env, xy, yaw, steer_rad, direction: float, length: float) -> torch.Tensor:
  """True where the car can drive length m along the arc at steer_rad (direction +1 forward / -1 reverse) without its
  footprint touching wall or clutter on the true map (people are left to the shield)."""
  k = torch.tan(steer_rad) / WHEELBASE                                      # [N]
  ss = torch.linspace(0.05, length, 6, device=xy.device)[None]              # [1, S]
  th = yaw[:, None] + k[:, None] * direction * ss                           # heading after signed travel direction*s
  kn = torch.where(k.abs() < 1e-3, torch.full_like(k, 1e-3), k)[:, None]   # full lock here: never near 0
  dx = (torch.sin(th) - torch.sin(yaw)[:, None]) / kn                       # exact arc of curvature k
  dy = (torch.cos(yaw)[:, None] - torch.cos(th)) / kn
  c = xy[:, None] + torch.stack([dx, dy], -1)                                # [N, S, 2]
  pts = torch.stack([c + f[0] * torch.stack([torch.cos(th), torch.sin(th)], -1)
                     + f[1] * torch.stack([-torch.sin(th), torch.cos(th)], -1) for f in FOOT], 2)   # [N, S, F, 2]
  return ~_occupied(env, pts, FOOT_R).flatten(1).any(-1)


def _free_along_arc(env, xy, yaw, kappa, length: float = 1.5) -> torch.Tensor:
  """m the car can drive forward along the arc of curvature kappa [N] before its footprint meets wall / clutter on the
  true map or comes within PERSON_CLEAR - CLEAR of a person (v2: measured on the route line, up to 1 m beside the car,
  so the teacher stopped for things not in its way and the stalled back-up drove it off the route; trace 2026-11-08)."""
  ss = torch.arange(1, int(round(length / PATH_STEP)) + 1, device=xy.device)[None] * PATH_STEP   # [1, S]
  th = yaw[:, None] + kappa[:, None] * ss
  kn = torch.where(kappa.abs() < 1e-3, torch.full_like(kappa, 1e-3), kappa)[:, None]
  c = xy[:, None] + torch.stack([(torch.sin(th) - torch.sin(yaw)[:, None]) / kn,
                                 (torch.cos(yaw)[:, None] - torch.cos(th)) / kn], -1)       # [N, S, 2]
  blk = _occupied(env, c + FOOT[0][0] * torch.stack([torch.cos(th), torch.sin(th)], -1), FOOT_R)
  e = X.explore(env)
  w = walkers(env, e.maps, e.layout)
  pd = torch.linalg.norm(c[:, :, None] - w.xy[:, None], dim=-1)
  blk |= ((pd < PERSON_CLEAR) & w.active[:, None]).any(-1)
  return torch.where(blk.any(-1), ss[0][blk.float().argmax(-1)] - PATH_STEP, ss[0, -1] + 1.0)


def _shield_arc(env, v, steer) -> torch.Tensor:
  """The drive shield's corridor test only (mentorpi_explore_env_cfg.arc_blocked with the shield's values). The teacher
  knows where people are and checks them itself; the shield's approaching-object part depends on which LiDAR scan is
  current, which is one step out of phase when asked before the drive (2026-11-08: one-step reverses)."""
  half_w = torch.where(v < 0, X.SHIELD_HALF_W_REV, X.SHIELD_HALF_W)[:, None]
  hit = X.arc_blocked(env, v, steer, X.SHIELD_FRONT, X.SHIELD_REAR, X.SHIELD_TIME, half_w)
  return hit & (v != 0)


def _shield_turn(env, v, steer) -> torch.Tensor:
  """The stop's corridor test with its manoeuvre-mode reaches (explore cfg TURN_*), from the sensors (2026-11-08: the
  turn's reverse legs were checked on the map only; the stop held them 2.75 s for things the map lacks)."""
  hit = X.arc_blocked(env, v, steer, X.TURN_FRONT, X.TURN_REAR, X.TURN_TIME, X.TURN_HALF_W)
  return hit & (v != 0)


def act(env) -> tuple[torch.Tensor, torch.Tensor]:
  """Drive action [N, 2] and the planned path in the car frame [N, PATH_N, 2] (the label)."""
  path, _ = plan(env)
  xy, yaw = car_pose(env)
  drv = env.action_manager.get_term("drive")
  v_now = env.scene["car"].data.root_link_lin_vel_b[:, 0].clamp(min=0.0)
  ld = (LOOKAHEAD_T * v_now).clamp(*LOOKAHEAD)
  rel = torch.stack([to_frame(path[:, i] - xy, yaw) for i in range(PATH_N)], 1)   # car frame
  d = torch.linalg.norm(rel, dim=-1)
  # pure pursuit target: the first path point at least ld away (the last one if none)
  far = d >= ld[:, None]
  k = torch.where(far.any(-1), far.float().argmax(-1), PATH_N - 1)
  ar = torch.arange(env.num_envs, device=env.device)
  tx, ty = rel[ar, k, 0], rel[ar, k, 1]
  L2 = (tx * tx + ty * ty).clamp(min=1e-4)
  kappa = 2.0 * ty / L2                                                      # pure pursuit curvature
  behind = tx < 0.0                                                          # target behind: turn at full lock
  delta = torch.atan(WHEELBASE * kappa)
  delta = torch.where(behind, torch.sign(ty + 1e-6) * drv.steer_max, delta)
  steer = (delta / drv.steer_max).clamp(-1, 1)
  # speed: turn radius, free path ahead
  kap = torch.where(behind, 1.0 / (WHEELBASE / math.tan(math.radians(29.0))), kappa.abs())
  v = torch.minimum(torch.full_like(kap, drv.cfg.v_max), torch.sqrt(A_LAT / kap.clamp(min=1e-3)))
  # free is arc length until the footprint touches; the obstacle is then ~free + 0.16 m from base_footprint, and the
  # shield stops at 0.30 + 0.4 v (= SHIELD_FRONT + SHIELD_TIME v): slowing to 0 at free = STOP_FREE stays just outside it
  def speed_for(k_arc, v_cap):
    fr = _free_along_arc(env, xy, yaw, k_arc)
    vv = torch.minimum(v_cap, (SLOW_GAIN * (fr - STOP_FREE)).clamp(min=0.0))
    return torch.where(vv > 0, vv.clamp(min=V_CREEP), vv)               # drive or stop, no creeping
  v = speed_for(torch.where(behind, torch.sign(ty + 1e-6) * kap, kappa), v)
  ok = (v > 0) & ~_shield_arc(env, v, delta)
  # blocked: the allowed candidate closest to the pure-pursuit steering
  best_d = torch.full_like(delta, 1e9); best_v = torch.zeros_like(v); best_s = delta.clone()
  for c in CAND:
    dc = torch.full_like(delta, c) * drv.steer_max
    kc = torch.tan(dc) / WHEELBASE
    vc = speed_for(kc, torch.minimum(torch.full_like(v, drv.cfg.v_max), torch.sqrt(A_LAT / kc.abs().clamp(min=1e-3))))
    okc = (vc > 0) & ~_shield_arc(env, vc, dc)
    dist = (dc - delta).abs()
    better = okc & (dist < best_d)
    best_d, best_v, best_s = torch.where(better, dist, best_d), torch.where(better, vc, best_v), torch.where(better, dc, best_s)
  use_c = ~ok & (best_d < 1e8)
  v = torch.where(ok, v, torch.where(use_c, best_v, 0.0))
  delta = torch.where(use_c, best_s, delta)
  steer = (delta / drv.steer_max).clamp(-1, 1)
  # nothing forward allowed: back up with the opposite lock if the rear is clear (the shield checks reversing too)
  rev_s = -torch.sign(delta + 1e-6) * drv.steer_max
  rev_ok = ~ok & ~use_c & ~_shield_arc(env, torch.full_like(v, -REV_V), rev_s) & _arc_clear(
    env, xy, yaw, rev_s, -1.0, REV_CLEAR)
  # a person within 1 m ahead blocks: wait for them to move instead of backing up (2026-11-08: reverses cut short)
  e = X.explore(env)
  w = walkers(env, e.maps, e.layout)
  relp = torch.stack([to_frame(w.xy[:, k] - xy, yaw) for k in range(w.xy.shape[1])], 1)        # [N, K, 2]
  person_ahead = ((relp[..., 0] > 0) & (relp[..., 0] < 1.0) & (relp[..., 1].abs() < 0.5) & w.active).any(-1)
  rev_ok &= ~person_ahead
  speed = torch.where(rev_ok, -REV_V / drv.cfg.v_reverse, (v / drv.cfg.v_max).clamp(0, 1))
  steer = torch.where(rev_ok, rev_s / drv.steer_max, steer)
  if PERSON_SLOW:
    near = ((relp[..., 0] > 0) & (relp[..., 0] < PERSON_AHEAD) & (relp[..., 1].abs() < 0.6) & w.active).any(-1)
    speed = torch.where(near & (speed > 0), speed.clamp(max=PERSON_V / drv.cfg.v_max), speed)
  # multi-point turn
  st = getattr(env, "_teacher_kturn", None)
  if st is None:
    st = env._teacher_kturn = {"on": torch.zeros_like(behind), "dir": torch.ones_like(tx), "cool": torch.zeros_like(tx)}
  fresh = env.episode_length_buf <= 1
  st["on"] &= ~fresh
  st["dir"] = torch.where(fresh, 1.0, st["dir"])
  st["cool"] = torch.where(fresh, 0.0, (st["cool"] - env.step_dt).clamp(min=0.0))
  ang = torch.atan2(ty, tx).abs()
  # v13 (2026-11-08): also when no forward move is allowed (DWA found none): the multi-point turn runs in manoeuvre mode
  # (the stop's turn reaches); the plain reverse kept the wide reaches and wall starts were held 1.24 s (teacher v12)
  # ... but not when a person blocks: wait for them (v13 started turns next to people, at-fault contacts 1.0 -> 2.1%)
  start = ~st["on"] & (st["cool"] <= 0) & ((ang > KTURN_ON) | (~ok & ~use_c & ~person_ahead))   # v19: both wait the cooldown
  done = st["on"] & (ang < KTURN_OFF) & (ok | use_c)        # v13: and a forward move is allowed again
  st["cool"] = torch.where(done, KTURN_COOLDOWN, st["cool"])
  st["on"] = (st["on"] | start) & ~done
  st["dir"] = torch.where(start, 1.0, st["dir"])
  side = torch.sign(ty + 1e-6)

  def legs(sd):
    f = _arc_clear(env, xy, yaw, sd * drv.steer_max, 1.0, KTURN_ARC) & ~_shield_turn(
      env, torch.full_like(tx, KTURN_V), sd * drv.steer_max)
    r = _arc_clear(env, xy, yaw, -sd * drv.steer_max, -1.0, KTURN_REV_CLEAR) & ~_shield_turn(
      env, torch.full_like(tx, -KTURN_V), -sd * drv.steer_max)
    return f, r
  fwd_ok, rev_ok = legs(side)
  # v19 (2026-11-08): both legs blocked on the route's side -> try the other lock (swing the nose the other way); still
  # blocked for KTURN_STUCK_S -> end the turn and let the drive's back-up rule act (v18 disabled the back-up inside the
  # turn: chair hits 1.3 -> 0.2%, but turns with both legs blocked waited: idle 3.2 -> 6.1%)
  f2, r2 = legs(-side)
  flip = ~fwd_ok & ~rev_ok & (f2 | r2)
  side = torch.where(flip, -side, side)
  fwd_ok, rev_ok = torch.where(flip, f2, fwd_ok), torch.where(flip, r2, rev_ok)
  if "stuck" not in st:
    st["stuck"] = torch.zeros_like(tx)
  both = st["on"] & ~fwd_ok & ~rev_ok
  st["stuck"] = torch.where(both, st["stuck"] + env.step_dt, 0.0)
  give_up = st["stuck"] > KTURN_STUCK_S
  st["on"] = st["on"] & ~give_up
  st["cool"] = torch.where(give_up, KTURN_COOLDOWN, st["cool"])
  st["stuck"] = torch.where(give_up, 0.0, st["stuck"])
  # switch direction at a blocked end; if both are blocked keep the current one (the shield / back-up take over)
  st["dir"] = torch.where(st["on"] & (st["dir"] > 0) & ~fwd_ok & rev_ok, -1.0,
                          torch.where(st["on"] & (st["dir"] < 0) & ~rev_ok & fwd_ok, 1.0, st["dir"]))
  kt_speed = torch.where(st["dir"] > 0, KTURN_V / drv.cfg.v_max, -KTURN_V / drv.cfg.v_reverse).clamp(-1, 1)
  # only move in a direction the true map shows clear (2026-11-08: 12 of 17 teacher chair hits were in the multi-point
  # turn, which kept driving when both ends were blocked); otherwise stop and let the shield / back-up rule act
  dir_ok = torch.where(st["dir"] > 0, fwd_ok, rev_ok)
  kt_speed = torch.where(dir_ok, kt_speed, 0.0)
  kt_steer = st["dir"] * side                                                # reverse: opposite lock
  # gentle exit (v15, 2026-11-08): for KTURN_COOLDOWN s after a turn, at most EXIT_V. Blocked-start trace: right after the
  # turn the teacher went to 0.6-0.7 m/s beside the wall and the stop held it again and again (held 1.0-1.3 s)
  speed = torch.where(~st["on"] & (st["cool"] > 0), speed.clamp(max=EXIT_V / drv.cfg.v_max), speed)
  speed = torch.where(st["on"], kt_speed, speed)
  steer = torch.where(st["on"], kt_steer, steer)
  env._robotlab_manoeuvre = st["on"].clone()     # the drive's stop uses its turn-mode reaches (explore cfg TURN_*)
  # minimum reverse: keep a started reverse going for REV_MIN_S with the same steering while the rear is clear
  if "rev_t" not in st:
    st["rev_t"] = torch.zeros_like(tx); st["rev_a"] = torch.zeros(env.num_envs, 2, device=env.device)
  hold = (st["rev_t"] > 0) & (st["rev_t"] < REV_MIN_S) & ~fresh & ~_shield_arc(
    env, st["rev_a"][:, 0] * drv.cfg.v_reverse, st["rev_a"][:, 1] * drv.steer_max)
  speed = torch.where(hold, st["rev_a"][:, 0], speed)
  steer = torch.where(hold, st["rev_a"][:, 1], steer)
  rev = speed < 0
  st["rev_t"] = torch.where(rev, st["rev_t"] + env.step_dt, 0.0)
  st["rev_a"] = torch.where((rev & ~hold)[:, None], torch.stack([speed, steer], -1), st["rev_a"])
  return torch.stack([speed, steer], -1), rel
