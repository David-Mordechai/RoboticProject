"""Go-to task (2026-11-04): explore v7g's car, sensors, shield, people and safety terms, driving a planned route to a
goal instead of exploring. Replaces car/nav/drive_to.py (hand-coded follow-the-gap), which stopped again and again on
the floor (hall5-7, 2026-11-04: remembered depth points as obstacles, narrow heading window, circling turnarounds).

The route comes from the hallway-graph planner of the navigate task (stand-in for Nav2's planner on the SLAM map, which
runs on the car); the policy sees it as navigate's route preview (8 points 0.25-2 m ahead + remaining length, car
frame) appended AFTER explore's inputs, so explore checkpoints warm-start it (lab/scripts/warm_start.py). When a goal
is reached the next one is drawn: an episode is a string of errands."""

from __future__ import annotations

import dataclasses
import math
import os

import torch

from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.observation_manager import ObservationTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.managers.termination_manager import TerminationTermCfg
from mjlab.rl import RslRlPpoAlgorithmCfg

from robotlab.tasks import mentorpi_explore_env_cfg as X
from robotlab.tasks import mentorpi_navigate_env_cfg as N
from robotlab.tasks import localization as Lz
from robotlab.tasks import office_terrain as OT
from robotlab.tasks.mdp import car_pose, place_car, to_frame

GOAL_RADIUS = N.GOAL_RADIUS
LOST_DISTANCE = 2.5      # m from the route: episode ends. goto v3 (navigate's 1.5): turn-back routes were lost after
                         # 2.2 s, before a turnaround could even start (tmp_goto_lost.py, goto_v2 3800)
DOOR_PROB = 0.35         # share of hallway segments with a doorway (design choice)
GOAL_DIST = 1.0         # m from the route's end once its length is used up: reached. goto v3 (navigate's GOAL_RADIUS
                         # + 0.15 = 0.5 m): cars passing the end 0.6 m off never arrived and got lost after it


MIDTURN_PROB = 0.3   # goto v5: share of episodes whose first route turns back in the middle of the hallway, 0.8-1.6 m
MIDTURN_AHEAD = (0.8, 1.6)   # ahead of the start, >= 1.4 m from both intersections (start is 0.6 m from node a, PITCH
                   # 3.6). The floor trip (5 m into a 1.45 m hall and back, goto3-8 2026-11-05): goto v4 never turned
                   # back mid-hall, only at junctions (sim routes turned back only there)
MIDTURN_AHEAD_NODE = (1.4, 2.2)   # m from the node just reached (goto v5d): >= 1.4 m from both intersections (PITCH 3.6)
MIDTURN_SIDE = 0.4   # m, goto v5e: the way back runs this far to one side (random left / right) of the way out. v5c 5200
                     # step trace: on a route straight back the steering flipped left / right every few steps (a
                     # Gaussian policy averaging two equally good turns) and the car drove straight on
BLOCKED_PROB = 0.25  # goto v6: share of episodes that start mid-turn, nose to a hallway wall (floor goto9 / goto10,
                     # 2026-11-05: v5e 6000 turned, stopped 0.17-0.38 m from the wall, kept asking forward 15+ s, never
                     # reversed; sim v5e reversed in ~25% of turns)
# goto v6e: corner starts harder (closer, more turned in): v6d passed them in 0.1-0.2 s in sim but froze at the real
# office door (goto14: frame 0.45 m at 45-55 deg right, heading 40 deg into the turn; tmp_stuck_probe.py: stop_held unused)
CORNER_PROB = 0.15   # goto v6d: share of episodes that start cutting a corner into a side hallway, the corner at the
                     # car's front corner (floor goto11, 2026-11-05: turning right into the office door too early, the
                     # door frame 0.38 m at 55 deg right, held 15+ s asking full speed + full right)
CORNER_RANGE = (0.25, 0.40)  # m, base_footprint to the corner (stop reach at full speed 0.30 + 0.4 x 0.7 = 0.58 m)
CORNER_BEARING = (30.0, 65.0)   # deg off the heading, on the turn side
CORNER_EARLY = (30.0, 60.0)     # deg the heading is already turned towards the side hallway
BLOCKED_GAP = (0.22, 0.42)   # m, base_footprint to the wall along the heading: at or just past the stop distance
                             # (SHIELD_FRONT 0.30 m at standstill)
BLOCKED_BOXED_ANGLE = (45.0, 80.0)   # deg, goto v6d boxed-in variant
BLOCKED_ANGLE = 35.0         # deg, heading off the wall normal (0 = square to the wall)
UNSTICK = (0.3, 0.8, 0.8, 2.0, 0.25)   # v6l: back up after 0.3 s held (was 0.5; 3-check mean of v6k 8200: blocked
                                       # starts held 1.25 s > 1.0 goal)   # held s, back-up s, x v_reverse, window s, min metres in the window (v6i: stalled
                             # trigger); as car/policy/explore_policy.py UNSTICK_*. goto v6f (floor
                             # goto16 2026-11-05: full trip with it; v6e 6950 sim: narrow-hall turn-backs 38%, few reversed)
SHIELD_STOP_W = -10.0
PERSON_TTC_W = -50.0         # goto v6o (v6l: -30, explore -20): at-fault person contacts stayed 1.0-1.3% (v6n 8400-8500)
STOP_HELD_FULL = 2.0         # s: the stop_held input goes 0 -> 1 over this long (goto v6c). v6b 6300 step trace: held by
                             # the stop, the observation stopped changing, so the policy asked the same thing forever        # goto v6 (explore: -3): asking to drive into the stop while it holds the car
MIDTURN_LOST = 1.3   # m from the route on a mid-hallway turn-back route (until its goal): lost. goto v6j (was 1.0): v6i
                     # 7450 completed 83.5% at 1.0 m but 89.4% at 1.3 m (tmp_check_lost13.py): late but successful turns;
                     # the floor hall has metres of room past the turn point, and 1.3 m is still short of the nearest
                     # junction centre (>= 1.4 m). goto v5b: v5 4950 drove
                     # 1.4-1.7 m past the turn point to the next junction and turned there (tmp_midturn_trace.py); the
                     # floor hall has no junction close by, so the turn must happen in the hallway
CHAIR_PROB = 0.5   # goto v7: share of free hallway segments with an office chair base (office_terrain.CHAIR_*)


def enable_depth_map(env, env_ids) -> None:
  """Startup event (goto v7): depth hits go into the built map (mentorpi_explore_env_cfg.update_depth_map)."""
  env._robotlab_depth_map = True


HALL_WIDTH = (1.0, 1.8)   # goto v6e (v5: 1.2-1.8): below ~1.1 m a U-turn takes several back-and-forth moves (the sim's single
                          # sweep needs 0.68-0.81 m + margins, turnaround_env_cfg): the user wants the car to keep
                          # correcting back and forth until it has turned (2026-11-05)   # m, goto v5: around the measured 1.45 m hall (LiDAR 0.7 + 0.75 m at the turn, goto8)
UTURN_PROB = 0.5   # share of next goals back the way the car came: the floor trip is "5 m into the hall, turn back,
                   # return to the charger" (user, 2026-11-04); navigate's goals never started with a U-turn


TURN_ZONE = 0.5   # m (goto v7, ours): no driving-away back-up this close to a turn-back route's turn points. Floor goto17
# (2026-11-08): back-ups #4-6 fired 0.18-0.40 m from the hall-end turn while the car swung wide on purpose; v6k's case
# (driving on past the turn point to the 1.3 m lost limit) is still caught past it. v7c: 0.5 (was 0.8; v6p 8750 in the
# v7 sim lost 132 runs on turn-back routes; the goto17 false triggers were 0.18-0.40 m from the corners). Car: explore_policy.TURN_ZONE.


def _set_turn(env, ids: torch.Tensor, p1: torch.Tensor, p2: torch.Tensor) -> None:
  """The two corners of a turn-back route's U (env frame) for moving_away; 1e3 = none."""
  if not hasattr(env, "_robotlab_turn_xy"):
    env._robotlab_turn_xy = torch.full((env.num_envs, 2, 2), 1e3, device=env.device)
  env._robotlab_turn_xy[ids] = torch.stack([p1, p2], 1)


def _midturn_pts(start_xy, base, axis, f):
  """start -> T (f along the hallway from base) -> MIDTURN_SIDE to a random side -> back level with base -> base."""
  side = torch.where(torch.rand(len(f), device=f.device) < 0.5, -1.0, 1.0)[:, None] * MIDTURN_SIDE
  lat = torch.stack([-axis[:, 1], axis[:, 0]], -1) * side
  t = base + axis * f[:, None]
  return [start_xy, t, t + lat, base + lat, base]


def _pick_goal(nv, ids: torch.Tensor, start: torch.Tensor, heading_dir: torch.Tensor) -> torch.Tensor:
  """navigate's goal pick (no U-turn first), or with UTURN_PROB any goal (a route that may start by turning back)."""
  g = N._pick_goal(nv, ids, start, heading_dir)
  first = nv.next_dir[nv.layout[ids], start]
  any_g = torch.multinomial((first >= 0).float(), 1)[:, 0]
  return torch.where(torch.rand(len(ids), device=g.device) < UTURN_PROB, any_g, g)


def goto_step(env) -> None:
  """Per policy step, before physics: explore's step (localization, people, built map), then route progress and the
  next goal once one is reached (navigate's advance_world without its walkers / marker)."""
  X.explore_step(env)
  nv, (xy, _) = N.nav(env), car_pose(env)
  ids = torch.arange(env.num_envs, device=env.device)
  nv.s_prev[:] = nv.s
  N._project(nv, ids, xy)
  goal_xy = nv.maps.node_xy[nv.layout, nv.goal]
  nv.reached = (N._remaining(nv) < GOAL_RADIUS) & (torch.linalg.norm(goal_xy - xy, dim=-1) < GOAL_DIST)
  if nv.reached.any():
    r = nv.reached.nonzero()[:, 0]
    if hasattr(env, "_robotlab_midturn_open"):
      env._robotlab_midturn_open[r] = False      # the mid-hallway route is done; later routes use LOST_DISTANCE
    nv.goals_reached[r] += 1
    start = nv.goal[r]
    g = _pick_goal(nv, r, start, nv.arrive_dir[r])
    pts = [xy[r]]
    nv.arrive_dir[r] = N._walk_to_goal(nv, r, pts, start, g, nv.arrive_dir[r])
    nv.goal[r] = g
    N._set_path(nv, r, pts, xy[r])
    # goto v5d: the next route is a mid-hallway turn-back too with MIDTURN_PROB (v5c: only at reset, ~50 per iteration
    # of 8192 cars; 1.8-3.1% done from 5100 to 5200). Out along the route's first hallway to T, back to this node.
    mid = torch.rand(len(r), device=xy.device) < MIDTURN_PROB
    if mid.any():
      m, lay = r[mid], nv.layout[r[mid]]
      d0 = nv.next_dir[lay, start[mid], g[mid]].clamp(min=0)
      na = nv.maps.node_xy[lay, start[mid]]
      nb = nv.maps.node_xy[lay, nv.maps.nbr[lay, start[mid], d0]]
      axis = (nb - na) / torch.linalg.norm(nb - na, dim=-1, keepdim=True).clamp(min=1e-6)
      f = torch.empty(len(m), device=xy.device).uniform_(*MIDTURN_AHEAD_NODE)
      nv.goal[m] = start[mid]
      nv.arrive_dir[m] = d0 ^ 1
      pts = _midturn_pts(xy[m], na, axis, f)
      N._set_path(nv, m, pts, xy[m])
      _set_turn(env, m, pts[1], pts[2])
      env._robotlab_midturn_open[m] = True
    nv.s_prev[r] = nv.s[r]                  # a new route is not progress
  env._robotlab_dprev = N.route_distance(env)   # before physics, for route_approach (goto v5c)
  if IMITATE_W > 0:   # RL fine-tuning of the imitation student: the teacher's action for the state the policy just acted in
    # every IMITATE_EVERY-th step (the teacher's look-ahead was ~80% of a fine-tune iteration: 31 s at 8192 cars); the
    # label is then up to 50 ms old, the teacher's action changes slowly (rate-limited steering)
    if env.common_step_counter % IMITATE_EVERY == 0 or getattr(env, "_robotlab_teacher_label", None) is None:
      from robotlab.tasks import teacher as TCH
      env._robotlab_teacher_label = TCH.act(env)[0]
  held = getattr(env, "_robotlab_held", None)
  if held is None:
    held = env._robotlab_held = torch.zeros(env.num_envs, device=env.device)
  sh = env.action_manager.get_term("drive").shielded          # the stop held the car in the last step (goto v6c)
  held.copy_(torch.where(sh, held + env.step_dt, torch.zeros_like(held)))


def _reset_twin(env, env_ids: torch.Tensor, xy: torch.Tensor) -> None:
  """Office twin (robotlab/tasks/office_twin.py): the floor route from the charger; the U corners at the hall end are
  the turn-back corners (= car node turn_backs); no mid-hallway / blocked / corner start types."""
  from robotlab.tasks import office_twin as _TW
  nv, dev, n = N.nav(env), xy.device, env.num_envs
  pts = [xy] + [p.expand(len(env_ids), 2) for p in _TW.route_points(dev)[1:]]
  nv.goal[env_ids] = 0
  nv.arrive_dir[env_ids] = 0
  N._set_path(nv, env_ids, pts, xy)
  for nm, dt in (("_robotlab_midturn", torch.bool), ("_robotlab_midturn_open", torch.bool), ("_robotlab_blocked", torch.bool),
                 ("_robotlab_corner", torch.bool), ("_robotlab_turn_s", torch.float32)):
    if not hasattr(env, nm):
      setattr(env, nm, torch.zeros(n, dtype=dt, device=dev))
  env._robotlab_midturn[env_ids] = False
  env._robotlab_midturn_open[env_ids] = True     # the U corners count for the near-turn / turn-zone rules
  env._robotlab_blocked[env_ids] = False
  env._robotlab_corner[env_ids] = False
  env._robotlab_turn_s[env_ids] = 1e3
  tb = torch.tensor(_TW.data()["turn_backs"], dtype=torch.float32, device=dev)
  _set_turn(env, env_ids, tb[0].expand(len(env_ids), 2), tb[1].expand(len(env_ids), 2))
  if hasattr(env, "_robotlab_held"):
    env._robotlab_held[env_ids] = 0.0


def reset_goto(env, env_ids: torch.Tensor) -> None:
  """After reset_explore placed the car (0.6 m into a hallway from node a, heading along direction d): a route from
  there to a random goal node."""
  nv = N.nav(env)
  a, d, xy = env._robotlab_start            # set by reset_explore just before (same env_ids)
  from robotlab.tasks import office_twin as _TW
  if _TW.TWIN_PATH:
    _reset_twin(env, env_ids, xy)
    return
  k = nv.layout[env_ids]
  b = nv.maps.nbr[k, a, d]
  r = nv.maps.route[k, a, d]
  pts = [xy, r[:, 0], r[:, 1], nv.maps.node_xy[k, b]]
  g = _pick_goal(nv, env_ids, b, d)
  nv.arrive_dir[env_ids] = N._walk_to_goal(nv, env_ids, pts, b, g, d)
  nv.goal[env_ids] = g
  N._set_path(nv, env_ids, pts, xy)
  # goto v5: mid-hallway turn-back routes (start -> T ahead in the hallway -> back past the start to node a)
  mid = torch.rand(len(env_ids), device=xy.device) < MIDTURN_PROB
  if not hasattr(env, "_robotlab_midturn"):
    env._robotlab_midturn = torch.zeros(env.num_envs, dtype=torch.bool, device=xy.device)
    env._robotlab_turn_s = torch.zeros(env.num_envs, device=xy.device)
    env._robotlab_midturn_open = torch.zeros(env.num_envs, dtype=torch.bool, device=xy.device)
  env._robotlab_midturn[env_ids] = mid
  env._robotlab_midturn_open[env_ids] = mid
  _set_turn(env, env_ids, torch.full_like(xy, 1e3), torch.full_like(xy, 1e3))
  if mid.any():
    m = env_ids[mid]
    na, nb = nv.maps.node_xy[k[mid], a[mid]], nv.maps.node_xy[k[mid], b[mid]]
    axis = (nb - na) / torch.linalg.norm(nb - na, dim=-1, keepdim=True)
    f = torch.empty(len(m), device=xy.device).uniform_(*MIDTURN_AHEAD)
    nv.goal[m] = a[mid]
    nv.arrive_dir[m] = d[mid] ^ 1
    pts = _midturn_pts(xy[mid], xy[mid], axis, f)
    N._set_path(nv, m, pts[:-1] + [na], xy[mid])
    _set_turn(env, m, pts[1], pts[2])
    env._robotlab_turn_s[m] = f
  # goto v6: blocked starts. A mid-hallway route whose car already stands at the turn point, nose to the wall on the
  # side it turns to: it has to back up (a 3-point turn) before it can drive the way back.
  blk = (torch.rand(len(env_ids), device=xy.device) < BLOCKED_PROB) & ~mid
  if not hasattr(env, "_robotlab_blocked"):
    env._robotlab_blocked = torch.zeros(env.num_envs, dtype=torch.bool, device=xy.device)
  env._robotlab_blocked[env_ids] = blk
  if blk.any():
    m, n_ = env_ids[blk], int(blk.sum())
    dev = xy.device
    na, nb = nv.maps.node_xy[k[blk], a[blk]], nv.maps.node_xy[k[blk], b[blk]]
    axis = (nb - na) / torch.linalg.norm(nb - na, dim=-1, keepdim=True)
    side = torch.where(torch.rand(n_, device=dev) < 0.5, -1.0, 1.0)
    lat = torch.stack([-axis[:, 1], axis[:, 0]], -1) * side[:, None]          # towards the wall it faces
    t = xy[blk] + axis * torch.empty(n_, device=dev).uniform_(*MIDTURN_AHEAD)[:, None]
    phi = torch.deg2rad(torch.empty(n_, device=dev).uniform_(-BLOCKED_ANGLE, BLOCKED_ANGLE))
    gap = torch.empty(n_, device=dev).uniform_(*BLOCKED_GAP)
    # goto v6d: half of them boxed in mid-turn (floor goto12: wall 0.45 m at 35 deg, 0.51 m ahead, 0.61 m behind): the
    # heading turned 45-80 deg off the wall normal, so the car stands across a 1.2-1.8 m hallway with little room either way
    boxed = torch.rand(n_, device=dev) < 0.5
    phi = torch.where(boxed, torch.deg2rad(torch.empty(n_, device=dev).uniform_(*BLOCKED_BOXED_ANGLE)) *
                      torch.where(torch.rand(n_, device=dev) < 0.5, -1.0, 1.0), phi)
    off = nv.maps.hall_width[k[blk]] / 2 - gap * torch.cos(phi)               # base_footprint off the centre line
    car = t + lat * off[:, None] - axis * 0.1 * torch.rand(n_, device=dev)[:, None]
    yaw = torch.atan2(lat[:, 1], lat[:, 0]) + phi * side
    # skip spots where the car's outline (+3 cm) overlaps a wall, door partition or clutter: v6 6200 placed some cars
    # inside clutter, which ended the episode as a bump at the first step (tmp_blocked_place.py)
    fp = torch.tensor([[x, y] for x in (-0.14, 0.0, 0.14) for y in (-0.11, 0.0, 0.11)], device=dev)   # car frame
    c_, s_ = torch.cos(yaw)[:, None], torch.sin(yaw)[:, None]
    wx, wy = car[:, :1] + c_ * fp[:, 0] - s_ * fp[:, 1], car[:, 1:] + s_ * fp[:, 0] + c_ * fp[:, 1]
    ix = (wx / OT.OCC_RES).floor().long().clamp(0, OT.OCC_N - 1)
    iy = (wy / OT.OCC_RES).floor().long().clamp(0, OT.OCC_N - 1)
    free = ~nv.maps.occ[k[blk][:, None].expand_as(ix), ix, iy].any(-1)
    sel = blk.nonzero()[:, 0][~free]
    blk[sel] = False
    env._robotlab_blocked[env_ids] = blk
    m, car, yaw, t, lat, a_, d_, xy_, na = (m[free], car[free], yaw[free], t[free], lat[free], a[blk], d[blk], xy[blk],
                                            na[free])
  if blk.any():
    place_car(env, m, car, yaw)
    Lz.reset_localization(env, m, torch.cat([car, yaw[:, None]], -1)[:, None])
    back = lat * MIDTURN_SIDE
    nv.goal[m] = a_
    nv.arrive_dir[m] = d_ ^ 1
    N._set_path(nv, m, [car, t + back, xy_ + back, na], car)
    _set_turn(env, m, car, t + back)
    env._robotlab_midturn_open[m] = True
  # goto v6d: corner starts. Just before intersection b, turned early towards a side hallway, the corner of the wall
  # block at the front corner: it has to straighten or back up before it can turn in.
  crn = (torch.rand(len(env_ids), device=xy.device) < CORNER_PROB) & ~mid & ~blk
  if not hasattr(env, "_robotlab_corner"):
    env._robotlab_corner = torch.zeros(env.num_envs, dtype=torch.bool, device=xy.device)
  dev = xy.device
  turn = torch.where(torch.rand(len(env_ids), device=dev) < 0.5, 2, 0) + torch.randint(0, 2, (len(env_ids),), device=dev)
  turn = torch.where(d >= 2, turn % 2, turn % 2 + 2)                      # a direction across d (0/1 = +-x, 2/3 = +-y)
  nb2 = nv.maps.nbr[k, b, turn]
  crn &= nb2 >= 0
  if crn.any():
    m, n_ = env_ids[crn], int(crn.sum())
    dv = torch.tensor(X._DIR_VEC, device=dev)
    u, v = dv[d[crn]], dv[turn[crn]]
    nb_xy = nv.maps.node_xy[k[crn], b[crn]]
    hw = nv.maps.hall_width[k[crn]][:, None] / 2
    corner = nb_xy - u * hw + v * hw
    ang_u = torch.atan2(u[:, 1], u[:, 0])
    s_ = torch.sign(u[:, 0] * v[:, 1] - u[:, 1] * v[:, 0])                # +1: turn left
    U = lambda r: torch.empty(n_, device=dev).uniform_(*r)
    h = ang_u + s_ * torch.deg2rad(U(CORNER_EARLY))
    bear = h + s_ * torch.deg2rad(U(CORNER_BEARING))
    car = corner - U(CORNER_RANGE)[:, None] * torch.stack([torch.cos(bear), torch.sin(bear)], -1)
    fp = torch.tensor([[x, y] for x in (-0.14, 0.0, 0.14) for y in (-0.11, 0.0, 0.11)], device=dev)
    c_, sn = torch.cos(h)[:, None], torch.sin(h)[:, None]
    wx, wy = car[:, :1] + c_ * fp[:, 0] - sn * fp[:, 1], car[:, 1:] + sn * fp[:, 0] + c_ * fp[:, 1]
    ix = (wx / OT.OCC_RES).floor().long().clamp(0, OT.OCC_N - 1)
    iy = (wy / OT.OCC_RES).floor().long().clamp(0, OT.OCC_N - 1)
    free = ~nv.maps.occ[k[crn][:, None].expand_as(ix), ix, iy].any(-1)       # outline clear of walls / clutter
    crn[crn.nonzero()[:, 0][~free]] = False
    if crn.any():
      m, car, h, nb_xy, v = m[free], car[free], h[free], nb_xy[free], v[free]
      place_car(env, m, car, h)
      Lz.reset_localization(env, m, torch.cat([car, h[:, None]], -1)[:, None])
      nv.goal[m] = nb2[crn]
      nv.arrive_dir[m] = turn[crn]
      N._set_path(nv, m, [car, nb_xy, nb_xy + v * 1.0, nv.maps.node_xy[k[crn], nb2[crn]]], car)
  env._robotlab_corner[env_ids] = crn
  if hasattr(env, "_robotlab_held"):
    env._robotlab_held[env_ids] = 0.0
  nv.goals_reached[env_ids] = 0
  nv.reached[env_ids] = False
  nv.loc_bias[env_ids] = 0.0              # explore's localization model (Lz) already offsets the map; none here
  nv.loc_jump[env_ids] = 0.0


def progress(env) -> torch.Tensor:
  nv = N.nav(env)
  return ((nv.s - nv.s_prev) / (X.EXPLORE_V_MAX * env.step_dt)).clamp(-1, 1)


# goto v5c: dense rewards on mid-hallway turn-back routes (until their goal). v5b 5050: 90% of the failed turn-backs never
# reversed, they drove on to the 1 m limit (tmp_midturn_trace.py); "lost" at the end was the only signal.
def route_approach(env) -> torch.Tensor:
  """Getting closer to the route's progress point this step (+), moving away (-), in units of full speed."""
  d = N.route_distance(env)
  dp = getattr(env, "_robotlab_dprev", d)
  return ((dp - d) / (X.EXPLORE_V_MAX * env.step_dt)).clamp(-1, 1) * _mid_open(env)


def route_heading(env) -> torch.Tensor:
  """cos(car heading - route direction 0.25 -> 1 m past the progress point)."""
  nv, (_, yaw) = N.nav(env), car_pose(env)
  p = N._points_at(nv, nv.s[:, None] + torch.tensor([0.25, 1.0], device=env.device))
  t = p[:, 1] - p[:, 0]
  return torch.cos(yaw - torch.atan2(t[:, 1], t[:, 0])) * _mid_open(env)


def _mid_open(env) -> torch.Tensor:
  m = getattr(env, "_robotlab_midturn_open", None)
  return m.float() if m is not None else torch.zeros(env.num_envs, device=env.device)


BACK_CLEAR = 0.6   # m clear behind (LiDAR rear quarter) to start a back-up (goto v6n; car: explore_policy.py BACK_CLEAR)
AWAY_DIST, AWAY_RATE = 0.7, 0.1   # m off the route, m/s moving away from it (goto v6k trigger, ours)


def moving_away(env) -> torch.Tensor:
  """goto v6k back-up trigger: on a mid-hallway turn-back route, the car is AWAY_DIST off the route and still moving
  away from it (v6j 7650-7800: 60% of the failed turn-backs drove on past the turn point to the 1.3 m limit)."""
  d = N.route_distance(env)
  dp = getattr(env, "_robotlab_away_prev", d)
  env._robotlab_away_prev = d.clone()
  away = _mid_open(env).bool() & (d > AWAY_DIST) & ((d - dp) / env.step_dt > AWAY_RATE)
  tz = getattr(env, "_robotlab_turn_xy", None)
  if tz is not None:                                   # goto v7: swinging wide through the U-turn is no reason
    xy, _ = car_pose(env)
    away &= torch.linalg.norm(tz - xy[:, None], dim=-1).amin(-1) > TURN_ZONE
  return away


# goto v7 smoothness (floor goto17, 2026-11-08). The ONNX steering output was +-4..10 (valid +-1) and flipped sign every
# ~0.6 s on a straight hall; the steering-command rate limit turned it into a +-15 deg zig-zag (steer std 10 deg,
# 1.7-2.8 flips/s, yaw-rate std 26 deg/s), and the speed output made 22 reverse blips of 1-9 cm. steer_rate (-20 on the
# executed command) cost only ~0.01 per zig-zag step, and in the sim a clamped +-5 is free.
STEER_FLIP_MIN = 0.02   # change of the normalized steering command that counts as a move (rate limit 0.1 per step)


def steer_flip(env) -> torch.Tensor:
  """1 when the executed steering command starts moving the other way (a zig-zag reversal)."""
  cmd = env.action_manager.get_term("drive")._steer_cmd
  st = getattr(env, "_robotlab_flip", None)
  if st is None:
    st = env._robotlab_flip = {"prev": cmd.clone(), "dir": torch.zeros_like(cmd)}
  d = cmd - st["prev"]
  st["prev"] = cmd.clone()
  fresh = env.episode_length_buf <= 1
  moving = d.abs() > STEER_FLIP_MIN
  flip = moving & (torch.sign(d) * st["dir"] < 0) & ~fresh
  st["dir"] = torch.where(fresh, 0.0, torch.where(moving, torch.sign(d), st["dir"]))
  return flip.float()


STRAIGHT_BEND = math.radians(10.0)
STRAIGHT_HEADING = math.radians(20.0)   # v7d: car heading within this of the route direction (ours)   # route turns less than this over the next 1.5 m: a straight stretch (ours)


def straight_yaw(env) -> torch.Tensor:
  """goto v7c: squared yaw rate (rad/s) while the route ahead is straight and the car is near it and driving forward.
  The weave on straights is a feedback loop (steer hard at the line, overshoot through the steering lag, steer back):
  in v7b, CAPS cut the policy mean's step-to-step change 3.1 -> 0.7 and the out-of-range part 4.4 -> 1.5, yet the
  steering std on straights stayed 10.1-10.4 deg. This penalises the weave itself (goto17 floor: yaw-rate std 26 deg/s)."""
  nv = N.nav(env)
  p = N._points_at(nv, nv.s[:, None] + torch.tensor([0.0, 0.75, 1.5], device=env.device))
  a1, a2 = p[:, 1] - p[:, 0], p[:, 2] - p[:, 1]
  bend = torch.atan2(a1[:, 0] * a2[:, 1] - a1[:, 1] * a2[:, 0], (a1 * a2).sum(-1)).abs()
  car = env.scene["car"].data
  _, yaw = car_pose(env)
  head = torch.atan2(torch.sin(yaw - torch.atan2(a1[:, 1], a1[:, 0])), torch.cos(yaw - torch.atan2(a1[:, 1], a1[:, 0])))
  # v7d: only while already heading along the route (v7c 9000 without it: the turn after a U, whose way back is straight,
  # was penalised; turn-backs 83% -> 52%, runs lost 10% -> 35%)
  ok = (bend < STRAIGHT_BEND) & (car.root_link_lin_vel_b[:, 0] > 0.2) & (N.route_distance(env) < 0.5) & (
    head.abs() < STRAIGHT_HEADING)
  return car.root_link_ang_vel_b[:, 2].pow(2) * ok.float()


def action_bound(env) -> torch.Tensor:
  """Squared excess of the raw actions over +-1 (the drive clamps them): keeps the outputs where a change matters."""
  raw = env.action_manager.get_term("drive").raw_action
  return ((raw.abs() - 1.0).clamp(min=0.0) ** 2).sum(-1)


def policy_reverse(env) -> torch.Tensor:
  """1 when the policy itself drives backwards (not the scripted back-up). The car raises |v| < 0.03 m/s to 0.03, so the
  sim's harmless -0.003 m/s is a real reverse blip on the floor (explore_policy.V_MIN)."""
  drv = env.action_manager.get_term("drive")
  return ((drv.v_cmd < -0.005) & ~drv.backing).float()


# RL fine-tuning after DAgger (2026-11-08; UZH "student RL fine-tuning", 40% -> 100%; warning in Weihs et al. 2021 that
# RL after imitation can unlearn it): reward staying near the teacher's action, so PPO improves on the student without
# drifting back to the bang-bang PPO policy. IMITATE_W from the env var ROBOTLAB_IMITATE_W (0 = off, the default).
IMITATE_W = float(__import__("os").environ.get("ROBOTLAB_IMITATE_W", "0"))
IMITATE_EVERY = 2


def teacher_imitation(env) -> torch.Tensor:
  """Negative L1 distance of the policy's (clamped) action to the teacher's for the same state."""
  lab = getattr(env, "_robotlab_teacher_label", None)
  if lab is None:
    return torch.zeros(env.num_envs, device=env.device)
  a = env.action_manager.get_term("drive").raw_action.clamp(-1, 1)
  return -(a - lab).abs().sum(-1)


def stop_held(env) -> torch.Tensor:
  """How long the obstacle stop has held the car, 0 -> 1 over STOP_HELD_FULL s (goto v6c; car: explore_policy.py)."""
  h = getattr(env, "_robotlab_held", None)
  h = torch.zeros(env.num_envs, device=env.device) if h is None else h
  return (h / STOP_HELD_FULL).clamp(0, 1)[:, None]


def last_drive(env) -> torch.Tensor:
  """The drive command executed in the last step, normalized like the action: speed / v_max forward, / v_reverse
  backward (after the shield and back-up), steering command (after the rate limit). 2026-11-08 student v2: the
  teacher's multi-point turn and reverse hold are modes the sensors don't show; with its own last command the student
  can continue a move it started (car: explore_policy.py appends the same two values)."""
  drv = env.action_manager.get_term("drive")
  v = drv.v_cmd
  vn = torch.where(v >= 0, v / drv.cfg.v_max, v / max(drv.cfg.v_reverse, 1e-6))
  return torch.stack([vn, drv._steer_cmd], -1)


def goals_reached(env) -> torch.Tensor:
  return N.nav(env).reached.float()


def lost_route(env) -> torch.Tensor:
  dist = N.route_distance(env)
  mid = getattr(env, "_robotlab_midturn_open", None)
  return (dist > LOST_DISTANCE) | ((dist > MIDTURN_LOST) & mid) if mid is not None else dist > LOST_DISTANCE


# Sub-goal variant (GotoSub, 2026-11-04 research): DRL-VO (Xie & Dames, T-RO 2023, arXiv 2301.06512) gives the policy
# one sub-goal taken from the global path instead of the path itself, and Rudin et al. 2022 (arXiv 2209.12827) reward
# arriving, not tracking. So: the path point SUBGOAL_AHEAD m past the progress point (car frame) + remaining length, and
# no on_route term: a detour (e.g. swinging wide to turn into a doorway) costs nothing as long as the car gets there.
SUBGOAL_AHEAD = 1.75     # m (DRL-VO: sub-goal on the global path ~1.5-2 m ahead; design choice within that)


def subgoal(env) -> torch.Tensor:
  nv, (xy, yaw) = N.nav(env), car_pose(env)
  p = N._points_at(nv, nv.s[:, None] + SUBGOAL_AHEAD)[:, 0]
  rel = to_frame(p - xy, yaw)
  remain = (N._remaining(nv).clamp(max=N.REMAIN_CLIP) / N.REMAIN_CLIP)[:, None]
  return torch.cat([rel, remain], -1)


def make_mentorpi_gotosub_env_cfg(play: bool = False):
  cfg = make_mentorpi_goto_env_cfg(play)
  for g in ("actor", "critic"):
    del cfg.observations[g].terms["route"]
    cfg.observations[g].terms["subgoal"] = ObservationTermCfg(func=subgoal)
  del cfg.rewards["on_route"]
  return cfg


def make_mentorpi_gotostudent_env_cfg(play: bool = False):
  """Goto + the last_drive input (student v2, 2026-11-08): the imitation student's task. A separate task id so goto
  checkpoints (1302 inputs) keep loading in Robotlab-Goto; train_student.py pads a goto actor's input for it."""
  cfg = make_mentorpi_goto_env_cfg(play)
  for g in ("actor", "critic"):     # appended after stop_held
    cfg.observations[g].terms["last_drive"] = ObservationTermCfg(func=last_drive)
  return cfg


def enable_carskill(env, env_ids=None) -> None:
  from robotlab.tasks import carskill as _CS
  pol, KS = _CS.make_policy(env, lambda a: a)
  env._carskill_hook, env._carskill_KS = pol, KS


def make_mentorpi_goto_env_cfg(play: bool = False):
  cfg = X.make_mentorpi_explore_env_cfg(play)
  # goto v4: doorways in a third of the hallway segments (office_terrain.door_prob; goto2 froze facing the wall next to
  # the office door, 2026-11-04)
  cfg.scene.terrain.terrain_generator.sub_terrains["office"].door_prob = DOOR_PROB
  cfg.scene.terrain.terrain_generator.sub_terrains["office"].hall_width = HALL_WIDTH   # goto v5
  cfg.scene.terrain.terrain_generator.sub_terrains["office"].chair_prob = CHAIR_PROB   # goto v7
  from robotlab.tasks import office_twin as _TW
  if _TW.TWIN_PATH:                     # office twin exam (2026-11-09): one patch, the real office map
    tg = cfg.scene.terrain.terrain_generator
    tg.sub_terrains = {"office": _TW.OfficeTwinTerrainCfg()}
    tg.num_rows, tg.num_cols = 1, 1
  # goto v8 (office_terrain.DESK_LEG_*): desk legs on the route line + route jitter, so the route alone no longer avoids
  # the obstacles (floor goto18/19). Env-set for now, default off, until the teacher is checked on them.
  cfg.scene.terrain.terrain_generator.sub_terrains["office"].route_leg_prob = float(os.environ.get("ROBOTLAB_ROUTE_LEG", "0"))
  cfg.scene.terrain.terrain_generator.sub_terrains["office"].route_jitter = float(os.environ.get("ROBOTLAB_ROUTE_JITTER", "0"))
  cfg.scene.sensors = tuple(dataclasses.replace(s, pattern=dataclasses.replace(s.pattern, sub=X.DEPTH_SUB))
                            if s.name == "depth" else s for s in cfg.scene.sensors)   # goto v7: 48 depth columns
  cfg.events["depth_map"] = EventTermCfg(func=enable_depth_map, mode="startup")   # goto v7
  # goto v8 (2026-11-09): RL fine-tuning with the CAR's skill layer in the loop (env ROBOTLAB_CARSKILL=1): the policy's
  # actions pass through robotlab/tasks/carskill.py (the audited port of the car node's skill) before the drive term,
  # so the policy learns to work with exactly what runs on the car. Off by default (the exam wraps it itself).
  if os.environ.get("ROBOTLAB_CARSKILL", "0") == "1":
    cfg.events["carskill"] = EventTermCfg(func=enable_carskill, mode="startup")
  cfg.observations["actor"].terms["route"] = ObservationTermCfg(func=N.route_preview, params={"noisy": True})
  cfg.observations["critic"].terms["route"] = ObservationTermCfg(func=N.route_preview)
  for g in ("actor", "critic"):     # goto v6c, appended after route (warm_start.py --actor-add 1 --critic-add 1)
    cfg.observations[g].terms["stop_held"] = ObservationTermCfg(func=stop_held)

  cfg.actions["drive"].leader_step = goto_step
  cfg.actions["drive"].unstick = UNSTICK   # goto v6f: the car's back-and-forth rule in the sim too
  cfg.actions["drive"].unstick_rear_clear = BACK_CLEAR   # goto v6n
  cfg.actions["drive"].unstick_escalate = True   # goto v7
  # 2026-11-08 output conditioning for imitation students (env vars until chosen; car node: explore_policy V_DEAD /
  # STEER_EMA). v_dead: |v| below it is a stop; car #1's motors do not follow < 0.03 m/s (measured 2026-11-03,
  # explore_policy.V_MIN), and the car raised such commands to 0.03 m/s, turning near-zero outputs into real reverse blips.
  _env = __import__("os").environ
  cfg.actions["drive"].v_dead = float(_env.get("ROBOTLAB_V_DEAD", "0"))
  cfg.actions["drive"].steer_ema = float(_env.get("ROBOTLAB_STEER_EMA", "0"))
  cfg.actions["drive"].forward_only = _env.get("ROBOTLAB_FORWARD_ONLY", "0") == "1"
  cfg.actions["drive"].unstick_away = moving_away   # goto v6k: back up too when driving away from the route (eval
  # on v6j 7800: runs lost 10% -> 5.7%; car side: explore_policy.py AWAY_*)
  cfg.events["reset_goto"] = dataclasses.replace(cfg.events["reset_explore"], func=reset_goto)   # after reset_explore
  del cfg.rewards["new_area"]
  # navigate's route terms (progress 2.0 / goal 10 / on_route 0.5); explore's safety and smoothness terms stay.
  cfg.rewards["progress"] = RewardTermCfg(func=progress, weight=2.0)
  cfg.rewards["goal"] = RewardTermCfg(func=goals_reached, weight=10.0)
  cfg.rewards["on_route"] = RewardTermCfg(func=N.on_route, weight=0.5, params={"std": 0.35})
  cfg.terminations["lost"] = TerminationTermCfg(func=lost_route)
  from robotlab.tasks import office_twin as _TW2
  if _TW2.TWIN_PATH:   # twin = the floor: the car node never ends a run off the route, its back-up rule acts instead
    del cfg.terminations["lost"]
  cfg.rewards["route_approach"] = RewardTermCfg(func=route_approach, weight=1.0)   # goto v5c
  cfg.rewards["route_heading"] = RewardTermCfg(func=route_heading, weight=0.3)     # goto v5c
  cfg.rewards["shield_stop"] = dataclasses.replace(cfg.rewards["shield_stop"], weight=SHIELD_STOP_W)   # goto v6
  cfg.rewards["person_ttc"] = dataclasses.replace(cfg.rewards["person_ttc"], weight=PERSON_TTC_W)       # goto v6l
  # goto v7 (x dt 0.05): a flip costs 0.3 (~3/s on goto17 = 0.9/s, ~ full-speed progress 2.0); a turn needs 1-2 flips
  # goto v7c: steer_flip / action_bound dropped (v7: -6 / -0.1, v7b: -1 / -0.02). On the sampled actions they mostly punished
  # exploration noise; with them and CAPS (v7b) runs lost rose 7% -> 27.6% and the weave stayed. straight_yaw penalises the
  # weave itself: 0.45 rad/s (goto17's yaw-rate std) costs 0.2 x 5 x dt = 0.05 a step, half of full-speed progress.
  cfg.rewards["straight_yaw"] = RewardTermCfg(func=straight_yaw, weight=-5.0)
  cfg.rewards["policy_reverse"] = RewardTermCfg(func=policy_reverse, weight=-2.0)
  if IMITATE_W > 0:
    cfg.rewards["teacher_imitation"] = RewardTermCfg(func=teacher_imitation, weight=IMITATE_W)   # -0.1 a step = full-speed progress
  return cfg


# entropy 0.003 (explore v7e: 0.001): with 0.001 the action std fell 0.25 -> 0.09 in 50 iterations (goto_v1), too
# little exploration to learn the new route inputs.
@dataclasses.dataclass
class PpoSymmetryCfg(RslRlPpoAlgorithmCfg):
  """mjlab's PPO config + rsl_rl's symmetry_cfg (goto v6g: left / right mirror augmentation, goto_symmetry.py)."""
  symmetry_cfg: dict[str, str | bool | float] | None = None


GOTO_SYMMETRY = {"data_augmentation_func": "robotlab.tasks.goto_symmetry:mirror", "use_data_augmentation": True,
                 "use_mirror_loss": False, "mirror_loss_coeff": 0.0}
@dataclasses.dataclass
class CapsPpoCfg(PpoSymmetryCfg):
  """goto v7b: PPO + CAPS smoothness on the policy mean (robotlab/tasks/caps_ppo.py). Coefficients: design choices."""
  class_name: str = "robotlab.tasks.caps_ppo:CapsPPO"
  caps_temporal_coef: float = 0.3      # v7c (v7b 1.0 / 0.5 / 1.0, 2 steps: route following fell apart, weave stayed)
  caps_spatial_coef: float = 0.1
  caps_spatial_std: float = 0.05
  caps_bound_coef: float = 0.3
  caps_steps: int = 1
  caps_frac: float = 0.25


MentorPiGotoRlCfg = dataclasses.replace(
  X.MentorPiExploreRlCfg, experiment_name="goto",
  algorithm=CapsPpoCfg(**{**dataclasses.asdict(X.MentorPiExploreRlCfg.algorithm), "entropy_coef": 0.003,
                          "class_name": CapsPpoCfg.class_name},   # the explore dict would set "PPO"
                       symmetry_cfg=None))   # GOTO_SYMMETRY off: v6g (mirror augmentation from v6f 7300)
# collapsed mid-hall turn-backs 84% -> 79% -> 67.5% in 150 iterations, symmetry loss 6.6 -> 10.3 (cause not found)
MentorPiGotoSubRlCfg = dataclasses.replace(MentorPiGotoRlCfg, experiment_name="gotosub",
                                           algorithm=dataclasses.replace(MentorPiGotoRlCfg.algorithm, symmetry_cfg=None))
