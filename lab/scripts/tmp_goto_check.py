"""Goal check of a goto checkpoint (1024 cars x 60 s, sensor noise on): PASS/FAIL per goal (2026-11-04).
Goals: gets there (goals per minute, few runs lost off the route), fast and without idling, and explore v7g's safety:
no walls, at-fault person contacts (car closed the gap faster than the person over the last 0.5 s) <= 1%."""
import sys
from dataclasses import asdict
import numpy as np, torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg
import robotlab.tasks  # noqa
from robotlab.tasks import mentorpi_explore_env_cfg as T
from robotlab.tasks import mentorpi_navigate_env_cfg as N
from robotlab.tasks.mdp import car_pose
from robotlab.tasks.people import walkers
from robotlab.tasks import office_terrain as OT
import math
TASK = sys.argv[2] if len(sys.argv) > 2 else "Robotlab-Goto-MentorPiA1"   # or Robotlab-GotoSub-MentorPiA1
cfg = load_env_cfg(TASK, play=True); cfg.scene.num_envs = 2048   # 2048 since goto v6l: 1024 gave single lucky passes; cfg.episode_length_s = 60.0
cfg.observations["actor"].enable_corruption = True
ag = load_rl_cfg(TASK)
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
venv = RslRlVecEnvWrapper(env, clip_actions=ag.clip_actions)
if sys.argv[1] == "teacher":   # 2026-11-08: the privileged scripted teacher (robotlab/tasks/teacher.py) takes the exam
  from robotlab.tasks import teacher as TCH
  pol = lambda _obs: TCH.act(env)[0]
elif sys.argv[1].startswith("hybrid:"):   # 2026-11-08: student drives forward; the teacher's manoeuvre skill (multi-point
  # turn, reverse when nothing forward is allowed) takes over when the teacher would reverse or turn (ROBOTLAB_TEACHER_MAP)
  from robotlab.tasks import teacher as TCH
  import os as _os
  NEAR_TURN = float(_os.environ.get("ROBOTLAB_HYBRID_NEAR_TURN", "0"))
  BEND_DEG = float(_os.environ.get("ROBOTLAB_HYBRID_BEND_DEG", "0"))
  HELD_S = float(_os.environ.get("ROBOTLAB_HYBRID_HELD_S", "0.3"))   # held this long -> the skill (car: KTURN_HELD_S)
  r = MjlabOnPolicyRunner(venv, asdict(ag), device="cuda:0")
  r.load(sys.argv[1][7:], load_cfg={"actor": True}, strict=True, map_location="cuda:0")
  stu = r.get_inference_policy(device="cuda:0")
  def pol(o):
    at, _ = TCH.act(env)
    # the skill also takes over once the stop has held the car 0.3 s (= car node kturn_step: held > UNSTICK_AFTER)
    held_ = getattr(env, "_robotlab_held", None)
    skill = env._teacher_kturn["on"] | (at[:, 0] < 0) | ((held_ > HELD_S) if held_ is not None else False)
    # near a turn-back corner (within NEAR_TURN m of the route's U; = car node turn_backs) the skill drives
    # (env ROBOTLAB_HYBRID_NEAR_TURN, 0 = off)
    if NEAR_TURN > 0 and hasattr(env, "_robotlab_turn_xy"):
      xy_, _ = car_pose(env)
      near = torch.linalg.norm(env._robotlab_turn_xy - xy_[:, None], dim=-1).amin(-1) < NEAR_TURN
      skill = skill | (near & env._robotlab_midturn_open)
    # a sharp route bend coming (route direction 0-0.4 m vs 0.4-0.8 m past the progress point differs by more than
    # BEND_DEG; = car node: the same from the route preview) -> the skill steers through it (env, 0 = off)
    if BEND_DEG > 0:
      nv_ = N.nav(env)
      rp_ = N._points_at(nv_, nv_.s[:, None] + torch.tensor([0.0, 0.4, 0.8], device=dev))
      b1, b2 = rp_[:, 1] - rp_[:, 0], rp_[:, 2] - rp_[:, 1]
      bend_ = torch.atan2(b1[:, 0] * b2[:, 1] - b1[:, 1] * b2[:, 0], (b1 * b2).sum(-1)).abs()
      skill = skill | (bend_ > math.radians(BEND_DEG))
    a_s = stu(o).clone()
    # gentle exit after the turn applies to the student too (= car node kturn_step: EXIT_V for KTURN_COOLDOWN)
    cool = (env._teacher_kturn["cool"] > 0) & ~skill
    drv_ = env.action_manager.get_term("drive")
    a_s[:, 0] = torch.where(cool, a_s[:, 0].clamp(max=TCH.EXIT_V / drv_.cfg.v_max), a_s[:, 0])
    return torch.where(skill[:, None], at, a_s)
elif sys.argv[1].startswith("carskill:"):   # 2026-11-09: the honest exam. The student + the CAR's own skill
  # (robotlab/tasks/carskill.py, a port of car/policy/explore_policy.py, audited). Floor goto18/19 failed while the
  # 'hybrid' exam (teacher as skill) said 16/16.
  from robotlab.tasks import carskill as CS
  if sys.argv[1][9:] == "teacher":   # diagnostic: the teacher drives forward, the car skill does the rest
    from robotlab.tasks import teacher as TCH
    def stu(_o):
      at = TCH.act(env)[0].clone(); at[:, 0] = at[:, 0].clamp(min=0.0); return at
  else:
    r = MjlabOnPolicyRunner(venv, asdict(ag), device="cuda:0")
    r.load(sys.argv[1][9:], load_cfg={"actor": True}, strict=True, map_location="cuda:0")
    stu = r.get_inference_policy(device="cuda:0")
  if getattr(env, "_carskill_hook", None) is not None:   # ROBOTLAB_CARSKILL=1: the env applies the skill already
    pol, KS = stu, env._carskill_KS
  else:
    pol, KS = CS.make_policy(env, stu)
else:
  r = MjlabOnPolicyRunner(venv, asdict(ag), device="cuda:0")
  r.load(sys.argv[1], load_cfg={"actor": True}, strict=True, map_location="cuda:0")
  pol = r.get_inference_policy(device="cuda:0")
n, dev = env.num_envs, env.device
# chair ids per layout: clutter ids whose footprint is a chair base (office_terrain marks chairs with fill_disc of
# radius CHAIR_R; the generator keeps no kind per id, so the chair ids are recorded there: OT.LAYOUTS[k].chairs)
CHAIRS = None
if all(hasattr(l, "chairs") for l in OT.LAYOUTS[-T.explore(env).maps.occ.shape[0]:]):
  ls = OT.LAYOUTS[-T.explore(env).maps.occ.shape[0]:]
  CHAIRS = torch.zeros(len(ls), OT.MAX_CLUTTER + 1, dtype=torch.bool, device=dev)
  for i, l in enumerate(ls):
    for c in l.chairs:
      CHAIRS[i, c] = True
obs = venv.get_observations()
alive = torch.ones(n, dtype=torch.bool, device=dev)
ended = {k: torch.zeros_like(alive) for k in ("bump", "hit_person", "lost")}
goals = torch.zeros(n, device=dev); V, PP = [], []
LOST_MID = LOST_OTHER = 0
MID = env._robotlab_midturn.clone(); TS = env._robotlab_turn_s.clone()   # goto v5 mid-hallway turn-back routes
CRN = env._robotlab_corner.clone(); crn_done = torch.zeros_like(CRN); crn_held = torch.zeros(n, device=dev)   # v6d
BLK = env._robotlab_blocked.clone(); blk_done = torch.zeros_like(BLK); blk_held = torch.zeros(n, device=dev)   # goto v6
t_turn0 = torch.full((n,), torch.nan, device=dev); t_turn1 = t_turn0.clone(); t_goal1 = t_turn0.clone()
CL = torch.zeros(10, n, 2, device=dev); FAULT = []
# goto v7 (floor goto17): steering on straight route stretches, and the policy's own reverse blips
drv = env.action_manager.get_term("drive")
ST, STRAIGHT = [], []                     # executed steering command (deg) per step; mask: straight stretch at speed
prev_cmd = drv._steer_cmd.clone(); last_dir = torch.zeros(n, device=dev); flips = torch.zeros(n, device=dev)
straight_t = torch.zeros(n, device=dev); rev_blips = torch.zeros(n, device=dev); rev_len = torch.zeros(n, device=dev)
rev_blips_free = torch.zeros(n, device=dev); rev_start_free = torch.zeros(n, dtype=torch.bool, device=dev)
chair_bump = torch.zeros_like(alive); CH_CLS = {}; ID_CLS = {}
with torch.inference_mode():
  for it in range(20 * 60 - 1):
    e, (xy, yaw) = T.explore(env), car_pose(env)
    w = walkers(env, e.maps, e.layout)
    dd = torch.where(w.active, torch.linalg.norm(w.xy - xy[:, None], dim=-1), 1e3); k = dd.argmin(-1)
    ar = torch.arange(n, device=dev)
    dw = w.xy[ar, k] - xy; u = dw / torch.linalg.norm(dw, dim=-1, keepdim=True).clamp(min=1e-3)
    vc = env.scene["car"].data.root_link_lin_vel_b[:, 0]
    CL = torch.roll(CL, 1, 0)
    CL[0, :, 0] = (torch.stack([torch.cos(yaw), torch.sin(yaw)], -1) * vc[:, None] * u).sum(-1)
    CL[0, :, 1] = -(w.vel[ar, k] * u).sum(-1)
    pp = T.person_in_path(env)
    mo = env._robotlab_midturn_open.clone()                # route type before the step (a reset re-draws it)
    cn = getattr(env, "_robotlab_corner", torch.zeros_like(mo)).clone()
    obs, _, dones, _ = venv.step(pol(obs))
    nv, step_t = N.nav(env), (it + 1) * env.step_dt
    # straight: the route turns < 10 deg over the next 1.5 m, the car drives forward > 0.3 m/s, no stop or back-up
    rp = N._points_at(nv, nv.s[:, None] + torch.tensor([0.0, 0.75, 1.5], device=dev))
    a1, a2 = rp[:, 1] - rp[:, 0], rp[:, 2] - rp[:, 1]
    bend = torch.atan2(a1[:, 0] * a2[:, 1] - a1[:, 1] * a2[:, 0], (a1 * a2).sum(-1)).abs()
    vx = env.scene["car"].data.root_link_lin_vel_b[:, 0]
    st_m = alive & (bend < math.radians(10)) & (vx > 0.3) & ~drv.shielded & ~drv.backing & (env.episode_length_buf > 1)
    # v3 (2026-11-09, goto v8 desk legs on the route): only stretches with nothing within 0.35 m of the straight line
    # 1.2 m ahead (LiDAR / depth), so steering around an obstacle is not counted as weaving (floor complaint: weaving
    # with nothing in the way)
    st_m &= ~T.arc_blocked(env, torch.full((n,), 0.3, device=dev), torch.zeros(n, device=dev), 1.2, 0.2, 0.0, 0.35)
    cmd = drv._steer_cmd.clone(); dcmd = cmd - prev_cmd; prev_cmd = cmd
    mv = dcmd.abs() > 0.02
    flips += (st_m & mv & (torch.sign(dcmd) * last_dir < 0)).float()
    last_dir = torch.where(mv, torch.sign(dcmd), last_dir)
    straight_t += st_m.float() * env.step_dt
    ST.append(torch.rad2deg(cmd * drv.steer_max).cpu().numpy()); STRAIGHT.append(st_m.cpu().numpy())
    # a reverse blip: a reverse of the car's own choice (not the scripted back-up) that lasts < 0.5 s (floor goto17: 22 of
    # 0.05-0.5 s); longer ones are deliberate moves (the teacher's multi-point turn)
    rv = alive & (drv.v_cmd < -0.005) & ~drv.backing
    ended_rev = ~rv & (rev_len > 0)
    rev_blips += (ended_rev & (rev_len < 0.5)).float()
    # v2 (2026-11-08): a blip only if driving forward was possible when the reverse started (the shield's corridor would
    # not stop 0.3 m/s straight ahead): the user's complaint was "reverse for no reason, then the same path". Forced
    # reverses in a multi-point turn (wall ahead) are not counted.
    fwd_free = ~T.arc_blocked(env, torch.full((n,), 0.3, device=dev), torch.zeros(n, device=dev), T.SHIELD_FRONT,
                              T.SHIELD_REAR, T.SHIELD_TIME, T.SHIELD_HALF_W)
    rev_start_free = torch.where(rv & (rev_len == 0), fwd_free, rev_start_free)
    rev_blips_free += (ended_rev & (rev_len < 0.5) & rev_start_free).float()
    rev_len = torch.where(rv, rev_len + env.step_dt, 0.0)
    t_turn0 = torch.where(MID & alive & t_turn0.isnan() & (nv.s >= TS - 0.3), step_t, t_turn0)
    # v2 (2026-11-09): the old end test (route progress >= TS + 0.5) fired in the same step as the start, because the out
    # and back legs overlap and the nearest-point progress jumps onto the back leg -> median 0 s. Now the turn ends when
    # the car is within 0.6 m of the back leg's point TS + 0.5 and heads within 30 deg of the back leg's direction.
    xy1, yaw1 = car_pose(env)
    bp = N._points_at(nv, torch.stack([TS + 0.5, TS + 1.0], -1))
    bd = bp[:, 1] - bp[:, 0]
    herr = torch.atan2(torch.sin(yaw1 - torch.atan2(bd[:, 1], bd[:, 0])), torch.cos(yaw1 - torch.atan2(bd[:, 1], bd[:, 0])))
    turned = (herr.abs() < math.radians(30)) & (torch.linalg.norm(xy1 - bp[:, 0], dim=-1) < 0.6)
    t_turn1 = torch.where(MID & alive & t_turn1.isnan() & ~t_turn0.isnan() & turned, step_t, t_turn1)
    t_goal1 = torch.where(MID & alive & t_goal1.isnan() & nv.reached, step_t, t_goal1)
    blk_held += (BLK & alive & ~blk_done & env.action_manager.get_term("drive").shielded).float() * env.step_dt
    blk_done |= BLK & alive & nv.reached
    crn_held += (CRN & alive & ~crn_done & env.action_manager.get_term("drive").shielded).float() * env.step_dt
    crn_done |= CRN & alive & nv.reached
    goals += nv.reached.float() * alive
    tm = env.termination_manager
    fin = alive & dones.bool()
    for key in ended:
      ended[key] |= fin & tm.get_term(key).bool()
    # a bump within 0.45 m of a chair (OCC cells of a chair base carry its clutter id; chairs are the only clutter
    # with a leg reach < 0.35 m that the LiDAR does not see: checked by footprint radius below)
    if CHAIRS is not None:
      bmp = fin & tm.get_term("bump").bool()
      if bmp.any():
        e_ = T.explore(env); xy_ = xy          # pose before the step: a finished car is already reset to a new start
        ix = (xy_[:, 0] / OT.OCC_RES).long().clamp(0, OT.OCC_N - 1); iy = (xy_[:, 1] / OT.OCC_RES).long().clamp(0, OT.OCC_N - 1)
        lay = e_.layout
        near = torch.zeros_like(bmp)
        for dx in range(-9, 10, 3):
          for dy in range(-9, 10, 3):
            cid = e_.maps.clutter[lay, (ix + dx).clamp(0, OT.OCC_N - 1), (iy + dy).clamp(0, OT.OCC_N - 1)]
            near |= CHAIRS[lay, cid.clamp(min=0)] & (cid > 0)
        chair_bump |= bmp & near
        if "KS" in globals():   # carskill: what the car was doing at the chair hit (diagnostic)
          for i in (bmp & near).nonzero()[:, 0].tolist():
            cls = (("turn fwd leg" if float(drv.v_cmd[i]) > 0 else "turn rev leg" if float(drv.v_cmd[i]) < 0 else "turn waiting") if bool(KS["on"][i]) else "steer-around" if float(KS["av"][i]) > 0
                   else "back-up" if bool(drv.backing[i]) else "reversing" if float(drv.v_cmd[i]) < 0
                   else "driving forward")
            CH_CLS[cls] = CH_CLS.get(cls, 0) + 1
    lo = fin & tm.get_term("lost").bool()
    LOST_MID += int((lo & mo).sum()); LOST_OTHER += int((lo & ~mo).sum())
    hitp = fin & tm.get_term("hit_person").bool()
    FAULT += [bool(CL[:, j, 0].mean() > CL[:, j, 1].mean()) for j in torch.nonzero(hitp)[:, 0].tolist()]
    if "KS" in globals():   # carskill: idle steps (|v| < 0.05, no person in the path) by what the car was doing
      idle_ = alive & ~pp & (env.scene["car"].data.root_link_lin_vel_b[:, 0].abs() < 0.05)
      sh_, bk_, vc_ = drv.shielded, drv.backing, drv.v_cmd
      for nm_, mk_ in (("turn: stop holds", KS["on"] & sh_), ("turn: waiting (both legs blocked)", KS["on"] & ~sh_ & (vc_ == 0)),
                       ("turn: moving off", KS["on"] & ~sh_ & (vc_ != 0)),
                       ("steer-around", ~KS["on"] & (KS["av"] > 0)), ("back-up", ~KS["on"] & bk_),
                       ("stop holds: first 0.15 s", ~KS["on"] & (KS["av"] <= 0) & ~bk_ & sh_ & (getattr(env, "_robotlab_held", torch.zeros(n, device=dev)) <= 0.15)),
                       ("stop holds: turn cooldown", ~KS["on"] & (KS["av"] <= 0) & ~bk_ & sh_ & (getattr(env, "_robotlab_held", torch.zeros(n, device=dev)) > 0.15) & (KS["cool"] > 0)),
                       ("stop holds: nothing acts", ~KS["on"] & (KS["av"] <= 0) & ~bk_ & sh_ & (getattr(env, "_robotlab_held", torch.zeros(n, device=dev)) > 0.15) & (KS["cool"] <= 0)),
                       ("driver commands 0", ~KS["on"] & (KS["av"] <= 0) & ~bk_ & ~sh_ & (vc_ == 0)),
                       ("accelerating", ~KS["on"] & (KS["av"] <= 0) & ~bk_ & ~sh_ & (vc_ != 0))):
        ID_CLS[nm_] = ID_CLS.get(nm_, 0) + int((idle_ & mk_).sum())
    m = alive.cpu().numpy()
    V.append(np.where(m, env.scene["car"].data.root_link_lin_vel_b[:, 0].cpu().numpy(), np.nan)); PP.append(pp.cpu().numpy())
    alive &= ~fin
V, PP = np.array(V), np.array(PP)
STa, SM = np.array(ST), np.array(STRAIGHT)
st_std = float(np.nanmean([STa[SM[:, j], j].std() for j in range(n) if SM[:, j].sum() >= 20])) if SM.any() else 99.0
# v2 (2026-11-08): the weave only: steering minus its own 1 s moving mean, on straight stretches. v1 also counted the slow
# settling onto the route after a turn (teacher trace: +8 -> 0 deg over 3 s, no zig-zag; v1 std 5.0 deg)
_k = 20
_mov = np.apply_along_axis(lambda c: np.convolve(c, np.ones(_k) / _k, mode="same"), 0, STa)
_hp = STa - _mov
st_wob = float(np.nanmean([_hp[SM[:, j], j].std() for j in range(n) if SM[:, j].sum() >= 20])) if SM.any() else 99.0
ok = ~np.isnan(V)
res = [
  ("goals reached per minute", goals.mean().item(), ">=", 2.0, "routes are ~3-8 m"),
  ("runs lost off the route", ended["lost"].float().mean().item(), "<=", 0.05, "goto_env_cfg.LOST_DISTANCE"),
  ("mean speed (m/s)", np.nanmean(np.abs(V)), ">=", 0.35, "explore v7g ~0.6"),
  ("idle share (|v| < 0.05) with no person in the path", (np.abs(V[ok & ~PP.astype(bool)]) < 0.05).mean(), "<=", 0.05, ""),
  ("runs ended by wall contact", ended["bump"].float().mean().item(), "<=", 0.05, ""),
  ("runs ended by person contact, car at fault", sum(FAULT) / n, "<=", 0.01, "car closed the gap faster"),
]
sec = max(float(straight_t.sum()), 1e-6)
res += [
  ("steering reversals per second on straight route stretches", float(flips.sum()) / sec, "<=", 0.5,
   "goto v7; floor goto17 (v6p 8750): 1.7-2.8/s"),
  ("steering weave on straights, deg (steering minus its 1 s mean; per car)", st_wob, "<=", 4.0,
   "v2 2026-11-08; floor goto17 zig-zag +-15 deg"),
  ("reverse blips (< 0.5 s) with forward free, per minute", float(rev_blips_free.sum()) / max(float(np.sum(ok)) * env.step_dt / 60, 1e-6),
   "<=", 1.0, "v2 2026-11-08: the user's 'reverse for no reason'"),
]
info = [
  ("[info, v1] steering std on straight stretches, deg (includes settling after turns)", st_std),
  ("[info, v1] reverse blips (< 0.5 s, not the back-up) per minute", float(rev_blips.sum()) / max(float(np.sum(ok)) * env.step_dt / 60, 1e-6)),
]
if CHAIRS is not None:
  res += [("runs ended by hitting a chair base", chair_bump.float().mean().item(), "<=", 0.01, "goto v7; floor goto17")]
  if CH_CLS: print("INFO  chair hits by state:", CH_CLS)
if ID_CLS:
  _t = max(sum(ID_CLS.values()), 1); print("INFO  idle steps by state (%):", {k: round(100 * v_ / _t, 1) for k, v_ in sorted(ID_CLS.items(), key=lambda kv: -kv[1])})
nm = max(int(MID.sum()), 1)
turn_t = (t_turn1 - t_turn0)[MID & ~t_turn1.isnan()].cpu().numpy()
res += [
  ("mid-hallway turn-back routes completed", float((MID & ~t_goal1.isnan()).sum()) / nm, ">=", 0.9, "goto v5, floor trip"),
  ("mid-hallway turn time, median s (0.3 m before to 0.5 m after the turn point)",
   float(np.median(turn_t)) if len(turn_t) else 99.0, "<=", 8.0, "goto8 floor: 20 s with the turnaround hand-over"),
]
nb = max(int(BLK.sum()), 1)
res += [
  ("blocked mid-turn starts completed", float(blk_done.sum()) / nb, ">=", 0.9, "goto v6, floor goto9/10"),
  ("blocked starts: mean time held by the stop, s", float(blk_held[BLK].mean()) if BLK.any() else 99.0, "<=", 1.0,
   "floor goto9/10: 15+ s"),
]
nc = max(int(CRN.sum()), 1)
res += [
  ("corner starts completed", float(crn_done.sum()) / nc, ">=", 0.9, "goto v6d, floor goto11 door frame"),
  ("corner starts: mean time held by the stop, s", float(crn_held[CRN].mean()) if CRN.any() else 99.0, "<=", 1.0, "goto11: 15+ s"),
]
print(f"corner starts: {int(CRN.sum())}, completed {int(crn_done.sum())}")
print(f"blocked starts: {int(BLK.sum())}, completed {int(blk_done.sum())}")
print(f"mid-hallway routes: {int(MID.sum())}, turned {len(turn_t)}, completed {int((MID & ~t_goal1.isnan()).sum())}")
print(f"lost runs: {LOST_MID} on a mid-hallway turn-back route, {LOST_OTHER} on other routes (of {n})")
ok_all = True
for name, val, op, thr, ref in res:
  good = val <= thr if op == "<=" else val >= thr
  ok_all &= bool(good)
  print(f"{'PASS' if good else 'FAIL'}  {name}: {val:.3f} (goal {op} {thr}; {ref})")
for name, val in info:
  print(f"INFO  {name}: {val:.3f}")
print(f"person contacts: {ended['hit_person'].float().mean().item():.3f} of runs, car closed faster in {sum(FAULT)}")
print("ALL GOALS MET" if ok_all else "NOT YET")
