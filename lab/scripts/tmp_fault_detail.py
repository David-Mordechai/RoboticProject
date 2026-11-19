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
TASK = sys.argv[2] if len(sys.argv) > 2 else "Robotlab-Goto-MentorPiA1"   # or Robotlab-GotoSub-MentorPiA1
cfg = load_env_cfg(TASK, play=True); cfg.scene.num_envs = 2048   # 2048 since goto v6l: 1024 gave single lucky passes; cfg.episode_length_s = 60.0
cfg.observations["actor"].enable_corruption = True
ag = load_rl_cfg(TASK)
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
venv = RslRlVecEnvWrapper(env, clip_actions=ag.clip_actions)
r = MjlabOnPolicyRunner(venv, asdict(ag), device="cuda:0")
r.load(sys.argv[1], load_cfg={"actor": True}, strict=True, map_location="cuda:0")
pol = r.get_inference_policy(device="cuda:0")
n, dev = env.num_envs, env.device
obs = venv.get_observations()
alive = torch.ones(n, dtype=torch.bool, device=dev)
ended = {k: torch.zeros_like(alive) for k in ("bump", "hit_person", "lost")}
goals = torch.zeros(n, device=dev); V, PP = [], []
LOST_MID = LOST_OTHER = 0
DET = []
MID = env._robotlab_midturn.clone(); TS = env._robotlab_turn_s.clone()   # goto v5 mid-hallway turn-back routes
CRN = env._robotlab_corner.clone(); crn_done = torch.zeros_like(CRN); crn_held = torch.zeros(n, device=dev)   # v6d
BLK = env._robotlab_blocked.clone(); blk_done = torch.zeros_like(BLK); blk_held = torch.zeros(n, device=dev)   # goto v6
t_turn0 = torch.full((n,), torch.nan, device=dev); t_turn1 = t_turn0.clone(); t_goal1 = t_turn0.clone()
CL = torch.zeros(10, n, 2, device=dev); FAULT = []
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
    drv = env.action_manager.get_term("drive")
    SNAP_V = env.scene["car"].data.root_link_lin_vel_b[:, 0].clone(); SNAP_S = drv._steer[:, 0].clone()
    SNAP_SH = drv.shielded.clone(); SNAP_BK = (drv.back_left > 0).clone()
    rel = w.xy[ar, k] - xy; SNAP_D = torch.linalg.norm(rel, dim=-1)
    SNAP_B = torch.rad2deg(torch.atan2(rel[:, 1], rel[:, 0]) - yaw); SNAP_B = (SNAP_B + 180) % 360 - 180
    SNAP_PV = torch.linalg.norm(w.vel[ar, k], dim=-1)
    mo = env._robotlab_midturn_open.clone()                # route type before the step (a reset re-draws it)
    cn = getattr(env, "_robotlab_corner", torch.zeros_like(mo)).clone()
    obs, _, dones, _ = venv.step(pol(obs))
    nv, step_t = N.nav(env), (it + 1) * env.step_dt
    t_turn0 = torch.where(MID & alive & t_turn0.isnan() & (nv.s >= TS - 0.3), step_t, t_turn0)
    t_turn1 = torch.where(MID & alive & t_turn1.isnan() & (nv.s >= TS + 0.5), step_t, t_turn1)
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
    lo = fin & tm.get_term("lost").bool()
    LOST_MID += int((lo & mo).sum()); LOST_OTHER += int((lo & ~mo).sum())
    hitp = fin & tm.get_term("hit_person").bool()
    for j in torch.nonzero(hitp)[:, 0].tolist():
      f = bool(CL[:, j, 0].mean() > CL[:, j, 1].mean()); FAULT.append(f)
      if f:
        DET.append(dict(v=float(SNAP_V[j]), steer=float(SNAP_S[j]), shield=bool(SNAP_SH[j]), back=bool(SNAP_BK[j]),
                        carclose=float(CL[:, j, 0].mean()), pclose=float(CL[:, j, 1].mean()), mid=bool(mo[j]),
                        bear=float(SNAP_B[j]), dist=float(SNAP_D[j]), pspeed=float(SNAP_PV[j])))
    m = alive.cpu().numpy()
    V.append(np.where(m, env.scene["car"].data.root_link_lin_vel_b[:, 0].cpu().numpy(), np.nan)); PP.append(pp.cpu().numpy())
    alive &= ~fin
V, PP = np.array(V), np.array(PP)
ok = ~np.isnan(V)
res = [
  ("goals reached per minute", goals.mean().item(), ">=", 2.0, "routes are ~3-8 m"),
  ("runs lost off the route", ended["lost"].float().mean().item(), "<=", 0.05, "goto_env_cfg.LOST_DISTANCE"),
  ("mean speed (m/s)", np.nanmean(np.abs(V)), ">=", 0.35, "explore v7g ~0.6"),
  ("idle share (|v| < 0.05) with no person in the path", (np.abs(V[ok & ~PP.astype(bool)]) < 0.05).mean(), "<=", 0.05, ""),
  ("runs ended by wall contact", ended["bump"].float().mean().item(), "<=", 0.05, ""),
  ("runs ended by person contact, car at fault", sum(FAULT) / n, "<=", 0.01, "car closed the gap faster"),
]
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
import numpy as _np
print(f"AT-FAULT contacts: {len(DET)}")
for d in DET[:30]:
  print(f"  car v {d['v']:+.2f} steer {d['steer']:+.2f} shield {int(d['shield'])} back {int(d['back'])} mid {int(d['mid'])} | person {d['bear']:+4.0f} deg {d['dist']:.2f} m speed {d['pspeed']:.2f} | closing car {d['carclose']:.2f} person {d['pclose']:.2f}")
if DET:
  a = lambda k: _np.array([d[k] for d in DET])
  print(f"  shield on {a('shield').mean():.0%}, backing {a('back').mean():.0%}, on turn-back route {a('mid').mean():.0%}, car v mean {a('v').mean():.2f}, |bearing|<30: {(abs(a('bear'))<30).mean():.0%}, 30-90: {((abs(a('bear'))>=30)&(abs(a('bear'))<90)).mean():.0%}, >=90: {(abs(a('bear'))>=90).mean():.0%}, person standing (<0.1): {(a('pspeed')<0.1).mean():.0%}")
ok_all = True
for name, val, op, thr, ref in res:
  good = val <= thr if op == "<=" else val >= thr
  ok_all &= bool(good)
  print(f"{'PASS' if good else 'FAIL'}  {name}: {val:.3f} (goal {op} {thr}; {ref})")
print(f"person contacts: {ended['hit_person'].float().mean().item():.3f} of runs, car closed faster in {sum(FAULT)}")
print("ALL GOALS MET" if ok_all else "NOT YET")
