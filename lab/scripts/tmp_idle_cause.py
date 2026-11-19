"""Where does explore's idle time come from? (v6 2026-11-04: the stall penalty did not move idle 20% or the 0.8 speed
ratio with people beside.) Same rollout as tmp_goal_check.py; the shield is wrapped to record its parts per step."""
import sys
from dataclasses import asdict
import math
import numpy as np, torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg
import robotlab.tasks  # noqa
from robotlab.tasks import mentorpi_explore_env_cfg as T
from robotlab.tasks.mdp import car_pose, to_frame
from robotlab.tasks.people import walkers
TASK = "Robotlab-Explore-MentorPiA1"
# variant: argv[2] = approach corridor half width (v7 shield; the v5 cone + hold variants were run before v7)
if len(sys.argv) > 2: T.SHIELD_APPROACH_HALF_W = float(sys.argv[2])
if len(sys.argv) > 3: T.SHIELD_HALF_W = float(sys.argv[3])
if len(sys.argv) > 4: T.SHIELD_FRONT = float(sys.argv[4])
print(f"variant: approach half width {T.SHIELD_APPROACH_HALF_W} m, arc half width {T.SHIELD_HALF_W} m, front {T.SHIELD_FRONT} m")
cfg = load_env_cfg(TASK, play=True); cfg.scene.num_envs = 1024; cfg.episode_length_s = 60.0
cfg.observations["actor"].enable_corruption = True
rec = {}
def wrapped(env, v, steer):
  arc = T.arc_blocked(env, v, steer, T.SHIELD_FRONT, T.SHIELD_REAR, T.SHIELD_TIME, T.SHIELD_HALF_W)
  appr = T.approaching(env, v)
  rec.update(arc=(arc & (v != 0)).clone(), appr=(appr & ~arc & (v != 0)).clone(), vpre=v.clone())
  return (arc | appr) & (v != 0)
cfg.actions["drive"].shield = wrapped
ag = load_rl_cfg(TASK)
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
venv = RslRlVecEnvWrapper(env, clip_actions=ag.clip_actions)
r = MjlabOnPolicyRunner(venv, asdict(ag), device="cuda:0")
r.load(sys.argv[1], load_cfg={"actor": True}, strict=True, map_location="cuda:0")
pol = r.get_inference_policy(device="cuda:0")
n, dev = env.num_envs, env.device
obs = venv.get_observations()
alive = torch.ones(n, dtype=torch.bool, device=dev); PF = []; CD = []; wall = torch.zeros_like(alive)
cols = {k: [] for k in ("v", "vpre", "arc", "appr", "pp", "c", "ah", "t", "a0", "wz", "front")}
with torch.inference_mode():
  for _ in range(20 * 60 - 1):
    e, (xy, yaw) = T.explore(env), car_pose(env)
    w = walkers(env, e.maps, e.layout)
    d = torch.where(w.active, torch.linalg.norm(w.xy - xy[:, None], dim=-1), 1e3); k = d.argmin(-1)
    rel = to_frame(w.xy[torch.arange(n, device=dev), k] - xy, yaw)
    pp, c = T.person_in_path(env), T.person_clearance(env)
    t = env.episode_length_buf.float() * env.step_dt
    v0 = env.scene["car"].data.root_link_lin_vel_b[:, 0].clone()   # speed before the step (fault: as tmp_goal_check)
    a = pol(obs)
    obs, _, dones, _ = venv.step(a)
    # velocity after the step = result of this step's command (and its shield decision)
    v = env.scene["car"].data.root_link_lin_vel_b[:, 0]
    wz = env.scene["car"].data.root_link_ang_vel_b[:, 2]
    m = alive.cpu().numpy()
    st = getattr(env, "_robotlab_lidar_hist_noisy", None)
    front = (st["buf"][:, 0, list(range(0, 3)) + list(range(33, 36))].min(-1).values * T.LIDAR_RANGE) if st else torch.zeros_like(v)
    for key, x in (("v", v), ("vpre", rec["vpre"]), ("arc", rec["arc"]), ("appr", rec["appr"]), ("pp", pp), ("c", c),
                   ("ah", (rel[:, 0] > 0) & (rel[:, 1].abs() < 0.5)), ("t", t), ("a0", a[:, 0]), ("wz", wz), ("front", front)):
      cols[key].append(np.where(m, x.float().cpu().numpy(), np.nan))
    ended = alive & dones.bool(); tm = env.termination_manager
    hitp = ended & tm.get_term("hit_person").bool(); wall |= ended & tm.get_term("bump").bool()
    PF += [(D0, R0) for D0, R0 in zip(v0[hitp].tolist(), rel[hitp, 0].tolist())]
    wv = getattr(w, "vel", None)
    for j in torch.nonzero(hitp)[:, 0].tolist():
      CD.append((v0[j].item(), math.degrees(math.atan2(rel[j, 1].item(), rel[j, 0].item())), pp[j].item(),
                 float(torch.linalg.norm(wv[j, k[j]]).item()) if wv is not None else float("nan")))
    alive &= ~dones.bool()
D = {k: np.array(x) for k, x in cols.items()}
ok = ~np.isnan(D["v"])
idle = ok & (D["pp"] == 0) & (np.abs(D["v"]) < 0.05)
base = ok & (D["pp"] == 0)
print(f"steps alive, no person in path: {base.sum()}; idle (|v|<0.05): {idle.sum()} = {idle.sum()/base.sum():.3f}")
def split(mask, name):
  tot = mask.sum()
  arc, appr = mask & (D["arc"] == 1), mask & (D["appr"] == 1)
  rest = mask & ~(D["arc"] == 1) & ~(D["appr"] == 1)
  pol0 = rest & (np.abs(D["vpre"]) < 0.05)
  mov = rest & ~(np.abs(D["vpre"]) < 0.05)
  early = mask & (D["t"] < 1.0)
  print(f"--- {name}: {tot} steps")
  for lab_, x in (("arc stop (obstacle in the driving arc)", arc), ("approach stop / 5 s hold", appr),
                  ("policy commands ~0 speed", pol0), ("policy commands motion, car not yet moving (accel / reversal)", mov),
                  ("  of all: first 1 s of an episode", early)):
    print(f"  {lab_:62s} {x.sum()/max(tot,1):6.1%}")
  for lab_, x in (("arc", arc), ("pol0", pol0)):
    if x.sum():
      print(f"  {lab_}: front LiDAR min median {np.nanmedian(D['front'][x]):.2f} m, |yaw rate| median {np.nanmedian(np.abs(D['wz'][x])):.2f} rad/s, a0 median {np.nanmedian(D['a0'][x]):+.2f}")
split(idle, "idle with no person in path")
# runs of idle: how long does a stop last?
lens = []
for j in range(idle.shape[1]):
  x = np.concatenate([[0], idle[:, j].astype(int), [0]]); s = np.flatnonzero(np.diff(x) == 1); e = np.flatnonzero(np.diff(x) == -1)
  lens += list((e - s) * env.step_dt)
lens = np.array(lens)
print(f"idle stretches: {len(lens)}, median {np.median(lens):.2f} s, p90 {np.percentile(lens,90):.2f} s, share of idle time in stretches > 2 s: {lens[lens>2].sum()/lens.sum():.1%}")
beside = ok & (D["c"] < 1.2) & (D["ah"] == 0); alone = ok & (D["c"] > 2.0)
for name, mk in (("beside/behind", beside), ("alone", alone)):
  print(f"--- {name}: {mk.sum()} steps, mean |v| {np.nanmean(np.abs(D['v'][mk])):.3f}, mean |cmd before shield| {np.nanmean(np.abs(D['vpre'][mk])):.3f}, "
        f"arc stop {np.mean(D['arc'][mk]==1):.1%}, approach/hold {np.mean(D['appr'][mk]==1):.1%}, reversing {np.mean(D['v'][mk] < -0.05):.1%}")
fault = sum(abs(v) >= 0.05 and np.sign(v) == np.sign(x) for v, x in PF)
print(f"person contacts {len(PF)}, car at fault {fault} = {fault/n:.3f} of runs (goal <= 0.01); wall-ended runs {wall.float().mean().item():.3f}")
CD = np.array(CD).reshape(-1, 4)
if len(CD):
  f = (np.abs(CD[:, 0]) >= 0.05) & (np.sign(CD[:, 0]) == np.sign(np.cos(np.radians(CD[:, 1]))))
  b = np.abs(CD[:, 1])
  for lo, hi in ((0, 20), (20, 45), (45, 90), (90, 180)):
    m = (b >= lo) & (b < hi)
    print(f"  contacts at bearing {lo:3d}-{hi:3d} deg: {m.sum():3d}, at fault {np.sum(m & f):3d}, person in path {np.sum(m & (CD[:,2]>0)):3d}, car |v| median {np.median(np.abs(CD[m,0])) if m.any() else 0:.2f}, person speed median {np.nanmedian(CD[m,3]) if m.any() else 0:.2f}")
