"""Quick goal check of an explore checkpoint during training (1024 cars x 60 s, sensor noise on): PASS/FAIL per goal.

Goals (explore v4e): smooth steering, slows near people and passes or waits, never drives into people, keeps
exploring as well as v3. Thresholds are ours (v3 values in brackets)."""
import sys
from dataclasses import asdict
import numpy as np, torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg
import robotlab.tasks  # noqa
from robotlab.tasks import mentorpi_explore_env_cfg as T
from robotlab.tasks.mdp import car_pose, to_frame
from robotlab.tasks.people import walkers
TASK = "Robotlab-Explore-MentorPiA1"
cfg = load_env_cfg(TASK, play=True); cfg.scene.num_envs = 1024; cfg.episode_length_s = 60.0
cfg.observations["actor"].enable_corruption = True
ag = load_rl_cfg(TASK)
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
venv = RslRlVecEnvWrapper(env, clip_actions=ag.clip_actions)
r = MjlabOnPolicyRunner(venv, asdict(ag), device="cuda:0")
r.load(sys.argv[1], load_cfg={"actor": True}, strict=True, map_location="cuda:0")
pol = r.get_inference_policy(device="cuda:0")
n, dev = env.num_envs, env.device
obs = venv.get_observations()
alive = torch.ones(n, dtype=torch.bool, device=dev); wall = torch.zeros_like(alive); person = torch.zeros_like(alive)
cov = torch.zeros(n, device=dev); S, V, W, C, AH, PF, PP = [], [], [], [], [], [], []
# v7g: at-fault by closing speed (DRL-VO-style at-fault contact): over the last 0.5 s before a contact, did the car close
# the gap to the nearest person faster than the person did? (v7g 3500: the person closed faster in 88% of contacts;
# walkers never yield to an oncoming car by design, people.py.) Ring buffer of (car share, person share) per env.
CL = torch.zeros(10, n, 2, device=dev); FAULT = []
drive = env.action_manager.get_term("drive")
with torch.inference_mode():
  for _ in range(20 * 60 - 1):
    e, (xy, yaw) = T.explore(env), car_pose(env)
    w = walkers(env, e.maps, e.layout)
    d = torch.where(w.active, torch.linalg.norm(w.xy - xy[:, None], dim=-1), 1e3); k = d.argmin(-1)
    rel = to_frame(w.xy[torch.arange(n, device=dev), k] - xy, yaw)
    dw = w.xy[torch.arange(n, device=dev), k] - xy; u = dw / torch.linalg.norm(dw, dim=-1, keepdim=True).clamp(min=1e-3)
    vc = env.scene["car"].data.root_link_lin_vel_b[:, 0]
    CL = torch.roll(CL, 1, 0)
    CL[0, :, 0] = (torch.stack([torch.cos(yaw), torch.sin(yaw)], -1) * vc[:, None] * u).sum(-1)
    CL[0, :, 1] = -(w.vel[torch.arange(n, device=dev), k] * u).sum(-1)
    v = env.scene["car"].data.root_link_lin_vel_b[:, 0].clone()
    cov_now = T.coverage(env)
    obs, _, dones, _ = venv.step(pol(obs))
    tm = env.termination_manager
    ended = alive & dones.bool()
    hitp = ended & tm.get_term("hit_person").bool()
    wall |= ended & tm.get_term("bump").bool(); person |= hitp
    PF += [(v[j].item(), rel[j, 0].item()) for j in torch.nonzero(hitp)[:, 0].tolist()]
    FAULT += [bool(CL[:, j, 0].mean() > CL[:, j, 1].mean()) for j in torch.nonzero(hitp)[:, 0].tolist()]
    cov = torch.where(alive, cov_now, cov); alive &= ~ended
    m = alive.cpu().numpy()
    S.append(np.where(m, drive._steer_cmd.cpu().numpy(), np.nan)); V.append(np.where(m, v.cpu().numpy(), np.nan))
    W.append(env.scene["car"].data.root_link_ang_vel_b[:, 2].cpu().numpy()); C.append(T.person_clearance(env).cpu().numpy())
    AH.append((rel[:, 0] > 0).cpu().numpy() & (np.abs(rel[:, 1].cpu().numpy()) < 0.5))
    PP.append(T.person_in_path(env).cpu().numpy())
cov = torch.where(alive, T.coverage(env), cov)
S, V, W, C, AH, PP = map(np.array, (S, V, W, C, AH, PP))
ds = np.abs(np.diff(S, axis=0)); straight = (V[1:] > 0.3) & (np.abs(W[1:]) < 0.15)
flips = ((np.sign(S[1:]) != np.sign(S[:-1])) & (np.abs(S[1:]) > 0.1) & (np.abs(S[:-1]) > 0.1))
near = (C > 0.45) & (C < 0.8) & ~np.isnan(V); infront = (C < 1.2) & AH & ~np.isnan(V)
car_fault = np.mean([abs(v) >= 0.05 and np.sign(v) == np.sign(x) for v, x in PF]) if PF else 0.0
# v6 goals (floor test 2026-11-04): people beside / behind must not slow the car; no idling with a clear path.
alive_m = ~np.isnan(V)
beside = alive_m & (C < 1.2) & ~AH                 # nearest person within 1.2 m clearance but not in the corridor ahead
alone = alive_m & (C > 2.0)
idle_clear = alive_m & ~PP
speed_ratio = np.nanmean(np.abs(V[beside])) / max(np.nanmean(np.abs(V[alone])), 1e-6)
res = [
  ("steering change/step, straight", np.nanmean(ds[straight]), "<=", 0.06, "v3 0.152"),
  ("left/right flips per s, straight", flips[straight].mean() * 20, "<=", 0.25, "v3 0.86"),
  ("coverage at 60 s", cov.mean().item(), ">=", 0.65, "v3 ~0.69"),
  ("runs ended by wall contact", wall.float().mean().item(), "<=", 0.05, "v3 ~0.05"),
  ("runs ended by person contact, car at fault", sum(FAULT) / n, "<=", 0.01,
   "v7g: car closed the gap faster than the person over the last 0.5 s"),
  ("speed with person beside/behind / speed alone", speed_ratio, ">=", 0.90, "v6: don't slow for people not in the path"),
  ("idle share (|v| < 0.05) with a clear path", (np.abs(V[idle_clear]) < 0.05).mean(), "<=", 0.05, "v6 anti-freeze"),
  ("mean speed with person <1.2 m ahead (m/s)", np.nanmean(np.abs(V[infront])), "<=", 0.30, "slows down"),
  ("stopped share at 0.45-0.8 m from a person", (np.abs(V[near]) < 0.05).mean(), "<=", 0.30, "v3 0.35"),
]
ok_all = True
for name, val, op, thr, ref in res:
  ok = val <= thr if op == "<=" else val >= thr
  ok_all &= bool(ok)
  print(f"{'PASS' if ok else 'FAIL'}  {name}: {val:.3f} (goal {op} {thr}; {ref})")
print(f"person contacts: {len(PF)} = {len(PF)/n:.3f} of runs; car moving toward the person in {car_fault:.0%} "
      f"(old at-fault {person.float().mean().item() * car_fault:.3f}); car closed faster in {sum(FAULT)}")
print("ALL GOALS MET" if ok_all else "NOT YET")
