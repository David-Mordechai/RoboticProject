"""Explore policy: at each person contact, was the car moving toward the person, or did the person walk into it?"""
import sys
from dataclasses import asdict
import numpy as np, torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg
import robotlab.tasks  # noqa
from robotlab.tasks.mentorpi_explore_env_cfg import explore, hit_person
from robotlab.tasks.mdp import car_pose, to_frame
from robotlab.tasks.people import walkers
T = "Robotlab-Explore-MentorPiA1"
cfg = load_env_cfg(T, play=True); cfg.scene.num_envs = 256
ag = load_rl_cfg(T)
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
venv = RslRlVecEnvWrapper(env, clip_actions=ag.clip_actions)
r = MjlabOnPolicyRunner(venv, asdict(ag), device="cuda:0")
r.load(sys.argv[1], load_cfg={"actor": True}, strict=True, map_location="cuda:0")
pol = r.get_inference_policy(device="cuda:0")
obs = venv.get_observations(); rows = []
with torch.inference_mode():
  for _ in range(20 * 90):
    e, (xy, yaw) = explore(env), car_pose(env)
    w = walkers(env, e.maps, e.layout)
    d = torch.where(w.active, torch.linalg.norm(w.xy - xy[:, None], dim=-1), 1e3); k = d.argmin(-1)
    i = torch.arange(len(k), device=k.device)
    rel = to_frame(w.xy[i, k] - xy, yaw); pv = to_frame(w.vel[i, k], yaw)
    v = env.scene["car"].data.root_link_lin_vel_b[:, 0].clone()
    a = pol(obs); obs, _, dones, _ = venv.step(a)
    hit = hit_person(env) | (dones.bool() & (env.termination_manager.get_term("hit_person")))
    m = env.termination_manager.get_term("hit_person")
    if m.any():
      for j in torch.nonzero(m)[:, 0].tolist():
        rows.append([v[j].item(), rel[j, 0].item(), rel[j, 1].item(), pv[j, 0].item(), pv[j, 1].item()])
R = np.array(rows); v, px, py, vx, vy = R.T
car_close = np.sign(v) * np.sign(px) > 0      # car moving toward the side the person is on
car_closing = np.where(car_close, np.abs(v), 0)
person_closing = -(px * vx + py * vy) / np.hypot(px, py)
print(f"person contacts: {len(R)} in {env.num_envs} cars x 90 s")
print(f"  car stopped (<0.05 m/s): {(np.abs(v)<0.05).mean():.0%}   car moving toward person: {(car_close & (np.abs(v)>=0.05)).mean():.0%}")
print(f"  person closing faster than car: {(person_closing > car_closing).mean():.0%}   car speed median {np.median(np.abs(v)):.2f}  person closing median {np.median(person_closing):.2f}")
print(f"  person in front (|bearing|<45): {(np.abs(np.degrees(np.arctan2(py,px)))<45).mean():.0%}  side: {((np.abs(np.degrees(np.arctan2(py,px)))>=45)&(np.abs(np.degrees(np.arctan2(py,px)))<=135)).mean():.0%}")
