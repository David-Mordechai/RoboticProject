"""Every person contact of an explore checkpoint: car speed/steer/shield, person bearing/speed/type, closing speeds."""
import sys, math
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
cfg = load_env_cfg(TASK, play=True); cfg.scene.num_envs = 256; cfg.episode_length_s = 60.0
cfg.observations["actor"].enable_corruption = True
ag = load_rl_cfg(TASK)
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
venv = RslRlVecEnvWrapper(env, clip_actions=ag.clip_actions)
r = MjlabOnPolicyRunner(venv, asdict(ag), device="cuda:0")
r.load(sys.argv[1], load_cfg={"actor": True}, strict=True, map_location="cuda:0")
pol = r.get_inference_policy(device="cuda:0")
drive = env.action_manager.get_term("drive")
obs = venv.get_observations(); rows = []; prev = None
with torch.inference_mode():
  for step in range(20 * 60 - 1):
    e, (xy, yaw) = T.explore(env), car_pose(env)
    w = walkers(env, e.maps, e.layout)
    d = torch.where(w.active, torch.linalg.norm(w.xy - xy[:, None], dim=-1), 1e3); k = d.argmin(-1)
    i = torch.arange(env.num_envs, device=env.device)
    snap = dict(v=env.scene["car"].data.root_link_lin_vel_b[:, 0].clone(), shield=drive.shielded.clone(),
                rel=to_frame(w.xy[i, k] - xy, yaw), pvel=to_frame(w.vel[i, k], yaw), dist=d[i, k],
                distracted=w.distracted[i, k].clone(), attentive=w.attentive[i, k].clone(), cmd=drive.v_cmd.clone())
    obs, _, dones, _ = venv.step(pol(obs))
    m = env.termination_manager.get_term("hit_person")
    for j in torch.nonzero(m)[:, 0].tolist():
      s = {kk: vv[j].item() if vv[j].numel() == 1 else vv[j].tolist() for kk, vv in snap.items()}
      rows.append(s)
print(f"{len(rows)} person contacts")
for s in rows[:40]:
  bx, by = s["rel"]; b = math.degrees(math.atan2(by, bx))
  pv = math.hypot(*s["pvel"])
  print(f"car v {s['v']:+.2f} cmd {s['cmd']:+.2f} shield {int(s['shield'])} | person at {b:+5.0f} deg {s['dist']:.2f} m, "
        f"speed {pv:.2f} vel(car frame) ({s['pvel'][0]:+.2f},{s['pvel'][1]:+.2f}) {'distracted' if s['distracted'] else ('attentive' if s['attentive'] else 'oblivious')}")
v = np.array([s["v"] for s in rows]); b = np.array([math.degrees(math.atan2(s["rel"][1], s["rel"][0])) for s in rows])
pv = np.array([math.hypot(*s["pvel"]) for s in rows])
print(f"car stopped (<0.05): {(np.abs(v)<0.05).mean():.0%}  car reversing: {(v<-0.05).mean():.0%}  person standing (<0.1): {(pv<0.1).mean():.0%}")
print(f"person in front |b|<45: {(np.abs(b)<45).mean():.0%}  side: {((np.abs(b)>=45)&(np.abs(b)<=135)).mean():.0%}  behind: {(np.abs(b)>135).mean():.0%}")
print(f"distracted: {np.mean([s['distracted'] for s in rows]):.0%}")
