"""The 2 s before each person contact: car speed, command, shield / approaching flags, person distance and bearing."""
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
cfg = load_env_cfg(TASK, play=True); cfg.scene.num_envs = 512; cfg.episode_length_s = 60.0
cfg.observations["actor"].enable_corruption = True
ag = load_rl_cfg(TASK)
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
venv = RslRlVecEnvWrapper(env, clip_actions=ag.clip_actions)
r = MjlabOnPolicyRunner(venv, asdict(ag), device="cuda:0")
r.load(sys.argv[1], load_cfg={"actor": True}, strict=True, map_location="cuda:0")
pol = r.get_inference_policy(device="cuda:0")
drive = env.action_manager.get_term("drive"); n = env.num_envs
H = 40; hist = []; shown = 0
obs = venv.get_observations()
with torch.inference_mode():
  for step in range(20 * 60 - 1):
    e, (xy, yaw) = T.explore(env), car_pose(env)
    w = walkers(env, e.maps, e.layout)
    d = torch.where(w.active, torch.linalg.norm(w.xy - xy[:, None], dim=-1), 1e3); k = d.argmin(-1)
    rel = to_frame(w.xy[torch.arange(n, device=env.device), k] - xy, yaw)
    v = env.scene["car"].data.root_link_lin_vel_b[:, 0]
    a = pol(obs)
    appr = T.approaching(env, drive.v_cmd)
    hist.append(torch.stack([v, a[:, 0].clamp(-1, 1), drive.shielded.float(), appr.float(), d[torch.arange(n, device=env.device), k],
                             torch.rad2deg(torch.atan2(rel[:, 1], rel[:, 0]))], -1).cpu().numpy())
    hist = hist[-H:]
    obs, _, dones, _ = venv.step(a)
    m = env.termination_manager.get_term("hit_person")
    for j in torch.nonzero(m)[:, 0].tolist():
      if shown >= 8: break
      shown += 1
      print(f"--- contact {shown}: t-2.0s .. t (every 0.2 s): v / action / shield / approaching / person dist / bearing")
      for h in hist[::4] + [hist[-1]]:
        x = h[j]; print(f"   v {x[0]:+.2f}  a0 {x[1]:+.2f}  shield {int(x[2])}  appr {int(x[3])}  dist {x[4]:.2f}  bearing {x[5]:+5.0f}")
    if shown >= 8: break
