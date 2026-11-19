"""Trace one goto env: car pose vs route points and progress s (debugging s = 0 at loss, 2026-11-04)."""
import sys
from dataclasses import asdict
import torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg
import robotlab.tasks  # noqa
from robotlab.tasks import mentorpi_navigate_env_cfg as N
from robotlab.tasks import mentorpi_explore_env_cfg as X
from robotlab.tasks.mdp import car_pose
TASK = "Robotlab-Goto-MentorPiA1"
cfg = load_env_cfg(TASK, play=True); cfg.scene.num_envs = 8
ag = load_rl_cfg(TASK)
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
venv = RslRlVecEnvWrapper(env, clip_actions=ag.clip_actions)
r = MjlabOnPolicyRunner(venv, asdict(ag), device="cuda:0")
r.load(sys.argv[1], load_cfg={"actor": True}, strict=True, map_location="cuda:0")
pol = r.get_inference_policy(device="cuda:0")
obs = venv.get_observations()
nv, e = N.nav(env), X.explore(env)
print("layouts nav", nv.layout.tolist(), "explore", e.layout.tolist())
print("path[0,:6]", nv.path[0, :6].tolist())
with torch.inference_mode():
  for t in range(120):
    xy, yaw = car_pose(env)
    if t % 10 == 0:
      print(f"t {t*0.05:4.1f} car {xy[0].tolist()} yaw {yaw[0].item():+.2f} s {nv.s[0].item():.2f} seg {nv.seg[0].item()} "
            f"route_dist {N.route_distance(env)[0].item():.2f} preview0 {obs['actor'][0, -17:-15].tolist()}")
    obs, _, d, _ = venv.step(pol(obs))
    if d[0]: print("env 0 done at", t)
