"""v7c sanity check: how often the CaT person_close termination fires (training cfg, 256 cars x 30 s) with a checkpoint."""
import sys
from dataclasses import asdict
import torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg
import robotlab.tasks  # noqa
TASK = "Robotlab-Explore-MentorPiA1"
cfg = load_env_cfg(TASK); cfg.scene.num_envs = 256
ag = load_rl_cfg(TASK)
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
venv = RslRlVecEnvWrapper(env, clip_actions=ag.clip_actions)
r = MjlabOnPolicyRunner(venv, asdict(ag), device="cuda:0")
r.load(sys.argv[1], load_cfg={"actor": True}, strict=True, map_location="cuda:0")
pol = r.get_inference_policy(device="cuda:0")
obs = venv.get_observations(); cnt = {k: 0 for k in env.termination_manager.active_terms}
with torch.inference_mode():
  for _ in range(20 * 30):
    obs, _, dones, _ = venv.step(pol(obs))
    for k in cnt:
      cnt[k] += int(env.termination_manager.get_term(k).sum())
print("terminations in 256 x 30 s:", cnt)
