"""goto warm start fix (2026-11-04): set the observation normalizer of the route columns (actor and critic) from route
observations sampled in the sim, and reset the action std. warm_start.py copied the stats of other columns, and the
normalizer barely moves after explore's millions of samples (goto_v1 3650: route weights 0.7 vs 75 for the rest).
  python lab/scripts/tmp_route_stats.py <in.pt> <out.pt>"""
import sys
from dataclasses import asdict
import torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg
import robotlab.tasks  # noqa
TASK = "Robotlab-Goto-MentorPiA1"
cfg = load_env_cfg(TASK); cfg.scene.num_envs = 1024
ag = load_rl_cfg(TASK)
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
venv = RslRlVecEnvWrapper(env, clip_actions=ag.clip_actions)
r = MjlabOnPolicyRunner(venv, asdict(ag), device="cuda:0")
r.load(sys.argv[1], load_cfg={"actor": True}, strict=True, map_location="cuda:0")
pol = r.get_inference_policy(device="cuda:0")
obs = venv.get_observations(); A, C = [], []
with torch.inference_mode():
  for _ in range(300):
    A.append(obs["actor"][:, -17:].clone()); C.append(obs["critic"][:, -17:].clone())
    obs, _, _, _ = venv.step(pol(obs))
c = torch.load(sys.argv[1], map_location="cpu", weights_only=False)
for part, X in (("actor_state_dict", torch.cat(A)), ("critic_state_dict", torch.cat(C))):
  sd = c[part]; m, v = X.mean(0).cpu(), X.var(0).cpu().clamp(min=1e-4)
  sd["obs_normalizer._mean"][0, -17:] = m; sd["obs_normalizer._var"][0, -17:] = v; sd["obs_normalizer._std"][0, -17:] = v.sqrt()
  print(part, "route mean", m.numpy().round(2), "std", v.sqrt().numpy().round(2))
c["actor_state_dict"]["distribution.std_param"] = torch.tensor([0.25, 0.2])
st = c["optimizer_state_dict"]["state"].get(0)
if st is not None and st["exp_avg"].shape == (2,):
  st["exp_avg"].zero_(); st["exp_avg_sq"].zero_()
torch.save(c, sys.argv[2]); print("wrote", sys.argv[2])
