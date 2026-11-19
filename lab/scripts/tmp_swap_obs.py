"""Swap the last observation term of a checkpoint for a new one (2026-11-04: goto's 17-value route preview -> GotoSub's
3-value sub-goal). The old columns' weights are dropped, the new columns start at zero weight (the network computes
what it did without the route), their normalizer stats come from observations sampled in the new task, Adam's moments
are cut / zero-padded the same way, and the action std is reset.
  python lab/scripts/tmp_swap_obs.py <in.pt> <out.pt> <drop> <TASK>      e.g. ... 17 Robotlab-GotoSub-MentorPiA1"""
import sys
from dataclasses import asdict
import torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg
import robotlab.tasks  # noqa
src, dst, drop, task = sys.argv[1], sys.argv[2], int(sys.argv[3]), sys.argv[4]
c = torch.load(src, map_location="cpu", weights_only=False)
cfg = load_env_cfg(task); cfg.scene.num_envs = 1024
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
ag = load_rl_cfg(task)
venv = RslRlVecEnvWrapper(env, clip_actions=ag.clip_actions)
obs = venv.get_observations()
add = {}
reshaped = {}
for part, group in (("actor_state_dict", "actor"), ("critic_state_dict", "critic")):
  sd = c[part]
  w = sd["mlp.0.weight"]
  keep = w.shape[1] - drop
  add[group] = obs[group].shape[1] - keep
  reshaped[tuple(w.shape)] = (keep, add[group])
  sd["mlp.0.weight"] = torch.cat([w[:, :keep], torch.zeros(w.shape[0], add[group])], 1)
  for k in ("obs_normalizer._mean", "obs_normalizer._var", "obs_normalizer._std"):
    sd[k] = torch.cat([sd[k][:, :keep], torch.ones(1, add[group]) * (0.0 if k.endswith("mean") else 1.0)], 1)
  print(f"{part}: input {w.shape[1]} -> {keep} + {add[group]}")
for st in c["optimizer_state_dict"]["state"].values():
  for k in ("exp_avg", "exp_avg_sq"):
    if k in st and tuple(st[k].shape) in reshaped:
      keep, n = reshaped[tuple(st[k].shape)]
      st[k] = torch.cat([st[k][:, :keep], torch.zeros(st[k].shape[0], n)], 1)
c["actor_state_dict"]["distribution.std_param"] = torch.tensor([0.25, 0.2])
st = c["optimizer_state_dict"]["state"].get(0)
if st is not None and st["exp_avg"].shape == (2,):
  st["exp_avg"].zero_(); st["exp_avg_sq"].zero_()
torch.save(c, dst)
# Normalizer stats of the new columns: sample them in the task with the converted policy driving.
r = MjlabOnPolicyRunner(venv, asdict(ag), device="cuda:0")
r.load(dst, load_cfg={"actor": True}, strict=True, map_location="cuda:0")
pol = r.get_inference_policy(device="cuda:0")
S = {"actor": [], "critic": []}
with torch.inference_mode():
  for _ in range(300):
    for g in S:
      S[g].append(obs[g][:, -add[g]:].clone())
    obs, _, _, _ = venv.step(pol(obs))
for part, g in (("actor_state_dict", "actor"), ("critic_state_dict", "critic")):
  X = torch.cat(S[g]); m, v = X.mean(0).cpu(), X.var(0).cpu().clamp(min=1e-4)
  sd = c[part]
  sd["obs_normalizer._mean"][0, -add[g]:] = m; sd["obs_normalizer._var"][0, -add[g]:] = v
  sd["obs_normalizer._std"][0, -add[g]:] = v.sqrt()
  print(part, "new columns mean", m.numpy().round(2), "std", v.sqrt().numpy().round(2))
torch.save(c, dst); print("wrote", dst, "iteration", c.get("iter"))
