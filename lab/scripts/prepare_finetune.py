"""Prepare PPO fine-tuning of an imitation student (2026-11-08).

DAgger -> RL fine-tuning, as in the UZH vision-based agile flight work (student RL fine-tuning raised success 40% -> 100%),
with the warnings of Weihs et al. (NeurIPS 2021) that RL after imitation can unlearn it. So:
  - actor: the student's network (mean in [-1, 1], copied from the teacher);
  - exploration std reset to --std (the PPO checkpoints carried a steering std of 1.67 for an action range of +-1: noise
    larger than the range, the setting in which PPO learned bang-bang steering);
  - critic: from a PPO goto checkpoint with the same observation (a trained value function, not a random one);
  - optimizer state dropped, iteration 0.
Output: <out-dir>/model_0.pt, loadable by `train ... --agent.resume True --agent.load-run <dir> --agent.load-checkpoint
model_0.pt`; fine-tune with ROBOTLAB_IMITATE_W > 0 (mentorpi_goto_env_cfg.teacher_imitation).
"""
import argparse
import os

import torch

p = argparse.ArgumentParser()
p.add_argument("--student", required=True)
p.add_argument("--critic-from", required=True, help="PPO goto checkpoint with the same observation layout")
p.add_argument("--out-dir", required=True)
p.add_argument("--std", type=float, nargs=2, default=(0.10, 0.15), help="speed, steering (design choice)")
a = p.parse_args()

st = torch.load(a.student, map_location="cpu", weights_only=False)
cr = torch.load(a.critic_from, map_location="cpu", weights_only=False)
actor = st["actor_state_dict"]
old = actor["distribution.std_param"].tolist()
actor["distribution.std_param"] = torch.tensor(a.std, dtype=actor["distribution.std_param"].dtype)
opt = cr["optimizer_state_dict"]       # same parameter layout (actor + critic); Adam moments emptied: a fresh start
opt["state"] = {}
out = {"actor_state_dict": actor, "critic_state_dict": cr["critic_state_dict"], "optimizer_state_dict": opt, "iter": 0,
       "infos": None}
os.makedirs(a.out_dir, exist_ok=True)
path = os.path.join(a.out_dir, "model_0.pt")
torch.save(out, path)
print(f"{path}: actor from {a.student} (std {old} -> {list(a.std)}), critic from {a.critic_from}")
