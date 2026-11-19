"""Student training by imitation of the privileged teacher, with DAgger (2026-11-08).

Method: DAgger (Ross, Gordon & Bagnell, AISTATS 2011): the cars drive with a mix of teacher and student actions (the
teacher's share beta decays each round, so the student meets the states its own mistakes lead to); in every visited
state the teacher's action is the label (robotlab/tasks/teacher.py: true map, true pose, people); the student is trained
by supervised regression on the aggregated data. Privileged teacher -> sensor-only student as in "Learning by Cheating"
(Chen et al., CoRL 2019).

The student is the goto actor network with the car's own observation (the 1302 values the car node builds), warm
started from a goto checkpoint (its sensor layers), and saved in rsl_rl's checkpoint format, so tmp_goto_check.py,
export_onnx.py and car/policy/explore_policy.py work unchanged. It predicts the teacher's drive action [speed, steering]
(each in [-1, 1]); regression to in-range targets keeps the output out of the clipped region where the PPO policy
learned bang-bang steering.

  python lab/scripts/train_student.py --init <goto checkpoint.pt> --run-name student_v1
"""
import argparse
import os
import time
from dataclasses import asdict
from datetime import datetime

import torch

from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg

import robotlab.tasks  # noqa: F401
from robotlab.tasks import teacher as TCH

p = argparse.ArgumentParser()
p.add_argument("--task", default="Robotlab-Goto-MentorPiA1")
p.add_argument("--init", required=True, help="goto checkpoint whose actor starts the student")
p.add_argument("--run-name", default="student")
p.add_argument("--envs", type=int, default=4096)
p.add_argument("--rounds", type=int, default=40)
p.add_argument("--steps", type=int, default=64, help="policy steps collected per round and car")
p.add_argument("--beta0", type=float, default=1.0, help="teacher share of executed actions in round 0")
p.add_argument("--beta-decay", type=float, default=0.8, help="beta *= this each round (design choice)")
p.add_argument("--buffer", type=int, default=3_000_000, help="aggregated samples kept (oldest dropped)")
p.add_argument("--epochs", type=float, default=2.0, help="passes over the buffer per round")
p.add_argument("--batch", type=int, default=16384)
p.add_argument("--lr", type=float, default=3e-4)
p.add_argument("--save-every", type=int, default=5)
p.add_argument("--forward-labels-only", action="store_true",
               help="no labels where the teacher reverses or runs its multi-point turn (the car skill owns those; "
                    "2026-11-09: students learned 'reverse', forward_only made it 'stand still', 24-37%% of idle time)")
p.add_argument("--loss", choices=("mse", "l1"), default="l1",
               help="l1 (student v2): the median of the teacher's actions per state, so a forward / reverse split picks "
                    "the more common move; mse (v1) averaged them to ~0 speed (idle 10%, wall starts held 3.6 s)")
a = p.parse_args()

dev = "cuda:0"
cfg = load_env_cfg(a.task, play=False)
cfg.scene.num_envs = a.envs
ag = load_rl_cfg(a.task)
env = ManagerBasedRlEnv(cfg=cfg, device=dev)
venv = RslRlVecEnvWrapper(env, clip_actions=ag.clip_actions)
log_dir = os.path.join("logs/rsl_rl", ag.experiment_name, datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + "_" + a.run_name)
os.makedirs(log_dir, exist_ok=True)
runner = MjlabOnPolicyRunner(venv, asdict(ag), log_dir=log_dir, device=dev)
actor = runner.alg.get_policy()
# Warm start from the init checkpoint's actor. If the task has more inputs than the checkpoint (GotoStudent: + last_drive
# after the goto inputs), pad: first-layer weights 0 for the new inputs, normalizer mean 0 / var and std 1.
ck = torch.load(a.init, map_location=dev, weights_only=False)
sd = ck["actor_state_dict"]
new_dim = actor.state_dict()["mlp.0.weight"].shape[1]
old_dim = sd["mlp.0.weight"].shape[1]
if new_dim != old_dim:
  for k, v in list(sd.items()):
    if v.dim() >= 1 and v.shape[-1] == old_dim:
      fill = 1.0 if ("var" in k or "std" in k) else 0.0
      sd[k] = torch.cat([v, torch.full((*v.shape[:-1], new_dim - old_dim), fill, device=v.device, dtype=v.dtype)], -1)
actor.load_state_dict(sd, strict=True)
opt = torch.optim.Adam(actor.mlp.parameters(), lr=a.lr)

N = env.num_envs
obs = venv.get_observations()
D = obs["actor"].shape[1]
buf_x = torch.zeros(a.buffer, D, dtype=torch.float16, device=dev)
buf_y = torch.zeros(a.buffer, 2, device=dev)
filled, head = 0, 0
beta = a.beta0
log = open(os.path.join(log_dir, "student.log"), "a")


def say(msg):
  line = f"{datetime.now():%H:%M:%S} {msg}"
  print(line, flush=True)
  log.write(line + "\n"); log.flush()


say(f"student {a.run_name}: task {a.task}, init {a.init} ({old_dim} -> {new_dim} inputs), loss {a.loss}, {N} cars, "
    f"log {log_dir}")
for rnd in range(a.rounds):
  t0 = time.time()
  # --- collect: execute teacher with prob beta (per car, per round), else the student; label every state with the teacher
  use_teacher = torch.rand(N, device=dev) < beta
  agree, n_lab = 0.0, 0
  with torch.inference_mode():
    for _ in range(a.steps):
      a_t, _ = TCH.act(env)
      x = obs["actor"]
      a_s = actor(obs).clamp(-1, 1)
      keep = torch.ones(N, dtype=torch.bool, device=dev)
      if a.forward_labels_only:   # (2026-11-09) the car's skill owns reversing / multi-point turns: no labels there
        keep = ~env._teacher_kturn["on"] & (a_t[:, 0] >= 0)
      xs, ys = x[keep], a_t[keep]
      M = xs.shape[0]
      k = min(M, a.buffer - head)
      buf_x[head:head + k] = xs[:k].half(); buf_y[head:head + k] = ys[:k]
      if k < M:
        buf_x[:M - k] = xs[k:].half(); buf_y[:M - k] = ys[k:]
      head = (head + M) % a.buffer
      filled = min(filled + M, a.buffer)
      agree += float((a_s - a_t).abs().mean()); n_lab += 1
      obs, _, _, _ = venv.step(torch.where(use_teacher[:, None], a_t, a_s))
  t_col = time.time() - t0
  # --- train: regression of the student's mean action on the teacher's labels
  actor.train()
  steps = max(1, int(a.epochs * filled / a.batch))
  tot = 0.0
  for _ in range(steps):
    idx = torch.randint(0, filled, (a.batch,), device=dev)
    x = buf_x[idx].float()
    pred = actor.mlp(actor.obs_normalizer(x))
    loss = (torch.nn.functional.l1_loss if a.loss == "l1" else torch.nn.functional.mse_loss)(pred, buf_y[idx])
    opt.zero_grad(); loss.backward()
    torch.nn.utils.clip_grad_norm_(actor.mlp.parameters(), 1.0)
    opt.step()
    tot += float(loss)
  say(f"round {rnd}: beta {beta:.2f}, buffer {filled}, |student - teacher| {agree / n_lab:.3f}, "
      f"train {a.loss} {tot / steps:.4f} ({steps} steps), collect {t_col:.0f} s, train {time.time() - t0 - t_col:.0f} s")
  beta *= a.beta_decay
  runner.current_learning_iteration = rnd
  if (rnd + 1) % a.save_every == 0 or rnd == a.rounds - 1:
    path = os.path.join(log_dir, f"model_{rnd + 1}.pt")
    runner.save(path)
    say(f"saved {path}")
