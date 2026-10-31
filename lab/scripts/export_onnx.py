"""Export a checkpoint to ONNX for the car (open issue #2), then prove the file is the policy.

  docker compose run --rm lab python lab/scripts/export_onnx.py Robotlab-Office-MentorPiA1 \\
      --checkpoint logs/rsl_rl/office/<run>/model_599.pt

- Export: mjlab's runner.export_policy_to_onnx emits actor(normalizer(obs)) (obs_normalization=True), so the car
  runs exactly what training saw (the path Microduck's export.py uses). Written next to the checkpoint.
- Metadata: task, checkpoint, control rate, and the exact actor observation layout (term name, size, offset) and
  action layout, so the car-side node can be checked against the file instead of against a doc.
- Parity: actor observations recorded from the task (policy driving, sensor noise on) through torch and ONNX
  Runtime; max abs difference must be <= 1e-5.
- Timing: ONNX Runtime on this CPU, 1 thread, one observation at a time (as on the car). Indicative only: the Pi 5
  (Cortex-A76) must be measured on the car.
"""

import argparse
import time
from dataclasses import asdict
from pathlib import Path

import numpy as np
import onnx
import onnxruntime as ort
import torch

from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.rl.exporter_utils import attach_metadata_to_onnx
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg

import robotlab.tasks  # noqa: F401

p = argparse.ArgumentParser()
p.add_argument("task")
p.add_argument("--checkpoint", required=True)
p.add_argument("--out", default=None, help="default: <checkpoint dir>/<checkpoint name>.onnx")
p.add_argument("--num-envs", type=int, default=32)
p.add_argument("--steps", type=int, default=200)
args = p.parse_args()

ckpt = Path(args.checkpoint)
out = Path(args.out) if args.out else ckpt.with_suffix(".onnx")
cfg = load_env_cfg(args.task, play=True)
cfg.scene.num_envs = args.num_envs
cfg.observations["actor"].enable_corruption = True          # record what the car's noisy sensors look like
agent_cfg = load_rl_cfg(args.task)
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
venv = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
runner = MjlabOnPolicyRunner(venv, asdict(agent_cfg), device="cuda:0")
runner.load(str(ckpt), load_cfg={"actor": True}, strict=True, map_location="cuda:0")
policy = runner.get_inference_policy(device="cuda:0")

runner.export_policy_to_onnx(str(out.parent), out.name)
om = env.observation_manager
layout, off = [], 0
for name, dim in zip(om.active_terms["actor"], om.group_obs_term_dim["actor"]):
  layout.append(f"{name}:{dim[0]}@{off}")
  off += dim[0]
meta = {   # mjlab's get_base_metadata is for legged robots (entity "robot", joint defaults); ours is a car

  "task": args.task, "checkpoint": str(ckpt), "control_hz": round(1.0 / env.step_dt),
  "obs_layout": ";".join(layout), "obs_dim": off,
  "action_layout": "speed:[-1,1] (>=0 forward up to V_MAX, <0 reverse up to v_reverse);steer:[-1,1] x car steer limit",
}
attach_metadata_to_onnx(str(out), meta)
onnx.checker.check_model(onnx.load(str(out)))

# Record actor observations while the policy drives.
obs = venv.get_observations()
recorded, torch_act = [], []
with torch.inference_mode():
  for _ in range(args.steps):
    a = policy(obs)
    recorded.append(obs["actor"].cpu().numpy())
    torch_act.append(a.cpu().numpy())
    obs, _, _, _ = venv.step(a)
X, Y = np.concatenate(recorded).astype(np.float32), np.concatenate(torch_act)
env.close()

so = ort.SessionOptions()
so.intra_op_num_threads = so.inter_op_num_threads = 1
sess = ort.InferenceSession(str(out), so, providers=["CPUExecutionProvider"])
inp = sess.get_inputs()[0]
y = np.concatenate([sess.run(None, {inp.name: X[i:i + 1]})[0] for i in range(len(X))])
diff = float(np.abs(y - Y).max())

times = []
for i in range(2000):
  x = X[i % len(X)][None]
  t0 = time.perf_counter()
  sess.run(None, {inp.name: x})
  times.append(time.perf_counter() - t0)
t = np.array(times[200:]) * 1e3

print(f"wrote {out} ({out.stat().st_size / 1024:.0f} KiB), input '{inp.name}' {inp.shape}")
print(f"actor observation layout ({off} values): {' | '.join(layout)}")
print(f"parity on {len(X)} recorded observations: max |onnx - torch| = {diff:.2e} -> {'PASS' if diff <= 1e-5 else 'FAIL'} (<= 1e-5)")
print(f"ONNX Runtime, 1 thread, this CPU: median {np.median(t):.3f} ms, p99 {np.percentile(t, 99):.3f} ms per step "
      f"(budget at {round(1.0 / env.step_dt)} Hz: {1000 * env.step_dt:.0f} ms)")
