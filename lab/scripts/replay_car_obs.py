"""Replay observations recorded on the car (explore_policy.py --log) through an exported explore policy.

The gate before a car test: where an older model froze or failed on the floor, a new model must command something
better. Prints, per window of the drive, what the car's model commanded (recorded) next to what the given model
commands for the same inputs. Explore steps by default; --turn replays the turnaround steps (act column 10 = 1, a shorter zero-padded observation).

  python lab/scripts/replay_car_obs.py --onnx M.onnx --npz car_runs/v5_drive1.npz [--t0 34 --t1 58] [--every 0.5]

Action columns of the recording (explore_policy.py): a0 a1 (raw model output), v, steer (after limits), stop, x, y,
yaw, slam_yaw, t, turn.
"""
import argparse

import numpy as np
import onnxruntime as ort

p = argparse.ArgumentParser()
p.add_argument("--onnx", required=True)
p.add_argument("--npz", required=True)
p.add_argument("--t0", type=float, default=0.0)
p.add_argument("--t1", type=float, default=1e9)
p.add_argument("--every", type=float, default=0.5, help="s between printed rows")
p.add_argument("--turn", action="store_true", help="replay the turnaround steps (a turnaround model) instead")
p.add_argument("--executed-last", action="store_true", help="turnaround: last_action speed = 0 after a step the stop held")
a = p.parse_args()

d = np.load(a.npz)
obs, act = d["obs"], d["act"]
sess = ort.InferenceSession(a.onnx)
inp = sess.get_inputs()[0]
n_in = inp.shape[-1]
sel = (act[:, 10] == float(a.turn)) & (act[:, 9] >= a.t0) & (act[:, 9] <= a.t1)
if not sel.any():
  raise SystemExit("no matching steps in that window")
o = obs[sel][:, :n_in].astype(np.float32)
if a.executed_last:              # what the car executed: speed 0 where the obstacle stop held the previous step
  held_prev = np.r_[False, act[:-1, 4] > 0][sel]
  o[held_prev, n_in - 2] = 0.0
if obs.shape[1] < n_in:
  raise SystemExit(f"recorded observation has {obs.shape[1]} values, the model wants {n_in}")
out = np.concatenate([sess.run(None, {inp.name: o[i:i + 1]})[0] for i in range(len(o))])   # exported with batch 1
t, rec = act[sel, 9], act[sel, :2]
print(f"{a.onnx}: {sel.sum()} explore steps in [{a.t0}, {min(a.t1, t.max()):.1f}] s")
print("    t   recorded a0  a1  |  replay a0  a1   (a0 > 0 forward, < 0 reverse; a1 steering, + left)")
last = -1e9
for i in range(len(t)):
  if t[i] - last >= a.every:
    print(f"{t[i]:5.1f}   {rec[i, 0]:+6.2f} {rec[i, 1]:+5.2f}  |  {out[i, 0]:+6.2f} {out[i, 1]:+5.2f}")
    last = t[i]
moving = lambda x: (np.abs(np.clip(x, -1, 1)) > 0.1).mean()
print(f"share of steps commanding |speed action| > 0.1: recorded {moving(rec[:, 0]):.0%}, replay {moving(out[:, 0]):.0%}")
