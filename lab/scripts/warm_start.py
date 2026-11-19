"""Warm start: widen a checkpoint's first layers for observation terms appended at the END of the actor/critic input,
so training continues from the old policy instead of from scratch. New input columns get zero weights (the network
computes exactly what it did before), the observation normalizer gets statistics copied from an existing term of the
same kind (so the new inputs are scaled sensibly from the first step), Adam's moments are zero-padded.

  python lab/scripts/warm_start.py <old.pt> <new.pt> --actor-add 144 --critic-add 144 --copy-stats 4:40
  (--copy-stats a:b: the new values are repeats of input columns a..b, e.g. lidar_hist = 4 x lidar_sectors @4)
"""
import argparse
import torch

p = argparse.ArgumentParser()
p.add_argument("old"); p.add_argument("new")
p.add_argument("--actor-add", type=int, required=True); p.add_argument("--critic-add", type=int, required=True)
p.add_argument("--copy-stats", required=True, help="a:b, or 'unit' (mean 0, std 1: inputs already in 0..1)")
a = p.parse_args()
lo, hi = map(int, a.copy_stats.split(":")) if a.copy_stats != "unit" else (0, 1)
c = torch.load(a.old, map_location="cpu", weights_only=False)
widened = {}
for part, add in (("actor_state_dict", a.actor_add), ("critic_state_dict", a.critic_add)):
  sd = c[part]
  w = sd["mlp.0.weight"]
  widened[tuple(w.shape)] = add
  sd["mlp.0.weight"] = torch.cat([w, torch.zeros(w.shape[0], add)], 1)
  reps = add // (hi - lo)
  assert reps * (hi - lo) == add
  for k in ("obs_normalizer._mean", "obs_normalizer._var", "obs_normalizer._std"):
    new = sd[k][:, lo:hi].repeat(1, reps)
    if a.copy_stats == "unit":
      new = torch.zeros_like(new) if k.endswith("_mean") else torch.ones_like(new)
    sd[k] = torch.cat([sd[k], new], 1)
  print(f"{part}: input {w.shape[1]} -> {sd['mlp.0.weight'].shape[1]}")
for st in c["optimizer_state_dict"]["state"].values():
  for k in ("exp_avg", "exp_avg_sq"):
    if k in st and tuple(st[k].shape) in widened:
      st[k] = torch.cat([st[k], torch.zeros(st[k].shape[0], widened[tuple(st[k].shape)])], 1)
torch.save(c, a.new)
print(f"wrote {a.new} (iteration {c['iter']})")
