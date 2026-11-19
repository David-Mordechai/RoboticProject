"""Reset a checkpoint's action std (rsl_rl scalar std) and its Adam moments, to restore PPO exploration before
continuing training. v7d (2026-11-04): explore's steering std had collapsed to 0.0016 (speed 0.098).
  python lab/scripts/reset_std.py <old.pt> <new.pt> 0.25 0.2"""
import sys
import torch
old, new, stds = sys.argv[1], sys.argv[2], [float(x) for x in sys.argv[3:]]
c = torch.load(old, map_location="cpu", weights_only=False)
sd = c["actor_state_dict"]
k = "distribution.std_param"
print("std before:", sd[k].tolist())
sd[k] = torch.tensor(stds, dtype=sd[k].dtype)
# Adam state of that parameter: the actor's params come first in the optimizer, std_param is the actor's last entry
names = list(sd.keys()); idx = [n for n in names if not n.startswith("obs_normalizer")].index(k)
st = c["optimizer_state_dict"]["state"].get(idx)
if st is not None and st["exp_avg"].shape == sd[k].shape:
  st["exp_avg"].zero_(); st["exp_avg_sq"].zero_()
  print("Adam moments of std reset (param index", idx, ")")
else:
  print("WARNING: Adam state for std not found at index", idx)
print("std after:", sd[k].tolist())
torch.save(c, new)
