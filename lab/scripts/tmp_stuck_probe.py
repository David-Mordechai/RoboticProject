"""Which input pins the policy at a floor stuck point? (2026-11-05) Run the ONNX on a recorded stuck observation with
one input group changed at a time and print the action (speed, steer)."""
import sys, numpy as np, onnxruntime as ort
onnx, npz, t_stuck = sys.argv[1], sys.argv[2], float(sys.argv[3])
d = np.load(npz); o, a = d["obs"], d["act"]
j = int(np.argmin(np.abs(a[:, 9] - t_stuck)))
s = ort.InferenceSession(onnx); inp = s.get_inputs()[0].name
run = lambda x: s.run(None, {inp: x[None].astype(np.float32)})[0][0][:2]
x = o[j].copy()
print(f"t {a[j, 9]:.1f}: recorded action {a[j, :2].round(2)}, model now {run(x).round(2)}")
groups = {"car_motion": (0, 4), "lidar": (4, 40), "depth": (40, 52), "map_fine": (52, 852), "map_coarse": (852, 1140),
          "lidar_hist": (1140, 1284), "route": (1284, 1301), "stop_held": (1301, 1302)}
for name, (lo, hi) in groups.items():
  for how, val in (("zero", 0.0), ("as 0.5 s earlier", None)):
    y = x.copy()
    y[lo:hi] = val if val is not None else o[max(j - 10, 0), lo:hi]
    print(f"  {name:11s} {how:17s} -> {run(y).round(2)}")
y = x.copy(); y[52:1140] = o[max(j - 60, 0), 52:1140]
print(f"  maps from 3 s earlier            -> {run(y).round(2)}")
k = int(np.argmin(np.abs(a[:, 9] - 3.0)))
y = x.copy(); y[52:1140] = 0.5; print(f"  maps all 0.5                     -> {run(y).round(2)}")
