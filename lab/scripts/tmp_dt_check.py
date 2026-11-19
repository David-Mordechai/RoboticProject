"""Physics time step 5 ms vs 10 ms (decimation 10 vs 5, same 20 Hz policy): does the car behave the same?
  python tmp_dt_check.py <dt>        explore env: speed tracking, stopping, wall contact detection, step time
  python tmp_dt_check.py <dt> pytest the turnaround spec tests (turning radius vs H/tan, U-turn widths, K-turn)"""
import sys, time, math
import mjlab.tasks.registry as reg
DT = float(sys.argv[1]); DEC = round(0.05 / DT)
_orig = reg.load_env_cfg
def load(task, play=False):
  c = _orig(task, play=play); c.sim.mujoco.timestep = DT; c.decimation = DEC; return c
reg.load_env_cfg = load
if len(sys.argv) > 2:
  import pytest
  sys.exit(pytest.main(["-q", "-s", "-p", "no:cacheprovider", "lab/tests/test_turnaround.py", "-k", "radius or sweep or multipoint"]))
import torch
from mjlab.envs import ManagerBasedRlEnv
import robotlab.tasks  # noqa
from robotlab.tasks.mdp import car_pose
cfg = reg.load_env_cfg("Robotlab-Explore-MentorPiA1", play=True); cfg.scene.num_envs = 512
cfg.actions["drive"].shield = None; cfg.actions["drive"].steer_cmd_rate = 0.0
for k in ("tyre_friction", "mass_inertia", "drive_motor"): cfg.events.pop(k, None)
d = cfg.actions["drive"]; d.latency_range_s = d.steer_deadtime_range_s = (0.0, 0.0); d.steer_rate_scale_range = (1.0, 1.0)
cfg.terminations = {k: v for k, v in cfg.terminations.items() if k == "time_out"}
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0"); env.reset()
n, dev = env.num_envs, env.device
car = env.scene["car"]
def run(v, steer, steps):
  a = torch.zeros(n, 2, device=dev); a[:, 0] = v; a[:, 1] = steer
  out = []
  for _ in range(steps):
    env.step(a); out.append(car.data.root_link_lin_vel_b[:, 0].clone())
  return torch.stack(out, 1)
xy0, yaw0 = [t.clone() for t in car_pose(env)]
vel = run(1.0, 0.0, 30)                      # 0.7 m/s straight, 1.5 s
xy1, yaw1 = [t.clone() for t in car_pose(env)]
vel2 = run(0.0, 0.0, 20)                     # stop
xy2 = car_pose(env)[0]
bump = env.scene["bump"].data.found[:, 0] > 0
print(f"dt {DT*1000:.0f} ms: speed after 0.5 s {vel[:, 9].mean():.3f} m/s, after 1.5 s {vel[:, -1].mean():.3f} (cmd 0.70); "
      f"distance in 1.5 s {torch.linalg.norm(xy1 - xy0, dim=-1).median():.3f} m; yaw drift {torch.rad2deg((yaw1 - yaw0).abs()).median():.2f} deg")
print(f"   stopping distance {torch.linalg.norm(xy2 - xy1, dim=-1).median():.3f} m, speed 0.25 s after stop {vel2[:, 4].abs().mean():.3f} m/s")
env.reset()
xy0, yaw0 = [t.clone() for t in car_pose(env)]
run(0.43, 1.0, 120)                          # 0.3 m/s full left lock, 6 s (hits walls in many envs)
hits = 0
for _ in range(40):
  env.step(torch.tensor([[1.0, 0.0]], device=dev).expand(n, 2)); hits += (env.scene["bump"].data.found[:, 0] > 0).float()
z = car.data.root_link_pos_w[:, 2]
print(f"   driving into walls: cars with a detected contact {(hits > 0).float().mean():.0%}; chassis height min {z.min():.4f} median {z.median():.4f} m (no sinking/jumping)")
env.reset(); torch.cuda.synchronize(); t = time.perf_counter()
cfg2 = None
for _ in range(50): env.step(torch.zeros(n, 2, device=dev))
torch.cuda.synchronize(); print(f"   step time at {n} envs: {1000*(time.perf_counter()-t)/50:.1f} ms")
