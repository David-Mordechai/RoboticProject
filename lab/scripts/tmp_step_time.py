"""Explore env step time at 8192 envs, people pass_by on/off, plus contact counts."""
import sys, time, torch
import robotlab.tasks.mentorpi_explore_env_cfg as X
import robotlab.tasks.people as P
from mjlab.envs import ManagerBasedRlEnv
from mjlab.tasks.registry import load_env_cfg
flag = sys.argv[1] == "1"
orig = P.step_walkers
X.step_walkers = lambda env, w, xy, pass_by=False: orig(env, w, xy, pass_by=flag)
cfg = load_env_cfg("Robotlab-Explore-MentorPiA1"); cfg.scene.num_envs = 8192
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0"); env.reset()
a = torch.zeros(env.num_envs, 2, device=env.device); a[:, 0] = 0.5
for _ in range(60): env.step(a)
torch.cuda.synchronize(); t = time.perf_counter()
for _ in range(100): env.step(a)
torch.cuda.synchronize()
print(f"pass_by={flag}: {1000*(time.perf_counter()-t)/100:.1f} ms/step, contacts {int(env.sim.data.nacon.sum().item())}")
