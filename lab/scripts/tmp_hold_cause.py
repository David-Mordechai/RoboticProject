"""How often the approaching stop / hold blocks the car, and whether a person is near when it does."""
import sys
from dataclasses import asdict
import numpy as np, torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg
import robotlab.tasks  # noqa
from robotlab.tasks import mentorpi_explore_env_cfg as T
TASK = "Robotlab-Explore-MentorPiA1"
cfg = load_env_cfg(TASK, play=True); cfg.scene.num_envs = 512; cfg.episode_length_s = 60.0
cfg.observations["actor"].enable_corruption = True
ag = load_rl_cfg(TASK)
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
venv = RslRlVecEnvWrapper(env, clip_actions=ag.clip_actions)
r = MjlabOnPolicyRunner(venv, asdict(ag), device="cuda:0")
r.load(sys.argv[1], load_cfg={"actor": True}, strict=True, map_location="cuda:0")
pol = r.get_inference_policy(device="cuda:0")
drive = env.action_manager.get_term("drive")
orig = T.approaching; flags = {}
def spy(env_, v):
  out = orig(env_, v); flags["a"] = out.clone(); return out
T.approaching = spy
A, C, S = [], [], []
obs = venv.get_observations()
with torch.inference_mode():
  for _ in range(20 * 60 - 1):
    flags.clear()
    obs, *_ = venv.step(pol(obs))
    a = flags.get("a", torch.zeros(env.num_envs, dtype=torch.bool, device=env.device))
    A.append(a.cpu().numpy()); S.append(drive.shielded.cpu().numpy()); C.append(T.person_clearance(env).cpu().numpy())
A, S, C = map(np.array, (A, S, C))
print(f"share of car-steps blocked by the approach stop/hold: {A.mean():.1%}; by any shield: {S.mean():.1%}")
for lo, hi in [(0, 1.0), (1.0, 2.0), (2.0, 1e9)]:
  m = (C >= lo) & (C < hi)
  print(f"  nearest person {lo:.0f}-{hi if hi < 1e8 else 'inf'} m: {m.mean():5.1%} of time, approach stop/hold active {A[m].mean():5.1%} of that time")
print(f"approach stop/hold with no person within 2 m: {(A & (C >= 2.0)).sum() / max(A.sum(), 1):.0%} of all its activations")
