"""Explore v5 sim checks before training: actor layout = car's OBS_LAYOUT_HIST, history buffer shifts at the scan
rate, the shield stops a car driving at a wall, person_ttc fires on a collision course."""
import importlib.util
from pathlib import Path
import torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.tasks.registry import load_env_cfg
import robotlab.tasks  # noqa
from robotlab.tasks import mentorpi_explore_env_cfg as X
spec = importlib.util.spec_from_file_location("eo", Path("car/policy/explore_obs.py")); O = importlib.util.module_from_spec(spec); spec.loader.exec_module(O)
cfg = load_env_cfg("Robotlab-Explore-MentorPiA1"); cfg.scene.num_envs = 256
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0"); env.reset()
om = env.observation_manager; lay, off = [], 0
for n, d in zip(om.active_terms["actor"], om.group_obs_term_dim["actor"]):
  lay.append(f"{n}:{d[0]}@{off}"); off += d[0]
print("layout matches car:", ";".join(lay) == O.OBS_LAYOUT_HIST, ";".join(lay)[-40:])
a = torch.zeros(env.num_envs, 2, device=env.device); a[:, 0] = 1.0     # full speed straight on
hits = shield_any = 0; ttc = 0.0; snaps = []
for i in range(120):
  obs, *_ = env.step(a)
  o = obs["actor"]
  snaps.append((o[:, 4:40].clone(), o[:, 1140:1176].clone(), getattr(env, "_robotlab_lidar_cast", None)))
  shield_any += env.action_manager.get_term("drive").shielded.float().mean().item()
  hits += env.termination_manager.get_term("bump").float().sum().item()
  ttc += X.person_ttc(env).mean().item()
# after a cast step k (new scan), hist[0] at the next cast step equals sectors at step k (noise aside: compare critic-free tolerance)
import itertools
diffs = []
for j in range(10, 100):
  cur_k, _, cast_k = snaps[j]
  if cast_k:
    for m in range(j + 1, j + 4):
      if snaps[m][2]:
        diffs.append((snaps[m][1] - cur_k).abs().mean().item()); break
print(f"history[0] vs the previous scan's sectors: mean |diff| {sum(diffs)/len(diffs):.4f} (noise only, ~0.01-0.03)")
print(f"shield engaged on {shield_any/120:.1%} of car-steps; wall contacts while driving straight at 0.7 m/s: {hits:.0f} in 256 cars x 6 s")
print(f"person_ttc mean {ttc/120:.4f} (> 0 when people around)")
