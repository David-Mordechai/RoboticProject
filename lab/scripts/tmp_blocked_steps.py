"""Step trace of goto v6 blocked mid-turn starts that do not get out (2026-11-05)."""
import sys
from dataclasses import asdict
import numpy as np, torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg
import robotlab.tasks  # noqa
from robotlab.tasks import mentorpi_navigate_env_cfg as N
from robotlab.tasks.mdp import car_pose
TASK = "Robotlab-Goto-MentorPiA1"
cfg = load_env_cfg(TASK, play=True); cfg.scene.num_envs = 256; cfg.episode_length_s = 12.0
cfg.observations["actor"].enable_corruption = True
ag = load_rl_cfg(TASK)
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
venv = RslRlVecEnvWrapper(env, clip_actions=ag.clip_actions)
r = MjlabOnPolicyRunner(venv, asdict(ag), device="cuda:0")
r.load(sys.argv[1], load_cfg={"actor": True}, strict=True, map_location="cuda:0")
pol = r.get_inference_policy(device="cuda:0")
B = env._robotlab_blocked.nonzero()[:, 0]
obs = venv.get_observations(); rows = {int(i): [] for i in B}; done = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
drive = env.action_manager.get_term("drive")
with torch.inference_mode():
  for it in range(20 * 10):
    xy, yaw = car_pose(env); a = pol(obs)
    lid = obs["actor"][:, 4:40] * 3
    for i in B.tolist():
      rows[i].append((it * 0.05, env.scene["car"].data.root_link_lin_vel_b[i, 0].item(), a[i, 0].item(), a[i, 1].item(),
                      int(drive.shielded[i]), min(lid[i, 0].item(), lid[i, 35].item()), lid[i, 17:19].min().item(),
                      N.route_distance(env)[i].item()))
    obs, _, dones, _ = venv.step(a)
    done |= N.nav(env).reached
fails = [i for i in B.tolist() if not done[i]]
print(f"blocked {len(B)}, out within 10 s {len(B) - len(fails)}")
allv = np.array([[x[2] for x in rows[i][:60]] for i in fails])
print(f"failed: share of steps in the first 3 s with speed action < 0: {np.mean(allv < 0):.0%}, shielded {np.mean([[x[4] for x in rows[i][:60]] for i in fails]):.0%}")
for i in fails[:3]:
  print(f"--- env {i}")
  for row in rows[i][:80:5]:
    print("t%5.2f v %+.2f act(%+.2f,%+.2f) shield %d front %.2f rear %.2f route_d %.2f" % row)
