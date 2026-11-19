"""Turnaround v3: what happens in open-room episodes that time out? Final heading error, speed, clearance, shield share."""
import sys, math
from dataclasses import asdict
import torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg
import robotlab.tasks  # noqa
from robotlab.tasks import mentorpi_turnaround_env_cfg as T
TASK = "Robotlab-Turnaround-MentorPiA1"
cfg = load_env_cfg(TASK, play=True); cfg.scene.num_envs = 1024
cfg.observations["actor"].enable_corruption = True
T.OPEN_ROOM_PROB = 1.0
ag = load_rl_cfg(TASK)
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0"); venv = RslRlVecEnvWrapper(env, clip_actions=ag.clip_actions)
r = MjlabOnPolicyRunner(venv, asdict(ag), device="cuda:0"); r.load(sys.argv[1], load_cfg={"actor": True}, strict=True, map_location="cuda:0")
pol = r.get_inference_policy(device="cuda:0")
env.reset(); obs = venv.get_observations(); n = env.num_envs
drive = env.action_manager.get_term("drive"); sh = torch.zeros(n, device=env.device); steps = 0
with torch.inference_mode():
  for k in range(int(19.5 / env.step_dt)):
    obs, _, d, _ = venv.step(pol(obs)); sh += drive.shielded.float(); steps += 1
err = T.heading_error(env).abs(); v = env.scene["car"].data.root_link_lin_vel_b[:, 0].abs()
print(f"after 19.5 s (no episode ended counted): |heading err| median {math.degrees(err.median().item()):.0f} deg, "
      f"< 10 deg: {(err < math.radians(10)).float().mean().item():.0%}; |v| median {v.median().item():.2f} m/s, "
      f"< 0.05: {(v < 0.05).float().mean().item():.0%}; wall clearance median {T.wall_clearance(env).median().item():.2f} m; "
      f"shield-stopped share of steps {(sh / steps).mean().item():.0%}")
xy, yaw = __import__('robotlab.tasks.mdp', fromlist=['car_pose']).car_pose(env)
print(f"car x median {xy[:,0].median().item():.2f} (dead end at 0, start -0.9..-3.1), |y| median {xy[:,1].abs().median().item():.2f}")
