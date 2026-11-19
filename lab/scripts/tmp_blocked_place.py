"""Where do goto v6 blocked starts put the car? (2026-11-05) Pose vs hallway centre line and LiDAR ahead after a step."""
import math, torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.tasks.registry import load_env_cfg
import robotlab.tasks  # noqa
from robotlab.tasks import mentorpi_navigate_env_cfg as N
from robotlab.tasks.mdp import car_pose
cfg = load_env_cfg("Robotlab-Goto-MentorPiA1", play=True); cfg.scene.num_envs = 512
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
env.reset()
B = torch.cat([env._robotlab_blocked.nonzero()[:, 0][:12], env._robotlab_corner.nonzero()[:, 0][:6]])
nv = N.nav(env)
xy0, yaw0 = [t.clone() for t in car_pose(env)]
act = torch.zeros(env.num_envs, 2, device=env.device)
o, *_ = env.step(act)
xy, yaw = car_pose(env)
lid = o["actor"][:, 4:40] * 3
for i in B.tolist():
  k = nv.layout[i]
  print(f"env {i}: width {nv.maps.hall_width[k].item():.2f} | pose before step ({xy0[i,0]:+.2f},{xy0[i,1]:+.2f},{math.degrees(yaw0[i]):+.0f}) "
        f"after ({xy[i,0]:+.2f},{xy[i,1]:+.2f},{math.degrees(yaw[i]):+.0f}) | path0 ({nv.path[i,0,0]:+.2f},{nv.path[i,0,1]:+.2f}) "
        f"T+back ({nv.path[i,1,0]:+.2f},{nv.path[i,1,1]:+.2f}) | lidar min {lid[i].min():.2f} at {int(lid[i].argmin()) * 10 + 5} deg, ahead {min(lid[i,0], lid[i,35]):.2f} sides {lid[i,8:10].min():.2f}/{lid[i,26:28].min():.2f}")
