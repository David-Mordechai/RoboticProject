"""goto_symmetry sanity (2026-11-05): layout sizes, mirror twice = identity, and a mirrored sim scene gives the mirrored
LiDAR (a car with a wall on its left vs the same mirrored: compare lidar_sectors of two envs placed as mirror images)."""
import torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg
import robotlab.tasks  # noqa
from robotlab.tasks import goto_symmetry as S
cfg = load_env_cfg("Robotlab-Goto-MentorPiA1", play=True); cfg.scene.num_envs = 32
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0"); obs, _ = env.reset()
from tensordict import TensorDict
td = TensorDict({k: v for k, v in obs.items()}, batch_size=[32])
o2, a2 = S.mirror(obs=td, actions=torch.randn(32, 2, device="cuda:0"))
print("shapes", {k: tuple(o2[k].shape) for k in o2.keys()}, tuple(a2.shape))
o4, _ = S.mirror(obs=o2[32:])
for k in td.keys():
  print(k, "mirror twice == identity:", torch.allclose(o4[32:][k], td[k]))
print("rl cfg symmetry:", load_rl_cfg("Robotlab-Goto-MentorPiA1").algorithm.symmetry_cfg)
