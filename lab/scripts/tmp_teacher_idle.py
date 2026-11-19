"""Why the teacher stands still (2026-11-08): over 512 cars x 40 s, every step with |v| < 0.05 m/s and no person in the
path (the checker's idle goal) is put in one class: held by the shield, scripted back-up, multi-point turn, teacher
speed 0 because the true map blocks its arc, because a person is near its arc, or other."""
import torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.tasks.registry import load_env_cfg
import robotlab.tasks  # noqa
from robotlab.tasks import mentorpi_explore_env_cfg as T
from robotlab.tasks import teacher as TCH
from robotlab.tasks.mdp import car_pose
from robotlab.tasks.people import walkers

cfg = load_env_cfg("Robotlab-Goto-MentorPiA1", play=True); cfg.scene.num_envs = 512
cfg.observations["actor"].enable_corruption = True
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
env.reset()
cnt = {}
tot = 0
with torch.inference_mode():
  for it in range(20 * 40):
    a, _ = TCH.act(env)
    drv = env.action_manager.get_term("drive")
    pp = T.person_in_path(env)
    xy, yaw = car_pose(env)
    # the teacher's own arc check, split by cause
    kap = torch.zeros(env.num_envs, device=env.device)
    free_map = TCH._free_along_arc.__wrapped__(env, xy, yaw, kap) if hasattr(TCH._free_along_arc, "__wrapped__") else None
    _, _, _, _, _ = env.step(a)
    v = env.scene["car"].data.root_link_lin_vel_b[:, 0]
    idle = (v.abs() < 0.05) & ~pp & (env.episode_length_buf > 20)
    kt = env._teacher_kturn["on"]
    e = T.explore(env); w = walkers(env, e.maps, e.layout)
    near_person = ((torch.linalg.norm(w.xy - xy[:, None], dim=-1) < 1.2) & w.active).any(-1)
    cls = torch.where(drv.shielded, 0, torch.where(drv.backing, 1, torch.where(kt, 2, torch.where(
      a[:, 0] <= 0.0, torch.where(near_person, 4, 3), 5))))
    names = ["shield holds", "scripted back-up", "multi-point turn", "teacher stop: map blocks arc",
             "teacher stop: person within 1.2 m", "teacher drives, car not moving yet"]
    for c in range(6):
      cnt[names[c]] = cnt.get(names[c], 0) + int((idle & (cls == c)).sum())
    tot += int((~pp & (env.episode_length_buf > 20)).sum())
idle_all = sum(cnt.values())
print(f"idle share {idle_all / max(tot, 1):.3f} of {tot} car-steps")
for k, v in sorted(cnt.items(), key=lambda x: -x[1]):
  print(f"  {k}: {v / max(idle_all, 1):.0%}")
