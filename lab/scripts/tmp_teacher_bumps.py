"""Teacher bumps (2026-11-08): for every run that ends by wall / clutter contact, the commanded speed sign, whether the
scripted back-up or the teacher's own reverse / multi-point turn drove, and whether a chair base is within 0.45 m."""
import collections
import torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.tasks.registry import load_env_cfg
import robotlab.tasks  # noqa
from robotlab.tasks import mentorpi_explore_env_cfg as T
from robotlab.tasks import office_terrain as OT
from robotlab.tasks import teacher as TCH
from robotlab.tasks.mdp import car_pose

cfg = load_env_cfg("Robotlab-Goto-MentorPiA1", play=True); cfg.scene.num_envs = 1024
cfg.observations["actor"].enable_corruption = True
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
env.reset()
dev = env.device
ls = OT.LAYOUTS[-T.explore(env).maps.occ.shape[0]:]
CH = torch.zeros(len(ls), OT.MAX_CLUTTER + 1, dtype=torch.bool, device=dev)
for i, l in enumerate(ls):
  for c in l.chairs:
    CH[i, c] = True
cnt = collections.Counter()
with torch.inference_mode():
  for it in range(20 * 50):
    a, _ = TCH.act(env)
    xy, _ = car_pose(env)
    drv = env.action_manager.get_term("drive")
    kt = env._teacher_kturn["on"].clone()
    cool = env._teacher_kturn["cool"].clone()
    back_pre = drv.backing.clone()
    env.step(a)
    v_pre = drv.v_cmd.clone()   # v_cmd of this step is set in process_actions before physics / the reset
    bmp = env.termination_manager.get_term("bump").bool()
    if not bmp.any():
      continue
    e = T.explore(env)
    ix = (xy[:, 0] / OT.OCC_RES).long().clamp(0, OT.OCC_N - 1); iy = (xy[:, 1] / OT.OCC_RES).long().clamp(0, OT.OCC_N - 1)
    near = torch.zeros_like(bmp)
    for dx in range(-9, 10, 3):
      for dy in range(-9, 10, 3):
        cid = e.maps.clutter[e.layout, (ix + dx).clamp(0, OT.OCC_N - 1), (iy + dy).clamp(0, OT.OCC_N - 1)]
        near |= CH[e.layout, cid.clamp(min=0)] & (cid > 0)
    for j in torch.nonzero(bmp)[:, 0].tolist():
      who = "scripted back-up" if back_pre[j] else ("teacher multi-point turn" if kt[j] else (
        "teacher, just after a turn" if cool[j] > 0 else "teacher"))
      d = "reverse" if a[j, 0] < 0 else ("forward" if a[j, 0] > 0 else "stopped")
      cnt[f"{'CHAIR' if near[j] else 'wall/other'} | {d} | {who}"] += 1
for k, v in cnt.most_common():
  print(f"{v:5d}  {k}")
