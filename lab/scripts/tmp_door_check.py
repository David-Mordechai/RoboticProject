"""goto v4 sanity: doorways exist and every hallway route (incl. through door gaps) runs through free space."""
import numpy as np
from mjlab.envs import ManagerBasedRlEnv
from mjlab.tasks.registry import load_env_cfg
import robotlab.tasks  # noqa
from robotlab.tasks import office_terrain as O
cfg = load_env_cfg("Robotlab-Goto-MentorPiA1"); cfg.scene.num_envs = 16
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
L = O.LAYOUTS[-16:]
doors = bad = segs = 0
for l in L:
  extra = l.occ & ~l.walls & (l.clutter == 0)          # door pieces: occupied, not room walls, not clutter
  doors += int(extra.sum() > 0)
  for a in range(O.NODES * O.NODES):
    for d in (0, 2):
      if l.nbr[a, d] < 0: continue
      segs += 1
      p0, (p1, p2), p3 = l.node_xy[a], l.route[a, d], l.node_xy[l.nbr[a, d]]
      for q0, q1 in ((p0, p1), (p1, p2), (p2, p3)):
        for t in np.linspace(0, 1, 40):
          x, y = q0 + t * (q1 - q0); i, j = int(x / O.OCC_RES), int(y / O.OCC_RES)
          if l.occ[i, j]: bad += 1; break
print(f"floors with doorway walls: {doors} of {len(L)}; door-wall cells {sum(int((l.occ & ~l.walls & (l.clutter == 0)).sum()) for l in L)}; "
      f"route legs hitting an occupied cell: {bad} of {segs * 3}")
