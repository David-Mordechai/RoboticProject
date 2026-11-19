"""Debug the teacher's minimum-reverse hold: per step for the first cars that reverse, rev_t before the call, the rear
shield verdict for the stored reverse, and the action returned."""
import torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.tasks.registry import load_env_cfg
import robotlab.tasks  # noqa
from robotlab.tasks import teacher as TCH
from robotlab.tasks import mentorpi_explore_env_cfg as X

cfg = load_env_cfg("Robotlab-Goto-MentorPiA1", play=True); cfg.scene.num_envs = 256
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
env.reset()
watch = {}
with torch.inference_mode():
  for it in range(20 * 30):
    st = getattr(env, "_teacher_kturn", None)
    before = st["rev_t"].clone() if st is not None and "rev_t" in st else torch.zeros(env.num_envs, device=env.device)
    ra = st["rev_a"].clone() if st is not None and "rev_a" in st else torch.zeros(env.num_envs, 2, device=env.device)
    drv = env.action_manager.get_term("drive")
    rsh = X.shield(env, ra[:, 0] * drv.cfg.v_reverse, ra[:, 1] * drv.steer_max)
    a, _ = TCH.act(env)
    env.step(a)
    for j in range(env.num_envs):
      if (a[j, 0] < 0 or before[j] > 0) and (j in watch or len(watch) < 3):
        watch.setdefault(j, [])
        if len(watch[j]) < 25:
          watch[j].append(f"it {it} rev_t_before {before[j]:.2f} rear_shield {int(rsh[j])} a_v {a[j,0]:+.2f} a_st {a[j,1]:+.2f} "
                          f"v_cmd {drv.v_cmd[j]:+.2f} shielded {int(drv.shielded[j])} backing {int(drv.backing[j])}")
for j, rows in watch.items():
  print("== car", j); print("\n".join(rows))
