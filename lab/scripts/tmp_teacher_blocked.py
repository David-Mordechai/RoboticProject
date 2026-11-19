"""Teacher on blocked starts (nose to a wall, 2026-11-08): the first 3 s of a few cars that are held longest, per step:
teacher speed / steering, multi-point turn on / direction, manoeuvre mode, shield, back-up."""
import torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.tasks.registry import load_env_cfg
import robotlab.tasks  # noqa
from robotlab.tasks import teacher as TCH

cfg = load_env_cfg("Robotlab-Goto-MentorPiA1", play=True); cfg.scene.num_envs = 1024
cfg.observations["actor"].enable_corruption = True
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
env.reset()
BLK = env._robotlab_blocked.clone()
rows = []
held = torch.zeros(env.num_envs, device=env.device)
with torch.inference_mode():
  for it in range(60):
    a, _ = TCH.act(env)
    st = env._teacher_kturn
    man = getattr(env, "_robotlab_manoeuvre", torch.zeros_like(BLK))
    env.step(a)
    drv = env.action_manager.get_term("drive")
    held += drv.shielded.float() * env.step_dt
    rows.append(torch.stack([a[:, 0], a[:, 1], st["on"].float(), st["dir"], man.float(), drv.shielded.float(),
                             drv.backing.float(), drv.v_cmd], -1).cpu())
ids = torch.nonzero(BLK)[:, 0]
worst = ids[held[ids].argsort(descending=True)[:4]].tolist()
print(f"blocked starts {len(ids)}: held mean over 3 s {float(held[ids].mean()):.2f} s")
for j in worst:
  print(f"== car {j}: held {float(held[j]):.2f} s in 3 s")
  print("  a_v   a_st  kt  dir  man shld back v_cmd")
  for r in rows[::3]:
    print("  " + " ".join(f"{x:+.2f}" for x in r[j].tolist()))
