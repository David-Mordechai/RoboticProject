"""Teacher mid-hallway turn-backs by hall width (2026-11-08): share completed within 60 s, and how the others end."""
import collections
import torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.tasks.registry import load_env_cfg
import robotlab.tasks  # noqa
from robotlab.tasks import mentorpi_navigate_env_cfg as N
from robotlab.tasks import teacher as TCH

cfg = load_env_cfg("Robotlab-Goto-MentorPiA1", play=True); cfg.scene.num_envs = 2048
cfg.observations["actor"].enable_corruption = True
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
env.reset()
n, dev = env.num_envs, env.device
MID = env._robotlab_midturn.clone()
nv = N.nav(env)
width = nv.maps.hall_width[nv.layout].clone()
alive = torch.ones(n, dtype=torch.bool, device=dev)
done = torch.zeros_like(alive); end = {}
kt_time = torch.zeros(n, device=dev); held = torch.zeros(n, device=dev)
with torch.inference_mode():
  for it in range(20 * 60 - 1):
    a, _ = TCH.act(env)
    kt_time += (env._teacher_kturn["on"] & alive & MID & ~done).float() * env.step_dt
    _, _, term, trunc, _ = env.step(a)
    drv = env.action_manager.get_term("drive")
    held += (drv.shielded & alive & MID & ~done).float() * env.step_dt
    done |= MID & alive & N.nav(env).reached
    fin = alive & (term | trunc)
    for k in ("lost", "bump", "hit_person"):
      t = env.termination_manager.get_term(k).bool() & fin & MID & ~done
      for j in torch.nonzero(t)[:, 0].tolist():
        end[j] = k
    alive &= ~fin
bins = [(1.0, 1.2), (1.2, 1.4), (1.4, 1.6), (1.6, 1.81)]
for lo, hi in bins:
  m = MID & (width >= lo) & (width < hi)
  k = int(m.sum())
  if k == 0:
    continue
  ok = int((done & m).sum())
  why = collections.Counter(end.get(j, "still going at 60 s") for j in torch.nonzero(m & ~done)[:, 0].tolist())
  print(f"hall {lo:.1f}-{hi:.1f} m: {ok}/{k} = {ok / k:.0%} done; multi-point turn {float(kt_time[m].mean()):.1f} s, "
        f"held {float(held[m].mean()):.1f} s; not done: {dict(why)}")
