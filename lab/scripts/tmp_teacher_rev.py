"""What ends the teacher's short reverses (2026-11-08): for every reverse run of the drive command (v_cmd < 0, not the
scripted back-up) shorter than 0.5 s, the state at its last step and the step after."""
import collections
import torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.tasks.registry import load_env_cfg
import robotlab.tasks  # noqa
from robotlab.tasks import teacher as TCH

cfg = load_env_cfg("Robotlab-Goto-MentorPiA1", play=True); cfg.scene.num_envs = 512
cfg.observations["actor"].enable_corruption = True
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
env.reset()
n = env.num_envs
rev_len = torch.zeros(n, device=env.device)
prev = None
why = collections.Counter(); lens = collections.Counter()
with torch.inference_mode():
  for it in range(20 * 40):
    a, _ = TCH.act(env)
    st = env._teacher_kturn
    snap = {"kt": st["on"].clone(), "a_v": a[:, 0].clone(), "rev_t": st["rev_t"].clone()}
    env.step(a)
    drv = env.action_manager.get_term("drive")
    rv = (drv.v_cmd < -0.005) & ~drv.backing
    end = ~rv & (rev_len > 0) & (rev_len < 0.5)
    for j in torch.nonzero(end)[:, 0].tolist():
      lens[round(float(rev_len[j]), 2)] += 1
      if drv.backing[j]: why["scripted back-up took over"] += 1
      elif drv.shielded[j] and snap["a_v"][j] < 0: why["teacher still reversing, shield stopped it"] += 1
      elif snap["a_v"][j] >= 0 and snap["kt"][j]: why["multi-point turn switched to forward"] += 1
      elif snap["a_v"][j] >= 0: why["teacher switched to forward (DWA)"] += 1
      elif drv.v_cmd[j] == 0: why["v_cmd 0 (dead band / other)"] += 1
      else: why["other"] += 1
    rev_len = torch.where(rv, rev_len + env.step_dt, 0.0)
print("short reverses by cause:", dict(why.most_common()))
print("lengths (s):", dict(sorted(lens.items())))
