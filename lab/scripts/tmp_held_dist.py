"""goto blocked starts (2026-11-05): per car, total time held by the stop until it reaches its goal (30 s), how many
back-ups, and whether it got out; percentiles show whether the mean comes from a few stuck cars."""
import sys
from dataclasses import asdict
import numpy as np, torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg
import robotlab.tasks  # noqa
from robotlab.tasks import mentorpi_navigate_env_cfg as N
TASK = "Robotlab-Goto-MentorPiA1"
cfg = load_env_cfg(TASK, play=True); cfg.scene.num_envs = 2048; cfg.episode_length_s = 30.0
cfg.observations["actor"].enable_corruption = True
ag = load_rl_cfg(TASK)
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
venv = RslRlVecEnvWrapper(env, clip_actions=ag.clip_actions)
r = MjlabOnPolicyRunner(venv, asdict(ag), device="cuda:0")
r.load(sys.argv[1], load_cfg={"actor": True}, strict=True, map_location="cuda:0")
pol = r.get_inference_policy(device="cuda:0")
drive = env.action_manager.get_term("drive")
B = env._robotlab_blocked.clone(); n = env.num_envs; dev = env.device
alive = torch.ones(n, dtype=torch.bool, device=dev); done = torch.zeros_like(alive)
held = torch.zeros(n, device=dev); backs = torch.zeros(n, device=dev); prev_back = torch.zeros_like(alive)
end = torch.full((n,), -1, device=dev); keys = ["lost", "bump", "hit_person", "time_out"]
obs = venv.get_observations()
with torch.inference_mode():
  for it in range(20 * 30 - 1):
    obs, _, dones, _ = venv.step(pol(obs))
    act = B & alive & ~done
    held += (act & drive.shielded).float() * env.step_dt
    bk = drive.back_left > 0
    backs += (act & bk & ~prev_back).float(); prev_back = bk
    done |= B & alive & N.nav(env).reached
    fin = alive & dones.bool()
    for j, k in enumerate(keys):
      end = torch.where(fin & (end < 0) & env.termination_manager.get_term(k).bool(), j, end)
    alive &= ~fin
b = B.cpu().numpy(); h = held.cpu().numpy()[b]; ok = done.cpu().numpy()[b]; bc = backs.cpu().numpy()[b]; e = end.cpu().numpy()[b]
print(f"blocked starts {b.sum()}: done {ok.mean():.0%}; held mean {h.mean():.2f} s, median {np.median(h):.2f}, p75 {np.percentile(h,75):.2f}, p90 {np.percentile(h,90):.2f}, max {h.max():.1f}")
print(f"  done: held mean {h[ok].mean():.2f} s, back-ups mean {bc[ok].mean():.1f};  not done: held mean {h[~ok].mean() if (~ok).any() else 0:.2f} s, back-ups {bc[~ok].mean() if (~ok).any() else 0:.1f}, ends " + ", ".join(f"{k} {np.mean(e[~ok] == j):.0%}" for j, k in enumerate(keys)) + f", still going {np.mean(e[~ok] < 0):.0%}")
print(f"  share of total held time from the 10% most-held cars: {np.sort(h)[-max(1, len(h)//10):].sum() / max(h.sum(), 1e-9):.0%}")
