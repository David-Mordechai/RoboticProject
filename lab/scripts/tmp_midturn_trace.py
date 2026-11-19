"""How do goto mid-hallway turn-back routes fail? (goto v5, 2026-11-05) 1024 cars x 30 s: for the mid-turn routes, how far
the car runs past the turn point, whether it ever reverses, and how the run ends."""
import sys
from dataclasses import asdict
import numpy as np, torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg
import robotlab.tasks  # noqa
from robotlab.tasks import mentorpi_navigate_env_cfg as N
from robotlab.tasks.mdp import car_pose
TASK = "Robotlab-Goto-MentorPiA1"
cfg = load_env_cfg(TASK, play=True); cfg.scene.num_envs = 1024; cfg.episode_length_s = 30.0
cfg.observations["actor"].enable_corruption = True
ag = load_rl_cfg(TASK)
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
venv = RslRlVecEnvWrapper(env, clip_actions=ag.clip_actions)
r = MjlabOnPolicyRunner(venv, asdict(ag), device="cuda:0")
r.load(sys.argv[1], load_cfg={"actor": True}, strict=True, map_location="cuda:0")
pol = r.get_inference_policy(device="cuda:0")
n, dev = env.num_envs, env.device
MID = env._robotlab_midturn.clone(); TS = env._robotlab_turn_s.clone()
nv = N.nav(env); T = nv.path[:, 1].clone(); X0 = nv.path[:, 0].clone()
ax = (T - X0) / torch.linalg.norm(T - X0, dim=-1, keepdim=True).clamp(min=1e-6)
obs = venv.get_observations()
L0 = obs["actor"][:, 4:40] * 3; W0 = (L0[:, 8:10].min(-1).values + L0[:, 26:28].min(-1).values).cpu().numpy()
LAT0 = (nv.path[:, 2] - nv.path[:, 1]); SIDE = torch.sign(ax[:, 0] * LAT0[:, 1] - ax[:, 1] * LAT0[:, 0]).cpu().numpy()  # +1 left
alive = torch.ones(n, dtype=torch.bool, device=dev)
over = torch.zeros(n, device=dev); vmin = torch.zeros(n, device=dev); done1 = torch.zeros_like(alive)
end = torch.full((n,), -1, device=dev); keys = ["lost", "bump", "hit_person", "time_out"]
width = torch.tensor([0.0])
with torch.inference_mode():
  for it in range(20 * 30 - 1):
    xy, yaw = car_pose(env)
    over = torch.where(alive & ~done1, torch.maximum(over, ((xy - T) * ax).sum(-1)), over)
    v = env.scene["car"].data.root_link_lin_vel_b[:, 0]
    vmin = torch.where(alive & ~done1, torch.minimum(vmin, v), vmin)
    obs, _, dones, _ = venv.step(pol(obs))
    done1 |= alive & N.nav(env).reached
    fin = alive & dones.bool()
    for j, k in enumerate(keys):
      end = torch.where(fin & (end < 0) & env.termination_manager.get_term(k).bool(), j, end)
    alive &= ~fin
m = MID.cpu().numpy(); ok = done1.cpu().numpy(); ov = over.cpu().numpy(); vm = vmin.cpu().numpy(); e = end.cpu().numpy()
print(f"mid routes {m.sum()}, completed {ok[m].sum()}")
for name, sel in (("completed", m & ok), ("failed", m & ~ok)):
  if sel.sum():
    print(f"{name}: past turn point median {np.median(ov[sel]):.2f} m (p90 {np.percentile(ov[sel], 90):.2f}), reversed "
          f"(< -0.05 m/s) {np.mean(vm[sel] < -0.05):.0%}, ends: " +
          ", ".join(f"{k} {np.mean(e[sel] == j):.0%}" for j, k in enumerate(keys)) + f", alive at 30 s {np.mean(e[sel] < 0):.0%}")

for lo, hi in ((0, 1.3), (1.3, 1.5), (1.5, 1.7), (1.7, 9)):
  sel = m & (W0 >= lo) & (W0 < hi)
  if sel.sum(): print(f"width {lo:.1f}-{hi:.1f} m: {sel.sum()} routes, completed {ok[sel].mean():.0%}")
for sd, nm in ((1, "left"), (-1, "right")):
  sel = m & (SIDE == sd)
  if sel.sum(): print(f"turn {nm}: {sel.sum()} routes, completed {ok[sel].mean():.0%}")
