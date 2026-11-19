"""Where do goto runs get lost? Per lost run: goals reached before, seconds since the current route began, whether the
route began behind the car (a U-turn route), and distance along the route (2026-11-04)."""
import sys, math
from dataclasses import asdict
import numpy as np, torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg
import robotlab.tasks  # noqa
from robotlab.tasks import mentorpi_navigate_env_cfg as N
from robotlab.tasks.mdp import car_pose, to_frame
TASK = "Robotlab-Goto-MentorPiA1"
cfg = load_env_cfg(TASK, play=True); cfg.scene.num_envs = 1024; cfg.episode_length_s = 60.0
cfg.observations["actor"].enable_corruption = True
ag = load_rl_cfg(TASK)
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
venv = RslRlVecEnvWrapper(env, clip_actions=ag.clip_actions)
r = MjlabOnPolicyRunner(venv, asdict(ag), device="cuda:0")
r.load(sys.argv[1], load_cfg={"actor": True}, strict=True, map_location="cuda:0")
pol = r.get_inference_policy(device="cuda:0")
n, dev = env.num_envs, env.device
obs = venv.get_observations()
alive = torch.ones(n, dtype=torch.bool, device=dev)
t_route = torch.zeros(n, device=dev); behind = torch.zeros(n, dtype=torch.bool, device=dev); first = torch.ones_like(behind)
rows = []
with torch.inference_mode():
  for step in range(20 * 60 - 1):
    nv, (xy, yaw) = N.nav(env), car_pose(env)
    if first.any():   # route start: is the route's point 1 m ahead behind the car?
      p = N._points_at(nv, nv.s[:, None] + 1.0)[:, 0]
      b = to_frame(p - xy, yaw)[:, 0] < 0
      behind = torch.where(first, b, behind); t_route = torch.where(first, torch.zeros_like(t_route), t_route)
    # values BEFORE the step (a lost run is reset inside the step)
    s_pre, g_pre = nv.s.clone(), nv.goals_reached.clone()
    a = N._points_at(nv, nv.s[:, None] + torch.tensor([0.0, 0.5, 1.0], device=dev))      # route heading change ahead
    h1 = torch.atan2(*(a[:, 1] - a[:, 0]).flip(-1).unbind(-1)); h2 = torch.atan2(*(a[:, 2] - a[:, 1]).flip(-1).unbind(-1))
    turn_pre = torch.atan2(torch.sin(h2 - h1), torch.cos(h2 - h1)).abs()
    off_pre = N.route_distance(env)
    obs, _, dones, _ = venv.step(pol(obs))
    t_route += env.step_dt
    nv = N.nav(env)
    fin = alive & dones.bool()
    lost = fin & env.termination_manager.get_term("lost").bool()
    for j in torch.nonzero(lost)[:, 0].tolist():
      rows.append((g_pre[j].item(), t_route[j].item(), behind[j].item(), s_pre[j].item(), math.degrees(turn_pre[j].item())))
    first = nv.reached.clone()
    alive &= ~fin
R = np.array(rows)
print(f"lost runs: {len(R)} of {n}")
if len(R):
  print(f"goals before getting lost: 0: {np.mean(R[:,0]==0):.0%}, 1: {np.mean(R[:,0]==1):.0%}, 2+: {np.mean(R[:,0]>=2):.0%}")
  print(f"route began behind the car (U-turn): {np.mean(R[:,2]>0):.0%}; s along route at loss median {np.median(R[:,3]):.2f} m; "
        f"time on the route median {np.median(R[:,1]):.1f} s")
  for lab_, m in (("U-turn routes", R[:,2] > 0), ("forward routes", R[:,2] == 0)):
    if m.any():
      print(f"  {lab_}: {m.sum()}, s median {np.median(R[m,3]):.2f} m, time {np.median(R[m,1]):.1f} s")
  print(f"route turning > 45 deg within the next metre when lost: {np.mean(R[:,4] > 45):.0%}; > 135 deg (U-turn): {np.mean(R[:,4] > 135):.0%}")
