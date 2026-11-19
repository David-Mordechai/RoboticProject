"""At each person contact (explore, sensor noise on): is it a leg (LiDAR-visible, 0.14 m) or a foot (0.07 m tall, up to
0.19 m ahead of the leg: below the LiDAR plane)? And what did the LiDAR / depth show the car toward that person 0.5 s
before? (v7b 2026-11-04: ~9% of runs end with the car driving into a standing person at 20-90 deg.)"""
import sys, math
from dataclasses import asdict
import numpy as np, torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg
import robotlab.tasks  # noqa
from robotlab.tasks import mentorpi_explore_env_cfg as T
from robotlab.tasks.mdp import car_pose, to_frame
from robotlab.tasks.people import walkers
TASK = "Robotlab-Explore-MentorPiA1"
HX, HY, LEG_R, LEG_Y = 0.1065, 0.0795, 0.035, 0.087     # chassis half size; person.xml leg radius and offset
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
H = 10                                            # steps of history kept (0.5 s)
hist = []                                         # per step: (leg gap [N], lidar toward person [N], v [N], steer [N], shielded [N])
rows = []
def rect_gap(p):                                  # p [..., 2] car frame -> distance to the chassis rectangle
  dx, dy = (p[..., 0].abs() - HX).clamp(min=0), (p[..., 1].abs() - HY).clamp(min=0)
  return torch.hypot(dx, dy)
with torch.inference_mode():
  for _ in range(20 * 60 - 1):
    e, (xy, yaw) = T.explore(env), car_pose(env)
    w = walkers(env, e.maps, e.layout)
    side = torch.stack([-torch.sin(w.yaw), torch.cos(w.yaw)], -1) * LEG_Y
    legs = torch.stack([w.xy + side, w.xy - side], 2)                          # [N, K, 2, 2]
    dl = legs - xy[:, None, None]; c, s_ = torch.cos(yaw)[:, None, None], torch.sin(yaw)[:, None, None]
    rel = torch.stack([c * dl[..., 0] + s_ * dl[..., 1], -s_ * dl[..., 0] + c * dl[..., 1]], -1)
    gap = torch.where(w.active[..., None], rect_gap(rel) - LEG_R, 1e3).amin(-1)  # [N, K] leg surface to chassis
    k = gap.argmin(-1); ar = torch.arange(n, device=dev)
    pc = to_frame(w.xy[ar, k] - xy, yaw)
    b = torch.rad2deg(torch.atan2(pc[:, 1], pc[:, 0])) % 360
    st = env._robotlab_lidar_hist_noisy["buf"][:, 0] * T.LIDAR_RANGE
    lid = st[ar, (b // 10).long() % 36]
    drive = env.action_manager.get_term("drive")
    v = env.scene["car"].data.root_link_lin_vel_b[:, 0]
    hist.append((gap[ar, k].clone(), lid.clone(), v.clone(), drive._steer_cmd.clone(), drive.shielded.clone(),
                 torch.linalg.norm(pc, dim=-1), b.clone(), torch.linalg.norm(w.vel[ar, k], dim=-1), w.speed[ar, k].clone(),
                 # closing speeds along car->person: the car's share and the person's share (m/s)
                 (torch.stack([torch.cos(yaw), torch.sin(yaw)], -1) * v[:, None] * (w.xy[ar, k] - xy)).sum(-1)
                 / torch.linalg.norm(w.xy[ar, k] - xy, dim=-1).clamp(min=1e-3),
                 -(w.vel[ar, k] * (w.xy[ar, k] - xy)).sum(-1) / torch.linalg.norm(w.xy[ar, k] - xy, dim=-1).clamp(min=1e-3)))
    hist = hist[-H:]
    obs, _, dones, _ = venv.step(pol(obs))
    ended = alive & dones.bool()
    hitp = ended & env.termination_manager.get_term("hit_person").bool()
    for j in torch.nonzero(hitp)[:, 0].tolist():
      g0, l0, v0, s0, sh0, d0, b0, _, ws, _, _ = (x[j].item() for x in hist[-1])
      g5, l5, v5, _, _, d5, _, pv5, _, cc, pc_ = (x[j].item() for x in hist[0])
      cc1 = np.mean([x[9][j].item() for x in hist]); pc1 = np.mean([x[10][j].item() for x in hist])
      rows.append((g0, l0, v0, s0, sh0, d0, b0, g5, l5, v5, d5, pv5, ws, cc1, pc1))
    alive &= ~dones.bool()
R = np.array(rows)
print(f"person contacts: {len(R)}")
if len(R):
  foot = R[:, 0] > 0.02
  print(f"leg surface still > 2 cm from the chassis at contact (= foot hit, below the LiDAR): {foot.mean():.0%}; "
        f"leg gap median {np.median(R[:,0]):.3f} m, p90 {np.percentile(R[:,0],90):.3f} m")
  print(f"bearing to person median {np.median(np.where(R[:,6]>180,360-R[:,6],R[:,6])):.0f} deg; car v median {np.median(R[:,2]):+.2f} m/s; "
        f"|steer cmd| median {np.median(np.abs(R[:,3])):.2f}; shield engaged at contact {R[:,4].mean():.0%}")
  print(f"LiDAR toward the person at contact: median {np.median(R[:,1]):.2f} m (person centre {np.median(R[:,5]):.2f} m)")
  print(f"0.5 s before: leg gap median {np.median(R[:,7]):.2f} m, LiDAR toward person {np.median(R[:,8]):.2f} m, "
        f"person centre {np.median(R[:,10]):.2f} m, car v {np.median(R[:,9]):+.2f} m/s")
  print("leg gap at contact histogram (m):", np.histogram(R[:, 0], bins=[-1, 0.02, 0.05, 0.1, 0.15, 0.2, 1])[0])
  if len(R):
    stand = R[:, 12] == 0
    walk_late = (R[:, 12] > 0) & (R[:, 11] > 0.2)
    print(f"person standing all episode (STAND_PROB): {stand.mean():.0%}; a walker still walking 0.5 s before contact: "
          f"{walk_late.mean():.0%}; a walker already stopped 0.5 s before: {((R[:,12] > 0) & (R[:,11] <= 0.2)).mean():.0%}")
  if len(R):
    car_more = R[:, 13] > R[:, 14]
    print(f"closing speed over the last 0.5 s: car's share median {np.median(R[:,13]):.2f} m/s, person's {np.median(R[:,14]):.2f} m/s; "
          f"car closed faster than the person in {car_more.mean():.0%} of contacts ({car_more.sum()} of {len(R)} = {car_more.sum()/1024:.3f} of runs)")
