"""Live browser viewer with a scripted camera tour, for demo videos of the virtual lab.

  docker compose run --rm --service-ports viewer python lab/scripts/tour_viewer.py Robotlab-Navigate-MentorPiA1 \\
      --checkpoint logs/rsl_rl/navigate/<run>/model_999.pt

Runs the task in mjlab's viser viewer (http://localhost:8080) with every parallel world shown. The tour starts when
the file lab/scratch/tour_go appears (so a screen recording can start at the same moment) and drives the camera of
every connected browser: a steady zoom between "all offices" and single offices, from one fixed viewing direction.
The keyframes are read from lab/scratch/tour_plan.json at every run, so the path can change without a restart.
"""

import argparse
import json
import math
import threading
import time
from dataclasses import asdict
from pathlib import Path

import numpy as np
import torch
import viser

from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg
from mjlab.viewer import ViewerConfig
from mjlab.viewer.viser.viewer import ViserPlayViewer

import robotlab.tasks  # noqa: F401
from robotlab.tasks.office_terrain import PATCH_SIZE

GO = Path("/workspace/lab/scratch/tour_go")
PLAN = Path("/workspace/lab/scratch/tour_plan.json")
p = argparse.ArgumentParser()
p.add_argument("task")
p.add_argument("--checkpoint", required=True)
p.add_argument("--num-envs", type=int, default=16)
args = p.parse_args()

cfg = load_env_cfg(args.task, play=True)
cfg.scene.num_envs = args.num_envs
cfg.viewer.origin_type = ViewerConfig.OriginType.WORLD      # the tour owns the camera
agent = load_rl_cfg(args.task)
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
venv = RslRlVecEnvWrapper(env, clip_actions=agent.clip_actions)
runner = MjlabOnPolicyRunner(venv, asdict(agent), device="cuda:0")
runner.load(args.checkpoint, load_cfg={"actor": True}, strict=True, map_location="cuda:0")
policy = runner.get_inference_policy(device="cuda:0")

server = viser.ViserServer(host="0.0.0.0", port=8080, label="mjlab")
origins = env.scene.env_origins.cpu().numpy()
centre_all = (origins[:, :2].min(0) + origins[:, :2].max(0) + PATCH_SIZE) / 2
span = float(np.ptp(origins[:, 0]) + PATCH_SIZE)


def car_xy(k: int) -> np.ndarray:
  return env.scene["car"].data.root_link_pos_w[k, :2].cpu().numpy()


def people_near(k: int) -> float:
  from robotlab.tasks.mentorpi_navigate_env_cfg import person_clearance
  return float(person_clearance(env)[k])


def smooth(a, b, u):
  u = 0.5 - 0.5 * math.cos(math.pi * min(max(u, 0.0), 1.0))
  return a + (b - a) * u


def tour() -> None:
  while True:                    # re-armable: each new tour_go file plays the tour once
    while not GO.exists():
      time.sleep(0.05)
    GO.unlink(missing_ok=True)
    play_tour()


def view(name: str, offices: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
  """Camera (position, look-at) for a view: "all" = every office, "office:<k>" = office k. Always the same viewing
  direction (from the south, 50 deg down), only the distance changes: a pure zoom."""
  direction = np.array([0.0, -math.cos(math.radians(50)), math.sin(math.radians(50))])
  if name == "all":
    look, dist = np.r_[centre_all, 0.0], 1.25 * span
  else:
    look, dist = np.r_[offices[int(name.split(":")[1])], 0.0], 1.35 * PATCH_SIZE
  return look + dist * direction, look


def play_tour() -> None:
  """Keyframes from lab/scratch/tour_plan.json (re-read every run): [[t_s, view], ...]; the camera eases between
  consecutive keyframes and holds when two in a row are the same view."""
  plan = json.loads(PLAN.read_text())
  offices = np.unique(np.round(origins[:, :2], 3), axis=0) + PATCH_SIZE / 2
  order = np.argsort(np.linalg.norm(offices - centre_all, axis=1))
  offices = offices[order]                                # office:0 is the one nearest the middle
  keys = [(float(t), view(v, offices)) for t, v in plan]
  t0 = time.perf_counter()
  next_t = t0
  while True:
    t = time.perf_counter() - t0
    if t > keys[-1][0] + 0.5:
      break
    i = max(j for j, (tk, _) in enumerate(keys) if tk <= t) if t >= keys[0][0] else 0
    if i >= len(keys) - 1:
      pos, look = keys[-1][1]
    else:
      (ta, (pa, la)), (tb, (pb, lb)) = keys[i], keys[i + 1]
      u = (t - ta) / max(tb - ta, 1e-6)
      pos, look = smooth(pa, pb, u), smooth(la, lb, u)
    off = np.asarray(getattr(VIEWER._scene, "_scene_offset", np.zeros(3)), dtype=float) if VIEWER else np.zeros(3)
    for cid, client in server.get_clients().items():
      if cid not in UP_SET:              # setting "up" re-orients the camera: once per browser, never per frame
        client.camera.up_direction = (0.0, 0.0, 1.0)
        UP_SET.add(cid)
      with client.atomic():
        client.camera.position = tuple(float(x) for x in pos + off)
        client.camera.look_at = tuple(float(x) for x in look + off)
    next_t += 1 / 30                     # even 30 Hz updates, no drift
    time.sleep(max(0.0, next_t - time.perf_counter()))


class TourViewer(ViserPlayViewer):
  def setup(self) -> None:
    super().setup()
    self._scene.show_all_envs = True                     # every parallel world, not only env 0


VIEWER: TourViewer | None = None
UP_SET: set[int] = set()
GO.unlink(missing_ok=True)
VIEWER = TourViewer(venv, policy, viser_server=server)
threading.Thread(target=tour, daemon=True).start()
VIEWER.run()
