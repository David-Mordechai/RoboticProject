"""Record a captioned MP4 clip of a task played by a checkpoint (offscreen EGL rendering, real time).

  docker compose run --rm lab python lab/scripts/record_clip.py Robotlab-Office-MentorPiA1 \
      --checkpoint logs/rsl_rl/office/<run>/model_0.pt --seconds 12 --caption "Before training" \
      --out runs/video/01_before.mp4

Clips share size, frame rate and codec, so they join losslessly:
  ffmpeg -f concat -safe 0 -i list.txt -c copy out.mp4

Next to the clip it writes <out>.json: per frame, where the cars, the trail target and the closest
LiDAR hit appear on screen (pixels, projected through the camera MuJoCo rendered with) and what
happened (episode end, crash). make_showcase.py uses it to animate callouts that track the objects.
"""

import argparse
import json
import math
import subprocess
from dataclasses import asdict
from pathlib import Path

import numpy as np
import torch
from PIL import Image, ImageDraw, ImageFont

from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg

import robotlab.tasks  # noqa: F401  (registers the tasks)
from robotlab.tasks.mdp import car_pose

_FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
_FONT_SMALL = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"

p = argparse.ArgumentParser()
p.add_argument("task")
p.add_argument("--checkpoint", default=None, help="policy .pt; omit to drive with zero actions")
p.add_argument("--out", required=True)
p.add_argument("--seconds", type=float, default=12.0)
p.add_argument("--caption", default="")
p.add_argument("--subcaption", default="")
p.add_argument("--width", type=int, default=1280)
p.add_argument("--height", type=int, default=720)
p.add_argument("--distance", type=float, default=None, help="camera distance (m)")
p.add_argument("--elevation", type=float, default=None, help="camera elevation above the ground (deg)")
p.add_argument("--seed", type=int, default=1)
p.add_argument("--chase", action="store_true", help="camera behind the car, turning with it (smoothed)")
args = p.parse_args()

torch.manual_seed(args.seed)
env_cfg = load_env_cfg(args.task, play=True)
env_cfg.scene.num_envs = 1
env_cfg.seed = args.seed
env_cfg.viewer.width, env_cfg.viewer.height = args.width, args.height
if args.distance is not None:
  env_cfg.viewer.distance = args.distance
if args.elevation is not None:
  env_cfg.viewer.elevation = args.elevation
# Task configs use the viser convention (+ = camera above); MuJoCo's offscreen renderer uses - = above.
env_cfg.viewer.elevation = -abs(env_cfg.viewer.elevation)
agent_cfg = load_rl_cfg(args.task)

env = ManagerBasedRlEnv(cfg=env_cfg, device="cuda:0", render_mode="rgb_array")
venv = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
if args.checkpoint:
  runner = MjlabOnPolicyRunner(venv, asdict(agent_cfg), device="cuda:0")
  runner.load(args.checkpoint, load_cfg={"actor": True}, strict=True, map_location="cuda:0")
  policy = runner.get_inference_policy(device="cuda:0")
else:
  policy = lambda obs: torch.zeros(venv.num_envs, venv.num_actions, device="cuda:0")

fps = round(1.0 / env.step_dt)
out = Path(args.out)
out.parent.mkdir(parents=True, exist_ok=True)
ffmpeg = subprocess.Popen(
  ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
   "-s", f"{args.width}x{args.height}", "-r", str(fps), "-i", "-",
   "-c:v", "libx264", "-preset", "slow", "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(out)],
  stdin=subprocess.PIPE,
)
font = ImageFont.truetype(_FONT, args.height // 18)
font_small = ImageFont.truetype(_FONT_SMALL, args.height // 30)


def caption(frame: np.ndarray) -> bytes:
  if not args.caption:
    return frame.tobytes()
  img = Image.fromarray(frame).convert("RGBA")
  layer = Image.new("RGBA", img.size, (0, 0, 0, 0))
  d = ImageDraw.Draw(layer)
  pad = args.height // 36
  w = max(d.textlength(args.caption, font=font), d.textlength(args.subcaption, font=font_small)) + 2 * pad
  h = pad * 2 + font.size + (font_small.size + pad // 2 if args.subcaption else 0)
  d.rounded_rectangle((pad, pad, pad + w, pad + h), radius=pad // 2, fill=(0, 0, 0, 150))
  d.text((2 * pad, 2 * pad - 4), args.caption, font=font, fill=(255, 255, 255, 255))
  if args.subcaption:
    d.text((2 * pad, 2 * pad + font.size + pad // 2 - 4), args.subcaption, font=font_small, fill=(225, 225, 225, 255))
  return np.asarray(Image.alpha_composite(img, layer).convert("RGB")).tobytes()


def project(points: np.ndarray) -> list[list[float] | None]:
  """World points [K, 3] -> pixel (x, y) through the camera of the frame just rendered (None if behind)."""
  scn = env._offline_renderer.renderer.scene
  c0, c1 = scn.camera[0], scn.camera[1]
  pos = (np.array(c0.pos) + np.array(c1.pos)) / 2
  fwd, up = np.array(c0.forward), np.array(c0.up)
  right = np.cross(fwd, up)
  near, top, bottom = c0.frustum_near, c0.frustum_top, c0.frustum_bottom
  half_w = c0.frustum_width if c0.frustum_width > 0 else (top - bottom) / 2 * args.width / args.height
  d = points - pos
  z = d @ fwd
  xn, yn = near * (d @ right) / z, near * (d @ up) / z
  px = (xn - (c0.frustum_center - half_w)) / (2 * half_w) * args.width
  py = (top - yn) / (top - bottom) * args.height
  return [[round(float(x), 1), round(float(y), 1)] if zz > 0 else None for x, y, zz in zip(px, py, z)]


def frame_meta(done: bool) -> dict:
  """Screen positions and events of env 0 for this frame."""
  origin = env.scene.env_origins[0].cpu().numpy()
  car_xy, car_yaw = car_pose(env)
  pts = {"car": np.r_[car_xy[0].cpu().numpy(), 0.06] + origin * [1, 1, 0]}
  m: dict = {"done": done}
  if hasattr(env, "_robotlab_office"):
    from robotlab.tasks.mentorpi_office_env_cfg import trail_target
    s = env._robotlab_office
    pts["leader"] = np.r_[s.xy[0].cpu().numpy(), 0.06] + origin * [1, 1, 0]
    pts["target"] = np.r_[trail_target(env)[0][0].cpu().numpy(), 0.0] + origin * [1, 1, 0]
    lidar = env.scene["lidar"].data
    dist = lidar.distances[0].cpu().numpy()
    hits = lidar.hit_pos_w[0].cpu().numpy()
    ok = dist >= 0
    # Nearest wall/obstacle: ignore rays that hit the leader car.
    on_leader = np.linalg.norm(hits[:, :2] - pts["leader"][:2], axis=1) < 0.2
    obst = np.where(ok & ~on_leader, dist, np.inf)
    k = int(np.argmin(obst))
    m["obstacle_min"] = float(obst[k]) if np.isfinite(obst[k]) else None
    if np.isfinite(obst[k]):
      pts["obstacle_hit"] = hits[k]
    m["rays"] = [p if hit else None for p, hit in zip(project(hits), ok)]
    m["gap"] = float(np.linalg.norm(s.xy[0].cpu().numpy() - car_xy[0].cpu().numpy()))
    m["crash"] = bool(env.termination_manager.get_term("bump")[0]) if done else False
  names = list(pts)
  m.update(zip(names, project(np.stack([pts[n] for n in names]))))
  return m


def chase_camera(done: bool) -> None:
  """Point the camera along the car's heading (MuJoCo azimuth = viewing direction), smoothed."""
  global cam_yaw
  yaw = float(car_pose(env)[1][0])
  if cam_yaw is None or done:
    cam_yaw = yaw
  cam_yaw += 0.12 * math.atan2(math.sin(yaw - cam_yaw), math.cos(yaw - cam_yaw))
  env._offline_renderer._cam.azimuth = math.degrees(cam_yaw)


cam_yaw: float | None = None
meta: list[dict] = []
obs = venv.get_observations()
with torch.inference_mode():
  for _ in range(int(args.seconds * fps)):
    obs, _, dones, _ = venv.step(policy(obs))
    if args.chase:
      chase_camera(bool(dones[0]))
    frame = env.render()
    meta.append(frame_meta(bool(dones[0])))
    ffmpeg.stdin.write(caption(frame))
ffmpeg.stdin.close()
ffmpeg.wait()
env.close()
out.with_suffix(".json").write_text(json.dumps({"fps": fps, "width": args.width, "height": args.height, "frames": meta}))
print(f"wrote {out} ({args.seconds:.0f} s, {fps} fps, {args.width}x{args.height})")
