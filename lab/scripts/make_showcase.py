"""Showcase video: how David's robot cars learn to drive, told for everyone.

Story (each on-screen message has a plain-language headline and a small technical line):
  1 hook        trained car in the office: "This little car taught itself to drive"
  2 inspiration Microduck (Pollen Robotics): learned to walk in a simulator, then for real
  3 title       David's Robot Cars: same recipe, new robot
  4 challenge   leader / learner / "stay 0.6 m behind" callouts tracking the cars
  5 senses      the LiDAR: 36 beams drawn sweeping out from the car
  6 learning    animated try -> score -> improve loop, with the point rules
  7 scale       1,024 cars practising at once, hours of driving counter
  8 watch it    attempt #1 -> 100 -> 300 -> final, score graph + progress, crash bursts at impacts
  9 compare     before / after split screen
  10 notice     what it learned: tether with live gap, obstacle highlights
  11 next       real cars, teamwork, formations; credits

1920x1080, 30 fps, H.264 + AAC at -14 LUFS. Open source only: MuJoCo/mjlab renders (record_clip.py,
which also exports per-frame screen positions), Pillow motion graphics, matplotlib, ffmpeg.

  docker compose run --rm lab python lab/scripts/make_showcase.py --run logs/rsl_rl/office/<run>
  (--skip-render reuses runs/video/shots/)
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import subprocess
from collections.abc import Callable, Iterator
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

W, H, FPS = 1920, 1080, 30
TASK = "Robotlab-Office-MentorPiA1"
OUT = Path("runs/video")
SHOTS, SEGS = OUT / "shots", OUT / "segments"
MICRODUCK = OUT / "source/microduck_hero.mov"
MICRODUCK_CUT = (0.0, 15.0)
# "Inspired" by Kevin MacLeod (incompetech.com), CC BY 4.0: credited on the end card.
MUSIC = OUT / "source/inspired_kevin_macleod.mp3"
MUSIC_CREDIT = 'Music: "Inspired" by Kevin MacLeod (incompetech.com), licensed under CC BY 4.0'

ORANGE, BLUE, CYAN = (255, 140, 40), (70, 140, 255), (60, 225, 235)
RED, GREEN, YELLOW = (255, 70, 70), (80, 220, 120), (255, 210, 60)
INK, SLATE, WHITE, MUTED = (14, 16, 22), (26, 30, 40), (245, 246, 248), (165, 172, 188)
_F = "/usr/share/fonts/truetype/dejavu/"


def font(size: int, bold: bool = True) -> ImageFont.FreeTypeFont:
  return ImageFont.truetype(_F + ("DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"), size)


def ease(x: float) -> float:
  x = min(max(x, 0.0), 1.0)
  return 1 - (1 - x) ** 3


def back(x: float) -> float:            # ease-out with a little overshoot ("pop")
  x = min(max(x, 0.0), 1.0)
  c = 1.70158
  return 1 + (c + 1) * (x - 1) ** 3 + c * (x - 1) ** 2


def window(t: float, t0: float, t1: float, fade: float = 0.4) -> float:
  """Visible between t0 and t1, eased in and out."""
  return min(ease((t - t0) / fade), ease((t1 - t) / fade))


# Frame I/O.


def read_frames(path: Path, start: float = 0.0, dur: float | None = None) -> Iterator[np.ndarray]:
  cmd = ["ffmpeg", "-loglevel", "error", "-ss", str(start), "-i", str(path)]
  if dur:
    cmd += ["-t", str(dur)]
  cmd += ["-vf", f"scale={W}:{H}:force_original_aspect_ratio=increase,crop={W}:{H},fps={FPS}",
          "-f", "rawvideo", "-pix_fmt", "rgb24", "-"]
  p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
  while (buf := p.stdout.read(W * H * 3)) and len(buf) == W * H * 3:
    yield np.frombuffer(buf, np.uint8).reshape(H, W, 3)
  p.wait()


def write_segment(path: Path, frames: Iterator[Image.Image]) -> tuple[Path, float]:
  path.parent.mkdir(parents=True, exist_ok=True)
  p = subprocess.Popen(
    ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(FPS),
     "-i", "-", "-c:v", "libx264", "-preset", "medium", "-crf", "16", "-pix_fmt", "yuv420p", str(path)],
    stdin=subprocess.PIPE,
  )
  n = 0
  for im in frames:
    p.stdin.write(im.convert("RGB").tobytes())
    n += 1
  p.stdin.close()
  p.wait()
  print(f"  {path.name}: {n / FPS:.1f} s", flush=True)
  return path, n / FPS


@dataclass
class Shot:
  path: Path
  meta: list[dict]
  fps: int

  def at(self, t: float) -> dict:
    return self.meta[min(int(t * self.fps), len(self.meta) - 1)]


def shot(name: str) -> Shot:
  m = json.loads((SHOTS / f"{name}.json").read_text())
  return Shot(SHOTS / f"{name}.mp4", m["frames"], m["fps"])


def frames_of(s: Shot | Path, dur: float, start: float = 0.0) -> Iterator[tuple[float, Image.Image]]:
  path = s.path if isinstance(s, Shot) else s
  for i, f in enumerate(read_frames(path, start, dur)):
    yield i / FPS, Image.fromarray(f).convert("RGBA")


# Graphics primitives. Everything draws on a transparent layer that is composited onto the frame.


class Layer:
  def __init__(self) -> None:
    self.im = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    self.d = ImageDraw.Draw(self.im)

  def onto(self, base: Image.Image) -> Image.Image:
    return Image.alpha_composite(base, self.im)


def col(c, a: float) -> tuple:
  return (*c, int(255 * max(0.0, min(a, 1.0))))


def text(L: Layer, xy, s: str, f, c, a=1.0, anchor="la", shadow=True) -> None:
  if a <= 0:
    return
  if shadow:
    sh = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    ImageDraw.Draw(sh).text((xy[0] + 3, xy[1] + 4), s, font=f, fill=(0, 0, 0, int(160 * a)), anchor=anchor)
    L.im.alpha_composite(sh.filter(ImageFilter.GaussianBlur(5)))
  L.d.text(xy, s, font=f, fill=col(c, a), anchor=anchor)


def pill(L: Layer, t: float, dur: float, label: str, c=ORANGE) -> None:
  """Chapter tag, top-left."""
  a = window(t, 0.0, dur)
  f = font(30)
  w = L.d.textlength(label, font=f)
  x = 70 - 40 * (1 - a)
  L.d.rounded_rectangle((x, 60, x + w + 56, 116), radius=28, fill=col(c, 0.92 * a))
  L.d.text((x + 28, 88), label, font=f, fill=col(INK, a), anchor="lm")


def headline(L: Layer, t: float, t0: float, t1: float, big: str, small: str = "", c=ORANGE) -> None:
  """Two-layer caption, bottom-left: plain-language headline + small technical line."""
  a = window(t, t0, t1, 0.45)
  if a <= 0:
    return
  fb, fs = font(56), font(30, bold=False)
  w = max(L.d.textlength(big, font=fb), L.d.textlength(small, font=fs) if small else 0) + 100
  x0, h = 70, (178 if small else 112)
  y0 = H - 70 - h
  L.d.rounded_rectangle((x0, y0, x0 + w * ease(a * 1.4), y0 + h), radius=16, fill=col(INK, 0.82 * a))
  L.d.rectangle((x0, y0, x0 + 10, y0 + h), fill=col(c, a))
  k = ease((a - 0.4) / 0.6)
  text(L, (x0 + 48, y0 + 26), big, fb, WHITE, k, shadow=False)
  if small:
    L.d.text((x0 + 48, y0 + 108), small, font=fs, fill=col(MUTED, k))


def ring(L: Layer, p, t: float, c, a: float = 1.0, r: float = 46) -> None:
  """Highlight ring with a repeating pulse."""
  if p is None or a <= 0:
    return
  x, y = p
  rr = r * back(min(a * 1.3, 1.0))
  L.d.ellipse((x - rr, y - rr, x + rr, y + rr), outline=col(c, a), width=5)
  ph = (t * 0.9) % 1.0
  pr = rr * (1 + 0.8 * ph)
  L.d.ellipse((x - pr, y - pr, x + pr, y + pr), outline=col(c, a * (1 - ph) * 0.8), width=3)


def pointer(L: Layer, p, t: float, t0: float, t1: float, title: str, sub: str, c, offset=(160, -150)) -> None:
  """Leader line from the object to a label card; draws on, card pops in."""
  a = window(t, t0, t1)
  if p is None or a <= 0:
    return
  x, y = p
  ex, ey = x + offset[0], y + offset[1]
  e = ease((t - t0) / 0.45)
  ring(L, p, t, c, a, 38)
  sx, sy = x + (ex - x) * 0.28, y + (ey - y) * 0.28        # start outside the ring
  L.d.line((sx, sy, sx + (ex - sx) * e, sy + (ey - sy) * e), fill=col(c, a), width=5)
  k = back((t - t0 - 0.3) / 0.45) * min(1.0, a)
  if k <= 0:
    return
  ft, fs = font(36), font(26, bold=False)
  w = max(L.d.textlength(title, font=ft), L.d.textlength(sub, font=fs) if sub else 0) + 44
  h = 92 if sub else 60
  left = offset[0] < 0
  bx = ex - w if left else ex
  by = ey - h / 2
  cx, cy = (bx + w / 2, by + h / 2)
  box = (cx - w / 2 * k, cy - h / 2 * k, cx + w / 2 * k, cy + h / 2 * k)
  L.d.rounded_rectangle(box, radius=14, fill=col(INK, 0.9 * a), outline=col(c, a), width=3)
  if k > 0.85:
    L.d.text((bx + 22, by + 12), title, font=ft, fill=col(c, a))
    if sub:
      L.d.text((bx + 22, by + 56), sub, font=fs, fill=col(WHITE, a))


def tether(L: Layer, p1, p2, t: float, a: float, label: str, c=YELLOW) -> None:
  """Animated dashed line between two objects with a label at the middle."""
  if p1 is None or p2 is None or a <= 0:
    return
  (x1, y1), (x2, y2) = p1, p2
  n = max(1.0, math.hypot(x2 - x1, y2 - y1))
  dash, off = 18, (t * 60) % 36
  s = -off
  while s < n:
    a0, a1 = max(s, 0) / n, min(s + dash, n) / n
    if a1 > a0:
      L.d.line((x1 + (x2 - x1) * a0, y1 + (y2 - y1) * a0, x1 + (x2 - x1) * a1, y1 + (y2 - y1) * a1),
               fill=col(c, a), width=5)
    s += 36
  mx, my = (x1 + x2) / 2, (y1 + y2) / 2
  f = font(30)
  w = L.d.textlength(label, font=f) + 28
  L.d.rounded_rectangle((mx + 26, my - 22, mx + 26 + w, my + 22), radius=11, fill=col(c, 0.95 * a))
  L.d.text((mx + 40, my), label, font=f, fill=col(INK, a), anchor="lm")


def burst(L: Layer, p, age: float, label: str, c=RED) -> None:
  """Impact burst: radial spikes + popping label, ~1.4 s."""
  if p is None or age < 0 or age > 1.4:
    return
  x, y = p
  a = 1.0 - ease((age - 0.9) / 0.5)
  r0, r1 = 20 + 60 * ease(age / 0.35), 50 + 110 * ease(age / 0.35)
  for k in range(12):
    ang = k * math.pi / 6 + 0.2
    L.d.line((x + r0 * math.cos(ang), y + r0 * math.sin(ang), x + r1 * math.cos(ang), y + r1 * math.sin(ang)),
             fill=col(c, a), width=7)
  s = back(age / 0.3)
  f = font(max(8, int(58 * s)))                      # pop-in starts small, never size 0
  text(L, (x, y - 150), label, f, c, a, anchor="mm")


def progress_bar(L: Layer, frac: float, label: str, sub: str) -> None:
  L.d.rectangle((0, 0, W, 12), fill=col(SLATE, 0.9))
  L.d.rectangle((0, 0, W * frac, 12), fill=col(ORANGE, 1))
  text(L, (W - 70, 74), label, font(40), WHITE, 1, anchor="ra")
  L.d.text((W - 70, 124), sub, font=font(24, bold=False), fill=col(MUTED, 1), anchor="ra")


_VIG: Image.Image | None = None


def grade(img: Image.Image) -> Image.Image:
  global _VIG
  if _VIG is None:
    y, x = np.ogrid[:H, :W]
    r = np.sqrt(((x - W / 2) / (W / 2)) ** 2 + ((y - H / 2) / (H / 2)) ** 2)
    v = np.zeros((H, W, 4), np.uint8)
    v[..., 3] = (np.clip((r - 0.75) / 0.6, 0, 1) * 140).astype(np.uint8)
    _VIG = Image.fromarray(v, "RGBA")
  return Image.alpha_composite(img, _VIG)


def backdrop(t: float) -> Image.Image:
  img = Image.new("RGBA", (W, H), (*INK, 255))
  d = ImageDraw.Draw(img)
  for i in range(-1, 25):
    x = i * 90 + (t * 18) % 90
    d.line((x, 0, x, H), fill=(*SLATE, 255), width=2)
  for j in range(-1, 14):
    y = j * 90 + (t * 10) % 90
    d.line((0, y, W, y), fill=(*SLATE, 255), width=2)
  glow = Image.new("RGBA", (W, H), (0, 0, 0, 0))
  ImageDraw.Draw(glow).ellipse((W / 2 - 700, H / 2 - 420, W / 2 + 700, H / 2 + 420), fill=(*BLUE, 34))
  return Image.alpha_composite(img, glow.filter(ImageFilter.GaussianBlur(160)))


def car_icon(L: Layer, x: float, y: float, s: float, c, a: float = 1.0, yaw: float = 0.0) -> None:
  """Top-down car glyph (body, 4 wheels, LiDAR dot), rotated by yaw."""
  def pt(u, v):
    return (x + s * (u * math.cos(yaw) - v * math.sin(yaw)), y + s * (u * math.sin(yaw) + v * math.cos(yaw)))
  def quad(u0, v0, u1, v1, fill):
    L.d.polygon([pt(u0, v0), pt(u1, v0), pt(u1, v1), pt(u0, v1)], fill=fill)
  for u, v in ((0.6, -0.62), (0.6, 0.62), (-0.6, -0.62), (-0.6, 0.62)):
    quad(u - 0.22, v - 0.1, u + 0.22, v + 0.1, col((20, 20, 24), a))
  quad(-1.0, -0.55, 1.0, 0.55, col(c, a))
  cx, cy = pt(-0.3, 0)
  L.d.ellipse((cx - 0.18 * s, cy - 0.18 * s, cx + 0.18 * s, cy + 0.18 * s), fill=col((30, 30, 34), a))


# Score curve (from the training log).


@dataclass
class Curve:
  panel: Image.Image
  x_of: Callable[[float], float]
  y_of: Callable[[float], float]
  its: np.ndarray
  rew: np.ndarray
  wall_s: float


def make_curve(run: Path) -> Curve:
  import matplotlib
  matplotlib.use("Agg")
  import matplotlib.pyplot as plt
  from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

  ea = EventAccumulator(glob.glob(str(run / "events*"))[0])
  ea.Reload()
  ev = ea.Scalars("Train/mean_reward")
  its, rew = np.array([e.step for e in ev], float), np.array([e.value for e in ev], float)
  smooth = np.convolve(np.pad(rew, 3, mode="edge"), np.ones(7) / 7, mode="valid")
  pw, ph, dpi = 640, 360, 100
  fig = plt.figure(figsize=(pw / dpi, ph / dpi), dpi=dpi)
  fig.patch.set_alpha(0)
  ax = fig.add_axes((0.12, 0.2, 0.84, 0.58))
  ax.set_facecolor((0, 0, 0, 0))
  ax.plot(its, smooth, color=np.array(ORANGE) / 255, lw=3.5)
  ax.set_xlim(0, its.max())
  ax.set_ylim(min(0, smooth.min()) - 2, smooth.max() * 1.15)
  for s in ("top", "right"):
    ax.spines[s].set_visible(False)
  for s in ("left", "bottom"):
    ax.spines[s].set_color(np.array(MUTED) / 255)
  ax.tick_params(colors=np.array(MUTED) / 255, labelsize=11)
  ax.set_xlabel("practice rounds (training iterations)", color=np.array(MUTED) / 255, fontsize=12)
  fig.text(0.12, 0.9, "Driving score", color=np.array(WHITE) / 255, fontsize=17, weight="bold")
  fig.text(0.12, 0.83, "mean episode reward", color=np.array(MUTED) / 255, fontsize=11)
  fig.canvas.draw()
  panel = Image.frombuffer("RGBA", fig.canvas.get_width_height(), fig.canvas.buffer_rgba()).copy()
  x_of = lambda it: ax.transData.transform((it, 0))[0]
  y_of = lambda r: ph - ax.transData.transform((0, r))[1]
  plt.close(fig)
  wall = ev[-1].wall_time - ev[0].wall_time
  return Curve(panel, x_of, y_of, its, smooth, wall)


def curve_overlay(img: Image.Image, cv: Curve, it_now: float, a: float) -> Image.Image:
  pw, ph = cv.panel.size
  card = Image.new("RGBA", (pw, ph), (0, 0, 0, 0))
  ImageDraw.Draw(card).rounded_rectangle((0, 0, pw - 1, ph - 1), radius=18, fill=(*INK, 215))
  cut = int(cv.x_of(it_now)) + 2
  mask = Image.new("L", (pw, ph), 255)
  ImageDraw.Draw(mask).rectangle((max(cut, int(cv.x_of(0))), int(ph * 0.2), pw, int(ph * 0.81)), fill=0)
  card.alpha_composite(Image.composite(cv.panel, Image.new("RGBA", (pw, ph)), mask))
  r = float(np.interp(it_now, cv.its, cv.rew))
  mx, my = cv.x_of(it_now), cv.y_of(r)
  glow = Image.new("RGBA", (pw, ph), (0, 0, 0, 0))
  ImageDraw.Draw(glow).ellipse((mx - 16, my - 16, mx + 16, my + 16), fill=(*ORANGE, 170))
  card.alpha_composite(glow.filter(ImageFilter.GaussianBlur(6)))
  d = ImageDraw.Draw(card)
  d.ellipse((mx - 7, my - 7, mx + 7, my + 7), fill=(*WHITE, 255))
  d.text((pw - 26, 24), f"{r:.0f}", font=font(40), fill=(*ORANGE, 255), anchor="ra")
  card.putalpha(card.getchannel("A").point(lambda v: int(v * a)))
  img.alpha_composite(card, (W - pw - 50, 150))
  return img


# Scenes.


def s_hook(sh: Shot, dur=5.5):
  for t, im in frames_of(sh, dur):
    L = Layer()
    a = ease((t - 0.3) / 0.8)
    L.im.alpha_composite(Image.new("RGBA", (W, H), (0, 0, 0, int(150 * a))))
    text(L, (W / 2, H / 2 - 50 + 30 * (1 - a)), "This little car taught itself to drive", font(86), WHITE, a, "mm")
    b = ease((t - 1.3) / 0.8)
    text(L, (W / 2, H / 2 + 55), "Nobody programmed how. It learned by practising.", font(42, False), ORANGE, b, "mm")
    yield L.onto(grade(im))


def s_microduck(dur: float):
  for t, im in frames_of(MICRODUCK, dur, MICRODUCK_CUT[0]):
    L = Layer()
    pill(L, t, dur, "THE INSPIRATION", CYAN)
    headline(L, t, 1.0, 7.5, "Microduck learned to walk in a video-game world…",
             "Pollen Robotics × Hugging Face · trained in MuJoCo with mjlab + PPO", CYAN)
    headline(L, t, 7.9, dur - 0.3, "…and then walked in the real world.",
             "sim-to-real: the same neural network runs on the robot", CYAN)
    L.d.text((W - 60, H - 40), "Footage: Pollen Robotics / Hugging Face", font=font(24, False),
             fill=col(WHITE, 0.8 * window(t, 0, dur)), anchor="rs")
    yield L.onto(im)


def s_title(dur=5.0):
  for i in range(int(dur * FPS)):
    t = i / FPS
    L = Layer()
    a = ease((t - 0.2) / 0.7)
    text(L, (W / 2, H / 2 - 80 - 30 * (1 - a)), "DAVID'S ROBOT CARS", font(124), WHITE, a, "mm")
    w = 780 * ease((t - 0.6) / 0.7)
    L.d.rectangle((W / 2 - w / 2, H / 2 + 2, W / 2 + w / 2, H / 2 + 9), fill=col(ORANGE, 1))
    text(L, (W / 2, H / 2 + 72), "Same idea, new robot: can a car learn to drive this way?", font(48, False), ORANGE,
         ease((t - 0.9) / 0.7), "mm")
    text(L, (W / 2, H / 2 + 160), "2× Hiwonder MentorPi A1  ·  MuJoCo Warp  ·  mjlab  ·  PPO (rsl_rl)", font(30, False),
         MUTED, ease((t - 1.5) / 0.7), "mm")
    yield L.onto(backdrop(t))


def s_challenge(sh: Shot, dur=8.0):
  for t, im in frames_of(sh, dur):
    m = sh.at(t)
    L = Layer()
    pill(L, t, dur, "THE CHALLENGE")
    pointer(L, m.get("leader"), t, 0.6, dur - 0.3, "LEADER", "drives a random route", BLUE, (190, -170))
    pointer(L, m.get("car"), t, 2.0, dur - 0.3, "LEARNER", "drives itself, nobody steers it", ORANGE, (-190, 170))
    tether(L, m.get("car"), m.get("leader"), t, window(t, 3.6, dur - 0.3), "stay 0.6 m behind")
    headline(L, t, 4.4, dur - 0.2, "Follow the leader through the office, touch nothing",
             "follow the leader's path at 0.6 m · walls, clutter and the leader count as collisions")
    yield L.onto(grade(im))


def s_senses(sh: Shot, dur=7.0):
  for t, im in frames_of(sh, dur):
    m = sh.at(t)
    L = Layer()
    pill(L, t, dur, "HOW IT SEES", CYAN)
    car, rays = m.get("car"), m.get("rays") or []
    sweep = ease((t - 0.6) / 1.6)                       # rays appear one by one around the car
    a = window(t, 0.6, dur - 0.3)
    if car:
      for k, p in enumerate(rays):
        if p is None or k / max(len(rays), 1) > sweep:
          continue
        L.d.line((car[0], car[1], p[0], p[1]), fill=col(CYAN, 0.55 * a), width=3)
        L.d.ellipse((p[0] - 7, p[1] - 7, p[0] + 7, p[1] + 7), fill=col(CYAN, a))
      ring(L, car, t, CYAN, a, 34)
    headline(L, t, 1.0, 3.9, "A spinning laser measures the distance to everything",
             "2D LiDAR: 36 beams, every 10°, up to 3 m, 20 times a second", CYAN)
    headline(L, t, 4.1, dur - 0.2, "That is all it sees, plus where the leader is",
             "policy input: 36 ranges + own speed/steering + leader's relative pose", CYAN)
    yield L.onto(grade(im))


def s_learning(dur=10.0):
  nodes = [("TRY", "it drives", (W / 2 - 420, 430), ORANGE), ("SCORE", "gets points", (W / 2 + 420, 430), YELLOW),
           ("IMPROVE", "adjusts its brain", (W / 2, 760), GREEN)]
  for i in range(int(dur * FPS)):
    t = i / FPS
    L = Layer()
    pill(L, t, dur, "HOW IT LEARNS", GREEN)
    text(L, (W / 2, 175), "Practice, with points", font(70), WHITE, ease((t - 0.2) / 0.6), "mm")
    cyc = max(0.0, t - 1.2) / 1.6                      # one full lap every 4.8 s
    for k, (name, sub, (x, y), c) in enumerate(nodes):
      a = back((t - 0.5 - 0.25 * k) / 0.5)
      if a <= 0:
        continue
      active = int(cyc) % 3 == k and t > 1.2
      r = 125 * a * (1.08 if active else 1.0)
      L.d.ellipse((x - r, y - r, x + r, y + r), fill=col(INK, 0.95), outline=col(c, 1), width=10 if active else 5)
      text(L, (x, y - 18), name, font(42), c, min(a, 1), "mm", shadow=False)
      L.d.text((x, y + 38), sub, font=font(28, False), fill=col(WHITE, min(a, 1)), anchor="mm")
    for k in range(3):                                  # arrows draw on in sequence
      (x0, y0), (x1, y1) = nodes[k][2], nodes[(k + 1) % 3][2]
      e = ease((t - 1.0 - 0.3 * k) / 0.5)
      if e <= 0:
        continue
      dx, dy = x1 - x0, y1 - y0
      n = math.hypot(dx, dy)
      ux, uy = dx / n, dy / n
      sx, sy = x0 + ux * 150, y0 + uy * 150
      ex, ey = sx + (dx - ux * 300) * e, sy + (dy - uy * 300) * e
      L.d.line((sx, sy, ex, ey), fill=col(WHITE, 0.85), width=6)
      if e > 0.95:
        L.d.polygon([(ex + ux * 22, ey + uy * 22), (ex - uy * 16, ey + ux * 16), (ex + uy * 16, ey - ux * 16)],
                    fill=col(WHITE, 0.85))
      # a dot travelling around the loop
    if t > 1.6:
      seg = int(cyc) % 3
      fr = cyc - int(cyc)
      (x0, y0), (x1, y1) = nodes[seg][2], nodes[(seg + 1) % 3][2]
      px, py = x0 + (x1 - x0) * fr, y0 + (y1 - y0) * fr
      L.d.ellipse((px - 14, py - 14, px + 14, py + 14), fill=col(ORANGE, 1))
    rules = [(GREEN, "+ points", "for staying right behind the leader"),
             (RED, "− points", "for touching anything, or losing the leader")]
    for k, (c, a_, b_) in enumerate(rules):
      a = ease((t - 2.6 - 0.5 * k) / 0.5)
      y = 905 + 58 * k
      x = W / 2 - 560
      L.d.text((x, y), a_, font=font(40), fill=col(c, a))
      L.d.text((x + L.d.textlength(a_, font=font(40)) + 24, y + 4), b_, font=font(36, False), fill=col(WHITE, a))
    a = ease((t - 4.0) / 0.6)
    L.d.text((W - 60, H - 34), "reinforcement learning (PPO) · reward = path tracking (fine + coarse) + heading − collisions − losing the leader − jerky steering",
             font=font(24, False), fill=col(MUTED, a), anchor="rs")
    yield L.onto(backdrop(t + 3))


def s_scale(cv: Curve, envs: int, iters: int, steps: int, step_dt: float, dur=8.0):
  hours = iters * steps * envs * step_dt / 3600
  minutes = cv.wall_s / 60
  cols, rows = 48, 22                                   # 1,056 slots; the first `envs` are filled
  for i in range(int(dur * FPS)):
    t = i / FPS
    L = Layer()
    pill(L, t, dur, "PRACTICE AT SCALE", BLUE)
    fill = ease((t - 0.4) / 2.2)
    shown = int(envs * fill)
    x0, y0, dx, dy = 150, 170, (W - 300) / cols, 560 / rows
    for k in range(shown):
      r, c_ = divmod(k, cols)
      yaw = 0.6 * math.sin(k * 1.7 + t * 2.0)
      car_icon(L, x0 + c_ * dx + dx / 2, y0 + r * dy + dy / 2, 11, ORANGE, 0.9, yaw)
    text(L, (W / 2, 820), f"{shown:,} cars practise at the same time", font(64), WHITE, ease((t - 0.4) / 0.5), "mm")
    h = hours * ease((t - 2.8) / 2.0)
    text(L, (W / 2, 920), f"{h:,.0f} hours of driving practice… in {minutes:.0f} minutes", font(52, False), ORANGE,
         ease((t - 2.6) / 0.5), "mm")
    L.d.text((W / 2, 1010), f"{envs:,} parallel environments on one laptop GPU (RTX 2060) · MuJoCo Warp physics · "
                            f"{iters} iterations × {steps} steps", font=font(26, False),
             fill=col(MUTED, ease((t - 3.5) / 0.6)), anchor="mm")
    yield L.onto(backdrop(t + 6))


def STAGE_ITERS(last: int) -> list[int]:
  """Checkpoints shown in "watch it learn": untrained, two early ones, final."""
  return [0, 50, 100, last]


@dataclass
class Stage:
  shot: Shot
  it: int
  big: str
  small: str


def crash_events(sh: Shot) -> list[tuple[float, list[float] | None, str]]:
  """(time, screen position, label) for each episode end in the shot; position = last frame before the reset."""
  ev = []
  for k, m in enumerate(sh.meta):
    if m.get("done") and k > 0:
      ev.append((k / sh.fps, sh.meta[k - 1].get("car"), "CRASH!" if m.get("crash") else "LOST THE LEADER"))
  return ev


def s_stage(st: Stage, it_from: float, cv: Curve, total: int, dur: float, first: bool):
  events = crash_events(st.shot)
  for t, im in frames_of(st.shot, dur):
    L = Layer()
    it_now = it_from + (st.it - it_from) * ease(t / 1.0)
    pill(L, t, dur, "WATCH IT LEARN")
    progress_bar(L, it_now / total, f"round {int(round(it_now))} of {total}", "training iteration")
    m = st.shot.at(t)
    ring(L, m.get("car"), t, ORANGE, 0.85, 40)
    for te, p, label in events:
      burst(L, p, t - te, label, RED if label == "CRASH!" else YELLOW)
    headline(L, t, 0.3, dur - 0.1, st.big, st.small)
    out = L.onto(grade(im))
    yield curve_overlay(out, cv, it_now, ease(t / 0.5) if first else 1.0)


def s_split(before: Shot, after: Shot, total: int, dur=8.0):
  ev_b = crash_events(before)
  for (t, a), (_, b) in zip(frames_of(before, dur), frames_of(after, dur)):
    im = Image.new("RGBA", (W, H))
    im.paste(a.crop((W // 4, 0, W // 4 + W // 2, H)), (0, 0))
    im.paste(b.crop((W // 4, 0, W // 4 + W // 2, H)), (W // 2, 0))
    L = Layer()
    for te, p, label in ev_b:
      if p is not None and W / 4 < p[0] < 3 * W / 4:
        burst(L, (p[0] - W / 4, p[1]), t - te, label, RED if label == "CRASH!" else YELLOW)
    e = ease(t / 0.6)
    L.d.rectangle((W / 2 - 4, H * (1 - e) / 2, W / 2 + 4, H * (1 + e) / 2), fill=col(WHITE, 1))
    for x, lab, sub, c in ((W / 4, "BEFORE", "round 0", BLUE), (3 * W / 4, "AFTER", f"round {total}", ORANGE)):
      a = window(t, 0.3, dur)
      L.d.rounded_rectangle((x - 170, 60, x + 170, 160), radius=20, fill=col(c, 0.92 * a))
      L.d.text((x, 96), lab, font=font(46), fill=col(INK, a), anchor="mm")
      L.d.text((x, 140), sub, font=font(24, False), fill=col(INK, 0.9 * a), anchor="mm")
    headline(L, t, 1.2, dur - 0.1, "Same office, same start: before and after practice",
             f"left: untrained policy · right: after {total} iterations")
    yield L.onto(grade(im))


def s_notice(chase: Shot, overview: Shot, d1=10.0, d2=7.0, near=0.35):
  # Obstacle moments: nearest wall/obstacle within `near` m for >= 0.3 s, shown for >= 1.6 s.
  close = [m.get("obstacle_min") is not None and m["obstacle_min"] < near for m in chase.meta]
  spans, k = [], 0
  while k < len(close):
    if close[k]:
      j = k
      while j < len(close) and close[j]:
        j += 1
      if (j - k) / chase.fps >= 0.3:
        spans.append((k / chase.fps, max(j / chase.fps, k / chase.fps + 1.6)))
      k = j
    else:
      k += 1
  for t, im in frames_of(chase, d1):
    m = chase.at(t)
    L = Layer()
    pill(L, t, d1, "WHAT IT LEARNED")
    gap = m.get("gap")
    tether(L, m.get("car"), m.get("leader"), t, window(t, 0.5, d1 - 0.2), f"{gap:.2f} m" if gap else "")
    for t0, t1 in spans:
      a = window(t, t0, t1, 0.25)
      if a > 0:
        p = m.get("obstacle_hit")
        ring(L, p, t, RED, a, 30)
        if p:
          text(L, (p[0], p[1] - 70), f"wall {m['obstacle_min']:.2f} m away", font(30), RED, a, "mm")
    headline(L, t, 0.4, 4.9, "It keeps its distance…", "the yellow line: live gap to the leader (target 0.6 m)", YELLOW)
    headline(L, t, 5.1, d1 - 0.1, "…takes the turns and stays off the walls",
             "red ring: the closest thing its LiDAR sees", RED)
    yield L.onto(grade(im))
  z = 1.6                                              # zoom into the centre so the cars read
  def zoom(p):
    return None if p is None else ((p[0] - W / 2) * z + W / 2, (p[1] - H / 2) * z + H / 2)
  for t, im in frames_of(overview, d2):
    m = overview.at(t)
    cw, ch = W / z, H / z
    im = im.crop((int((W - cw) / 2), int((H - ch) / 2), int((W + cw) / 2), int((H + ch) / 2))).resize((W, H), Image.LANCZOS)
    L = Layer()
    pill(L, t, d2, "BIRD'S-EYE VIEW", BLUE)
    ring(L, zoom(m.get("leader")), t, BLUE, window(t, 0.3, d2), 42)
    ring(L, zoom(m.get("car")), t, ORANGE, window(t, 0.3, d2), 42)
    headline(L, t, 0.4, d2 - 0.1, "The leader picks random turns; the learner has to react",
             "no map, no fixed route: the same network handles every corner", BLUE)
    yield L.onto(grade(im))


def s_next(dur=8.0):
  items = [("Put the brain on the real cars", "same network, running on the Raspberry Pi 5 at 20 Hz"),
           ("Two cars learning together as a team", "multi-agent RL (MAPPO)"),
           ("Formations: line, side by side, diagonal", "switch formation on command, avoid obstacles in formation")]
  for i in range(int(dur * FPS)):
    t = i / FPS
    L = Layer()
    text(L, (160, 170), "WHAT'S NEXT", font(72), ORANGE, ease((t - 0.2) / 0.6))
    for k, (big, small) in enumerate(items):
      b = ease((t - 0.7 - 0.35 * k) / 0.6)
      y = 310 + 130 * k
      L.d.rectangle((160, y + 12, 174, y + 54), fill=col(ORANGE, b))
      L.d.text((205 + 30 * (1 - b), y), big, font=font(50, False), fill=col(WHITE, b))
      L.d.text((205 + 30 * (1 - b), y + 66), small, font=font(28, False), fill=col(MUTED, b))
    c = ease((t - 2.4) / 0.8)
    credits = ["Microduck footage: Pollen Robotics / Hugging Face  ·  github.com/pollen-robotics/microduck_rl",
               MUSIC_CREDIT,
               "Built with open source: MuJoCo · MuJoCo Warp · mjlab · rsl_rl · PyTorch · ffmpeg"]
    for k, line in enumerate(credits):
      L.d.text((160, H - 225 + 46 * k), line, font=font(28, False), fill=col(MUTED, c))
    L.im.alpha_composite(Image.new("RGBA", (W, H), (0, 0, 0, int(255 * (1 - ease((dur - t) / 0.9))))))
    yield L.onto(backdrop(t + 10))


# Shots and assembly.


def render_shots(run: Path, last: int) -> None:
  def rec(name, ckpt, seconds, seed, distance, elevation, chase=True):
    cmd = ["python", "lab/scripts/record_clip.py", TASK, "--checkpoint", str(run / f"model_{ckpt}.pt"),
           "--seconds", str(seconds), "--seed", str(seed), "--distance", str(distance), "--elevation", str(elevation),
           "--width", str(W), "--height", str(H), "--out", str(SHOTS / f"{name}.mp4")]
    subprocess.run(cmd + (["--chase"] if chase else []), check=True)
  rec("hook", last, 6, 11, 2.4, 28)
  rec("challenge", last, 8.5, 5, 4.2, 50)
  rec("senses", last, 7.5, 13, 3.2, 72)
  for it in STAGE_ITERS(last):
    rec(f"iter_{it}", it, 8.5, 3, 3.6, 50)
  rec("notice", last, 10.5, 21, 3.0, 45)
  rec("overview", last, 7.5, 8, 8.0, 82, chase=False)


def assemble(segs: list[tuple[Path, float]], transitions: list[tuple[str, float]], md_index: int, out: Path) -> None:
  inputs, fc = [], []
  for p, _ in segs:
    inputs += ["-i", str(p)]
  inputs += ["-ss", str(MICRODUCK_CUT[0]), "-t", str(segs[md_index][1]), "-i", str(MICRODUCK)]
  label, t_end, starts = "[0:v]", segs[0][1], [0.0]
  for k, ((_, d), (kind, td)) in enumerate(zip(segs[1:], transitions), start=1):
    off = t_end - td
    starts.append(off)
    fc.append(f"{label}[{k}:v]xfade=transition={kind}:duration={td}:offset={off:.3f}[v{k}]")
    label, t_end = f"[v{k}]", off + d
  # Audio: music bed over the whole film (fade in, fade out over the end card), Microduck's own sound
  # quietly underneath its segment, then loudness-normalized to YouTube's -14 LUFS.
  inputs += ["-i", str(MUSIC)]
  delay = int(starts[md_index] * 1000)
  fc.append(f"[{len(segs)}:a]afade=t=in:d=0.6,afade=t=out:st={segs[md_index][1] - 1.2:.2f}:d=1.2,volume=0.35,"
            f"adelay={delay}|{delay},apad=whole_dur={t_end:.3f}[md]")
  fc.append(f"[{len(segs) + 1}:a]atrim=0:{t_end:.3f},asetpts=PTS-STARTPTS,afade=t=in:d=1.5,"
            f"afade=t=out:st={t_end - 3.5:.3f}:d=3.5,volume=0.8[mu]")
  fc.append("[mu][md]amix=inputs=2:duration=first:normalize=0,loudnorm=I=-14:TP=-1.5:LRA=11[a]")
  subprocess.run(["ffmpeg", "-y", "-loglevel", "error", *inputs, "-filter_complex", ";".join(fc),
                  "-map", label, "-map", "[a]", "-c:v", "libx264", "-preset", "slow", "-crf", "18",
                  "-pix_fmt", "yuv420p", "-c:a", "aac", "-b:a", "192k", "-ar", "48000",
                  "-movflags", "+faststart", "-t", f"{t_end:.3f}", str(out)], check=True)
  print(f"wrote {out} ({t_end:.1f} s)")


def ssim(ref: Path, test: Path) -> float:
  r = subprocess.run(["ffmpeg", "-hide_banner", "-i", str(test), "-i", str(ref), "-lavfi", "ssim", "-f", "null", "-"],
                     capture_output=True, text=True)
  return float(r.stderr.rsplit("All:", 1)[1].split()[0])


def deliver(master: Path, out: Path, min_ssim: float = 0.99) -> None:
  """Smallest H.264 file that is visually indistinguishable from the master (SSIM >= min_ssim).

  H.264 (not H.265) so it plays everywhere without extra codecs; tune=animation suits flat motion
  graphics and clean renders; aq-mode 3 keeps smooth dark gradients (the vignette) free of the block
  banding that whole-frame SSIM does not catch; 128 kb/s AAC. Binary search over CRF: 2-3 encodes.
  """
  def encode(crf: int) -> tuple[Path, float]:
    trial = out.with_name(f"{out.stem}_crf{crf}.mp4")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(master), "-c:v", "libx264", "-preset", "slow",
                    "-tune", "animation", "-aq-mode", "3", "-crf", str(crf), "-pix_fmt", "yuv420p", "-profile:v", "high",
                    "-c:a", "aac", "-b:a", "128k", "-movflags", "+faststart", str(trial)], check=True)
    q = ssim(master, trial)
    print(f"  crf {crf}: {trial.stat().st_size / 1e6:5.1f} MB, SSIM {q:.4f}", flush=True)
    return trial, q

  lo, hi, best = 18, 32, None                           # find the largest CRF that still passes
  while lo <= hi:
    crf = (lo + hi) // 2
    trial, q = encode(crf)
    if q >= min_ssim:
      if best is not None:
        best.unlink()
      best, lo = trial, crf + 2                         # 2-step resolution is plenty
    else:
      trial.unlink()
      hi = crf - 2
  assert best is not None, "no CRF reached the quality bar"
  best.replace(out)
  print(f"delivered {out} ({out.stat().st_size / 1e6:.1f} MB; master {master.stat().st_size / 1e6:.1f} MB)")


def main() -> None:
  ap = argparse.ArgumentParser()
  ap.add_argument("--run", required=True, type=Path)
  ap.add_argument("--skip-render", action="store_true")
  ap.add_argument("--only", default="", help="comma list of segment names to rebuild (others reused)")
  args = ap.parse_args()
  ckpts = sorted(int(Path(p).stem.split("_")[1]) for p in glob.glob(str(args.run / "model_*.pt")))
  last = ckpts[-1]
  if not args.skip_render:
    render_shots(args.run, last)
  cv = make_curve(args.run)
  total = last                                         # rounds trained = the last checkpoint's iteration
  # Read the few numbers we need straight from the run's params (mjlab's YAML carries python tags).
  import re
  def param(file: str, key: str) -> float:
    return float(re.search(rf"^\s*{key}: ([0-9.e-]+)$", (args.run / "params" / file).read_text(), re.M).group(1))
  envs, steps = int(param("env.yaml", "num_envs")), int(param("agent.yaml", "num_steps_per_env"))
  step_dt = param("env.yaml", "timestep") * param("env.yaml", "decimation")

  stages = [Stage(shot(f"iter_{it}"), it, big, small) for it, (big, small) in zip(STAGE_ITERS(last), (
    ("Attempt #1: it has no idea what to do", "round 0 · random neural network"),
    ("After 50 rounds: it starts to follow", "iteration 50"),
    ("After 100 rounds: it takes the turns", "iteration 100"),
    (f"After {last} rounds: it has learned the skill", "evaluation: 100% of 512 one-minute test drives, 0 crashes"),
  ))]
  only = set(filter(None, args.only.split(",")))
  def seg(name: str, gen: Callable[[], Iterator[Image.Image]]) -> tuple[Path, float]:
    path = SEGS / f"{name}.mp4"
    if only and name not in only and path.exists():
      d = float(subprocess.run(["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0",
                                str(path)], capture_output=True, text=True).stdout)
      return path, d
    return write_segment(path, gen())

  print("segments:", flush=True)
  md = MICRODUCK_CUT[1] - MICRODUCK_CUT[0]
  segs = [
    seg("01_hook", lambda: s_hook(shot("hook"))),
    seg("02_microduck", lambda: s_microduck(md)),
    seg("03_title", lambda: s_title()),
    seg("04_challenge", lambda: s_challenge(shot("challenge"))),
    seg("05_senses", lambda: s_senses(shot("senses"))),
    seg("06_learning", lambda: s_learning()),
    seg("07_scale", lambda: s_scale(cv, envs, total, steps, step_dt)),
  ]
  froms = [0] + STAGE_ITERS(last)[:-1]
  for k, st in enumerate(stages):
    segs.append(seg(f"08_stage_{k}", lambda st=st, k=k: s_stage(st, froms[k], cv, total, 6.5, k == 0)))
  segs.append(seg("09_split", lambda: s_split(stages[0].shot, stages[-1].shot, total)))
  segs.append(seg("10_notice", lambda: s_notice(shot("notice"), shot("overview"))))
  segs.append(seg("11_next", lambda: s_next()))
  transitions = [("fadeblack", 0.6), ("slideleft", 0.5), ("wipeleft", 0.5), ("fade", 0.4), ("fade", 0.4),
                 ("slideleft", 0.5), ("wipeup", 0.5), ("fade", 0.3), ("fade", 0.3), ("fade", 0.3), ("wipeleft", 0.5),
                 ("fade", 0.4), ("fadeblack", 0.8)]
  master = OUT / "davids_robot_cars_showcase_master.mp4"
  assemble(segs, transitions, 1, master)
  deliver(master, OUT / "davids_robot_cars_showcase.mp4")


if __name__ == "__main__":
  main()
