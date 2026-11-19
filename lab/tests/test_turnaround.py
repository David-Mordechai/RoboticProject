"""Turnaround task: the sim model checked against the spec before training (CLAUDE.md), on a nominal car (physics and
actuation randomization off, no sensor noise):
- turning radius at 29 and 37 deg vs Hiwonder's R = H / tan(theta) (manual 9.2);
- the narrowest corridor for a single full-lock U-turn vs the geometric formula, and the task's split width;
- a scripted multi-point turn turns around in the narrowest corridor (W_MIN) at 29 and 37 deg, with and without
  steering at rest, where a single sweep fails; a single sweep succeeds at W_MAX;
- the low obstacles are invisible to the LiDAR and visible to the depth camera;
- the observation layout the car-side node must reproduce."""

import math

import pytest
import torch

pytestmark = pytest.mark.skipif(not torch.cuda.is_available(), reason="needs the GPU")

TASK = "Robotlab-Turnaround-MentorPiA1"
N = 256


@pytest.fixture(scope="module")
def env():
  from mjlab.envs import ManagerBasedRlEnv
  from mjlab.tasks.registry import load_env_cfg

  import robotlab.tasks  # noqa: F401

  cfg = load_env_cfg(TASK, play=True)                      # no sensor noise
  cfg.scene.num_envs = N
  cfg.episode_length_s = 1e6
  cfg.terminations = {}
  for k in ("tyre_friction", "mass_inertia", "drive_motor"):  # nominal car
    cfg.events.pop(k, None)
  d = cfg.actions["drive"]
  d.latency_range_s = d.steer_deadtime_range_s = (0.0, 0.0)
  d.steer_rate_scale_range = (1.0, 1.0)
  d.shield = None                                          # geometry of the car itself, not the v2 obstacle stop
  e = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
  e.reset()
  yield e
  e.close()


def _setup(env, width, car_xy, yaw, steer_deg, at_rest=True, length=None, kind=None, obs_xy=None, obs_yaw=None,
           sink=None):
  """All N envs: scene, then a fresh action term with the given steering limit."""
  from robotlab.tasks import mentorpi_turnaround_env_cfg as T

  ids = torch.arange(env.num_envs, device=env.device)
  length = torch.full_like(width, 4.0) if length is None else length
  T.set_scene(env, ids, width, length, car_xy, yaw, kind, obs_xy, obs_yaw, sink)
  drive = env.action_manager.get_term("drive")
  drive.reset(ids)
  drive.steer_max[:] = torch.deg2rad(steer_deg)
  drive._steer_at_rest[:] = at_rest
  env.sim.forward()                                          # fresh poses before the first step reads them
  return drive


def _act(env, v, steer):
  """Speed in m/s (forward / reverse) and steering in [-1, 1] -> policy action."""
  from robotlab.tasks.mentorpi_turnaround_env_cfg import TURN_V_MAX, TURN_V_REVERSE

  a0 = torch.where(v >= 0, v / TURN_V_MAX, v / TURN_V_REVERSE)
  return torch.stack([a0, steer], -1)


def _rear_axle(env):
  from robotlab.tasks.mdp import car_pose
  from robotlab.tasks.mentorpi_turnaround_env_cfg import AXLE_X

  xy, yaw = car_pose(env)
  return xy - AXLE_X * torch.stack([yaw.cos(), yaw.sin()], -1), yaw


def test_observation_layout(env):
  om = env.observation_manager
  names = list(om.active_terms["actor"])
  dims = [d[0] for d in om.group_obs_term_dim["actor"]]
  assert list(zip(names, dims)) == [("car_motion", 4), ("lidar_sectors", 36), ("depth", 12), ("heading_error", 2),
                                    ("last_action", 2)]


def test_turning_radius(env):
  """Steady circles at 0.2 m/s, full lock: rear-axle-centre radius vs R = H / tan(theta)."""
  from robotlab.tasks import mentorpi_turnaround_env_cfg as T

  n, dev = env.num_envs, env.device
  steer = torch.where(torch.arange(n, device=dev) % 2 == 0, 29.0, 37.0)
  side = torch.where(torch.arange(n, device=dev) % 4 < 2, 1.0, -1.0)
  _setup(env, torch.full((n,), 1.4, device=dev), torch.zeros(n, 2, device=dev), torch.zeros(n, device=dev), steer)
  ids = torch.arange(n, device=dev)
  for name in ("wall_l", "wall_r", "wall_end", "wall_back"):   # open floor
    T._put(env, name, ids, torch.tensor(T.PARK, device=dev).expand(n, 2), torch.zeros(n, device=dev))
  a = _act(env, torch.full((n,), 0.2, device=dev), side)
  pts, ang = [], []
  car = env.scene["car"]
  sid = car.find_joints(("steer_fl", "steer_fr"), preserve_order=True)[0]
  for k in range(160):                                       # 8 s; fit the last 4 s
    env.step(a)
    if k >= 80:
      pts.append(_rear_axle(env)[0])
      ang.append(car.data.joint_pos[:, sid].abs().mean(-1))
  ang = torch.rad2deg(torch.stack(ang, 1).mean(1))
  p = torch.stack(pts, 1)                                    # [N, S, 2]
  # Kasa circle fit: x^2 + y^2 + D x + E y + F = 0.
  A = torch.cat([p, torch.ones_like(p[..., :1])], -1)
  b = -(p ** 2).sum(-1, keepdim=True)
  sol = torch.linalg.lstsq(A, b).solution[..., 0]
  r = torch.sqrt((sol[:, 0] ** 2 + sol[:, 1] ** 2) / 4 - sol[:, 2])
  res = {}
  for deg in (29.0, 37.0):
    m = steer == deg
    spec = T.turning_radius(math.radians(deg))
    meas = r[m].mean().item()
    actual = ang[m].mean().item()
    res[deg] = (meas / spec - 1, actual - deg)
    print(f"\nturning radius at {deg:.0f} deg: sim {meas:.3f} m (spread {r[m].std().item():.4f}), "
          f"spec H/tan = {spec:.3f} m, {100 * (meas / spec - 1):+.1f}%; steering joints reach {actual:.1f} deg "
          f"(H/tan of that: {T.turning_radius(math.radians(actual)):.3f} m)")
  # The steering joints reach the command, but the parallel-steered front wheels (both at the same angle, research/01
  # §3) scrub: the sim circles 13-19% wider than H / tan(theta). Hiwonder's formula is an idealisation and the real
  # radius is unmeasured, so the plausible range is [formula, sim] (flagged in the task docstring).
  for err, dang in res.values():
    assert abs(dang) < 0.5 and 0.0 <= err < 0.25


def _single_sweep(env, width, steer_deg, gap=1.2):
  """Full-lock left U-turn from the best start (1 cm off the right wall, along the corridor), steering set at rest,
  0.2 m/s. Returns (turned 175 deg, touched a wall)."""
  from robotlab.tasks import mentorpi_turnaround_env_cfg as T

  n, dev = env.num_envs, env.device
  xy = torch.stack([torch.full((n,), -gap - T.BODY_HALF_L, device=dev), -width / 2 + T.BODY_HALF_W + 0.01], -1)
  _setup(env, width, xy, torch.zeros(n, device=dev), steer_deg)
  hit = torch.zeros(n, dtype=torch.bool, device=dev)
  done = torch.zeros(n, dtype=torch.bool, device=dev)
  ones = torch.ones(n, device=dev)
  for _ in range(10):                                        # steer to full lock while stopped
    env.step(_act(env, 0 * ones, ones))
  for _ in range(200):
    done |= T.heading_error(env).abs() < math.radians(5.0)
    env.step(_act(env, torch.where(done | hit, 0.0, 0.2), ones))
    hit |= T.bumped(env)
  return done & ~hit, hit


def test_single_uturn_width(env):
  """Narrowest width a single sweep clears, per steering limit, vs uturn_width_formula and SINGLE_TURN_MEASURED."""
  from robotlab.tasks import mentorpi_turnaround_env_cfg as T

  n, dev = env.num_envs, env.device
  widths = torch.arange(n // 2, device=dev) * 0.005 + 0.45            # 0.45-1.09 m, 5 mm steps
  width = torch.cat([widths, widths])
  steer = torch.cat([torch.full((n // 2,), 29.0, device=dev), torch.full((n // 2,), 37.0, device=dev)])
  ok, _ = _single_sweep(env, width, steer)
  checks = []
  for deg in (29, 37):
    m = steer == deg
    w, s = width[m], ok[m]
    fail = w[~s]
    w_meas = (fail.max().item() + 0.005) if len(fail) else w.min().item()
    f = T.uturn_width_formula(math.radians(deg))
    f_sim = T.uturn_width_formula(math.radians(deg), T.TURN_RADIUS_SIM[deg]) + 0.01   # + the 1 cm start margin
    print(f"\nsingle U-turn at {deg} deg: narrowest clear width in sim {w_meas:.3f} m (all wider widths clear: "
          f"{bool(s[w >= w_meas].all())}); formula with R = H/tan: {f:.3f} m, with the sim's measured radius "
          f"{T.TURN_RADIUS_SIM[deg]} m: {f_sim:.3f} m; task constant {T.SINGLE_TURN_MEASURED[deg]}")
    checks.append((bool(s[w >= w_meas].all()), f, f_sim, w_meas, T.SINGLE_TURN_MEASURED[deg]))
  for all_clear, f, f_sim, w_meas, const in checks:
    assert all_clear
    assert f - 0.02 <= w_meas <= f + 0.10          # Hiwonder's radius: lower bound; the sim's wider circle: above it
    assert abs(w_meas - f_sim) <= 0.02             # the sweep geometry explains the sim's width
    assert abs(w_meas - const) <= 0.02             # the task's width split uses the measured value


def test_single_sweep_fails_narrow_succeeds_wide(env):
  from robotlab.tasks import mentorpi_turnaround_env_cfg as T

  n, dev = env.num_envs, env.device
  i = torch.arange(n, device=dev)
  width = torch.where(i % 2 == 0, T.W_MIN, T.W_MAX)
  steer = torch.where(i % 4 < 2, 29.0, 37.0)
  ok, hit = _single_sweep(env, width, steer)
  assert not ok[width == T.W_MIN].any() and hit[width == T.W_MIN].all()
  assert ok[width == T.W_MAX].all()


def _predicted_clearance(env, xy, yaw, direction, steer, dist, k=6):
  """Smallest wall clearance along the next `dist` m of a full-lock arc (kinematic bicycle about the rear axle)."""
  from robotlab.robot.mentorpi_a1_constants import WHEELBASE
  from robotlab.tasks import mentorpi_turnaround_env_cfg as T

  h = torch.stack([yaw.cos(), yaw.sin()], -1)
  pr, y = xy - T.AXLE_X * h, yaw.clone()
  best = T.wall_clearance(env, xy, yaw)
  for _ in range(k):
    ds = direction * dist / k
    y = y + ds * torch.tan(steer) / WHEELBASE
    pr = pr + ds[:, None] * torch.stack([y.cos(), y.sin()], -1)
    best = torch.minimum(best, T.wall_clearance(env, pr + T.AXLE_X * torch.stack([y.cos(), y.sin()], -1), y))
  return best


def _k_turn(env, width, steer_deg, at_rest, x_gap, y_off, seconds=20.0, v=0.12, margin=0.015, look=0.06):
  """Scripted multi-point turn from ground truth: forward at full lock toward the target side, reverse at the opposite
  lock, switching when the predicted clearance along the next `look` m drops below `margin`. Stops once the heading is
  within 3 deg of the target, then holds still. Returns success flags (task definition), contact, switches, time."""
  from robotlab.tasks import mentorpi_turnaround_env_cfg as T

  n, dev = env.num_envs, env.device
  x = -x_gap - T.BODY_HALF_L
  drive = _setup(env, width, torch.stack([x, y_off], -1), torch.zeros(n, device=dev), steer_deg, at_rest)
  rot = torch.ones(n, device=dev)                            # turn left (CCW)
  d = torch.ones(n, device=dev)
  hit = torch.zeros(n, dtype=torch.bool, device=dev)
  done = torch.zeros(n, dtype=torch.bool, device=dev)
  still = torch.zeros(n, dtype=torch.long, device=dev)
  success = torch.zeros(n, dtype=torch.bool, device=dev)
  switches = torch.zeros(n, device=dev)
  t_succ = torch.full((n,), float("nan"), device=dev)
  from robotlab.tasks.mdp import car_pose
  for k in range(int(seconds / env.step_dt)):
    xy, yaw = car_pose(env)
    done |= T.heading_error(env).abs() < math.radians(3.0)
    steer = d * rot * drive.steer_max
    blocked = _predicted_clearance(env, xy, yaw, d, steer, look) < margin
    flip = blocked & ~done
    d = torch.where(flip, -d, d)
    switches += flip.float()
    speed = torch.where(done | hit, 0.0, d * v)
    env.step(_act(env, speed, d * rot))
    hit |= T.bumped(env)
    ok = (T.heading_error(env).abs() < T.SUCCESS_YAW) & (env.scene["car"].data.root_link_lin_vel_b[:, 0].abs() <
                                                        T.SUCCESS_SPEED) & (T.wall_clearance(env) >= T.SUCCESS_CLEARANCE)
    still = torch.where(ok, still + 1, 0)
    new = (still >= round(T.SUCCESS_HOLD_S / env.step_dt)) & ~success & ~hit
    t_succ = torch.where(new, (k + 1) * env.step_dt, t_succ)
    success |= new
  return success & ~hit, hit, switches, t_succ


def test_multipoint_turn_in_narrowest(env):
  """At W_MIN the scripted K-turn turns around without contact for both steering limits, steering at rest or not,
  from several start positions."""
  from robotlab.tasks import mentorpi_turnaround_env_cfg as T

  n, dev = env.num_envs, env.device
  i = torch.arange(n, device=dev)
  steer = torch.where(i % 2 == 0, 29.0, 37.0)
  at_rest = (i // 2) % 2 == 0
  x_gap = 0.3 + 1.2 * ((i // 4) % 4).float() / 3
  y_off = 0.05 * (((i // 16) % 3).float() - 1)                # -5, 0, +5 cm
  width = torch.full((n,), T.W_MIN, device=dev)
  ok, hit, sw, ts = _k_turn(env, width, steer, at_rest, x_gap, y_off)
  for deg in (29.0, 37.0):
    for r in (True, False):
      m = (steer == deg) & (at_rest == r)
      print(f"\nK-turn at W={T.W_MIN} m, {deg:.0f} deg, steer at rest {r}: success {ok[m].float().mean().item():.0%}, "
            f"contact {hit[m].float().mean().item():.0%}, switches median {sw[m].median().item():.0f} "
            f"max {sw[m].max().item():.0f}, time median {ts[m].nanmedian().item():.1f} s")
  assert ok.all()


def test_low_obstacle_invisible_to_lidar_visible_to_depth(env):
  """Pairs of envs with the same car pose, one with a chair base / low box straight ahead (top at its highest and
  lowest), one without: the LiDAR scans are identical, the depth sectors see the obstacle."""
  from robotlab.tasks import mentorpi_turnaround_env_cfg as T

  n, dev = env.num_envs, env.device
  i = torch.arange(n, device=dev)
  has = i % 2 == 1
  kind = torch.where(has, torch.where((i // 2) % 2 == 0, T.CHAIR, T.BOX), T.NONE)
  sink = torch.where((i // 4) % 2 == 0, 0.0, torch.where(kind == T.CHAIR, T.CHAIR_SINK, T.BOX_SINK))
  ext = torch.where(kind == T.CHAIR, T.CHAIR_R, T.BOX_HALF[0])
  ox = -ext - 0.1
  gap = 0.6
  car_x = ox - ext - gap - T.BODY_HALF_L
  car_x = torch.where(has, car_x, torch.roll(car_x, -1))          # the pair's partner has the same pose
  width = torch.full((n,), 1.0, device=dev)
  _setup(env, width, torch.stack([car_x, torch.zeros(n, device=dev)], -1), torch.zeros(n, device=dev),
         torch.full((n,), 33.0, device=dev), kind=kind, obs_xy=torch.stack([ox, torch.zeros(n, device=dev)], -1),
         obs_yaw=torch.where(kind == T.CHAIR, math.radians(36.0), 0.0), sink=sink)  # chair: legs at +-36 deg
  zero = torch.zeros(n, 2, device=dev)
  for _ in range(4):
    env.step(zero)
  lid = env.scene["lidar"].data.distances
  with_o, without = lid[has], lid[torch.roll(has, -1)]
  assert (with_o - without).abs().max().item() < 1e-3          # LiDAR plane passes over the obstacle
  dep = T.depth(env) * 3.0                                       # m, horizontal (camera.DEPTH_CLIP)
  near = dep[has].amin(-1)
  far = dep[torch.roll(has, -1)].amin(-1)
  expect = gap + T.BODY_HALF_L - 0.064                           # camera 0.064 m ahead of the body centre
  print(f"\ndepth: obstacle {near.min().item():.2f}-{near.max().item():.2f} m (expected ~{expect:.2f}), "
        f"without {far.min().item():.2f} m")
  assert (near < expect + 0.15).all() and (far > near + 0.3).all()
