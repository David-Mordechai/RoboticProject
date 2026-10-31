"""Domain randomization reaches the sim: per-car physics values differ, commands arrive late, the actor's
observations are noisy while the critic's stay true, and the randomized car still drives."""

import pytest
import torch

pytestmark = pytest.mark.skipif(not torch.cuda.is_available(), reason="needs the GPU")

TASK = "Robotlab-Office-MentorPiA1"


@pytest.fixture(scope="module")
def env():
  from mjlab.envs import ManagerBasedRlEnv
  from mjlab.tasks.registry import load_env_cfg

  import robotlab.tasks  # noqa: F401

  cfg = load_env_cfg(TASK)
  cfg.scene.num_envs = 64
  cfg.terminations = {k: v for k, v in cfg.terminations.items() if k == "time_out"}
  e = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
  e.reset()
  yield e
  e.close()


def _spread(x: torch.Tensor) -> float:
  return (x.max() - x.min()).item()


def test_physics_values_differ_per_car(env):
  from robotlab.tasks import randomization as R

  m, car = env.sim.model, env.scene["car"]
  tyre = car.indexing.geom_ids[car.find_geoms("tyre_.*")[0]]
  mu = m.geom_friction[:, tyre, 0]
  assert _spread(mu) > 0.2
  assert torch.allclose(mu, mu[:, :1].expand_as(mu)), "all four tyres of a car share one friction"
  assert mu.min() >= R.TYRE_FRICTION_RANGE[0] - 1e-6 and mu.max() <= R.TYRE_FRICTION_RANGE[1] + 1e-6

  chassis = car.indexing.body_ids[car.find_bodies("chassis")[0]]
  mass = m.body_mass[:, chassis].squeeze(-1)
  assert _spread(mass) > 0.0 and (mass / 1.0).min() > 0.949 and (mass / 1.0).max() < 1.051

  ids = torch.cat([a.global_ctrl_ids for a in car.actuators if "wheel_rl" in a.target_names])
  torque = m.actuator_forcerange[:, ids, 1].squeeze(-1)
  assert torque.min() >= 0.3 * R.DRIVE_TORQUE_SCALE_RANGE[0] - 1e-6
  assert torque.max() <= 0.3 * R.DRIVE_TORQUE_SCALE_RANGE[1] + 1e-6
  assert _spread(torque) > 0.1


def test_commands_arrive_late(env):
  from robotlab.tasks import randomization as R

  drive = env.action_manager.get_term("drive")
  env.reset()                                   # draws each car's delays; read them after this
  lag_s = drive._lag.float() * env.physics_dt
  steer_lag_s = drive._steer_lag.float() * env.physics_dt
  assert lag_s.min() >= 0 and lag_s.max() <= R.LATENCY_RANGE_S[1] + 1e-6
  assert (steer_lag_s >= lag_s).all()
  assert _spread(lag_s) > 0.0

  # Full left lock from rest: after one policy step (50 ms) the servo of a slow-reacting car has moved less.
  a = torch.zeros(env.num_envs, 2, device=env.device)
  a[:, 1] = 1.0
  env.step(a)
  moved = drive._steer_now[:, 0]
  slow, fast = steer_lag_s > 0.03, steer_lag_s < 0.01
  assert slow.any() and fast.any()
  assert moved[slow].mean() < moved[fast].mean()


def test_actor_noisy_critic_clean(env):
  obs = env.observation_manager.compute()
  om = env.observation_manager
  names = om.active_terms["actor"]
  dims = om.group_obs_term_dim["actor"]
  i = names.index("lidar")
  start = sum(d[0] for d in dims[:i])
  lidar_actor = obs["actor"][:, start:start + dims[i][0]]
  lidar_critic = obs["critic"][:, start:start + dims[i][0]]
  assert lidar_critic.min() >= 0 and lidar_critic.max() <= 1
  assert not torch.allclose(lidar_actor, lidar_critic)


def test_randomized_car_still_drives(env):
  """Half throttle straight ahead for 2 s: the cars, whatever their draw, reach a sensible speed."""
  from robotlab.robot.mentorpi_a1_constants import V_MAX

  env.reset()
  a = torch.zeros(env.num_envs, 2, device=env.device)
  a[:, 0] = 0.5
  for _ in range(40):
    env.step(a)
  v = env.scene["car"].data.root_link_lin_vel_b[:, 0]
  # A few cars may have met a wall after 0.7 m; the rest must be near the commanded 0.35 m/s.
  ok = (v > 0.5 * 0.5 * V_MAX) & (v < 1.2 * 0.5 * V_MAX)
  assert ok.float().mean() > 0.9, f"speeds {v.min():.2f}-{v.max():.2f} m/s"
