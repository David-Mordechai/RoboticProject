"""Cast the LiDAR rays every second policy step instead of every step.

The real MS200 scans at 10.0-10.1 Hz (docs/measurements/car1-2026-11-02.md §1) and the policy runs at 20 Hz, so on
every other policy step the car still holds the previous scan. The sim used to cast all 360 rays every step and then
throw half the scans away (randomization.noisy_rays held scans at random). Ray casting was 43% of the GPU time per
step on the GB10 (profile 2026-11-02), most of it the LiDAR, so casting at the real rate saves that work with no loss
of fidelity.

How: a startup event wraps env.sim.sense. On odd steps it runs mjlab's full sense graph (BVH refit, depth fan and
LiDAR); on even steps a second CUDA graph that refits the BVH and casts every other ray sensor but not the LiDAR. The
LiDAR keeps its last distances, and env._robotlab_lidar_cast tells randomization.noisy_rays whether the scan is new.
The second graph is re-captured whenever mjlab re-captures its own (create_graph after model field expansion).
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import mujoco_warp as mjwarp
import torch
import warp as wp

from mjlab.sim.sim import _suspend_gc

if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv


def cast_lidar_every_other_step(env: ManagerBasedRlEnv, env_ids: torch.Tensor | slice | None,
                                sensor_name: str = "lidar") -> None:
  sim = env.sim
  if getattr(sim, "_robotlab_lidar_skip", False):
    return
  sim._robotlab_lidar_skip = True
  ctx = sim._sensor_context
  lidar = env.scene[sensor_name]
  others = [s for s in ctx.raycast_sensors if s is not lidar]
  full_sense = sim.sense
  state = {"graph": None, "captured_for": None, "tick": 0}

  def sense_without_lidar() -> None:
    rc = ctx.render_context
    mjwarp.refit_bvh(sim.wp_model, sim.wp_data, rc)
    if ctx.has_cameras:
      mjwarp.render(sim.wp_model, sim.wp_data, rc)
      ctx.unpack_rgb()
    for s in others:
      s.raycast_kernel(rc=rc)

  def sense() -> None:
    state["tick"] += 1
    env._robotlab_lidar_cast = state["tick"] % 2 == 1
    if env._robotlab_lidar_cast:
      full_sense()
      return
    ctx.prepare()
    with wp.ScopedDevice(sim.wp_device):
      if sim.use_cuda_graph and sim.sense_graph is not None:
        if state["graph"] is None or state["captured_for"] is not sim.sense_graph:
          with _suspend_gc(), wp.ScopedCapture() as capture:
            sense_without_lidar()
          state["graph"], state["captured_for"] = capture.graph, sim.sense_graph
        wp.capture_launch(state["graph"])
      else:
        sense_without_lidar()
    ctx.finalize()

  sim.sense = sense
