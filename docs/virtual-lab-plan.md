# Virtual lab: setup plan (start without the GB10)

*2026-10-30. Implements [project-plan.md](project-plan.md) §7 (simulation and training) and Phases 3–4. Written for the time before the GB10 arrives.*

## Why we don't need to wait for the GB10

The plan's simulator is a **batched kinematic bicycle model in plain PyTorch**. Per car per step that's a few dozen floating-point operations, plus a LiDAR raycast, and the policy is a small MLP. The GB10 lets us run tens of thousands of car pairs per step. A laptop GPU runs a few thousand: same code, same results, a longer wall-clock time. Everything we build now runs unchanged on the GB10. Only the batch size and the PyTorch wheel differ.

**This dev PC:** RTX 2060 6 GB (Turing, sm_75), i7-1165G7 (4 cores), 64 GB RAM, Windows 11 with WSL2 and Docker Desktop.

| Work | This PC | GB10 |
|---|---|---|
| Sim core, raycasting, reward, randomisation, tests | ✅ full speed (it's small) | same |
| MAPPO training, leader-follower and formations | ✅ ~2–4k envs; minutes to tens of minutes per run (*estimate*) | 30–60k envs; large sweeps |
| Hyperparameter sweeps, many seeds | ⚠️ slow; run overnight or a few at a time | ✅ |
| ONNX export + CPU inference check | ✅ | ✅ |
| ROS 2 Humble (Docker) to talk to the cars | ✅ (WSL2 mirrored networking) | ✅ |
| Isaac Lab / MuJoCo (optional, later) | ❌ 6 GB is too small | ✅ |

## Portability rules (so moving to the GB10 is just a copy)

1. **Everything runs in Docker.** One Ubuntu 24.04 image, built natively on each machine, the same base as DGX OS on the GB10. See [docker-env.md](docker-env.md). No native Windows Python, no WSL setup.
2. **Pure PyTorch.** No custom CUDA kernels, no JAX (CPU-only on aarch64), no x86-only wheels. `torch.compile` is optional and must never be required.
3. **Pick the device at runtime.** Use `--device cuda|cpu` and `--num-envs` from the config; nothing hard-coded. The run config records the device, the torch/CUDA versions and the git commit.
4. **One project, per-machine torch wheel, same torch version (2.11.0).** `uv` lockfile; the Docker build arg `TORCH_VARIANT` selects the wheel:

   | Machine | Architecture | PyTorch wheel |
   |---|---|---|
   | This PC | x86_64 | **cu128** (driver 576.x supports CUDA 12.9; cu130 needs driver ≥ 580) |
   | GB10 | aarch64 | **cu130**, from PyTorch's index |

5. **Deterministic and seeded.** Same seed and same device give the same rollouts; this is a test. Checkpoints are plain `state_dict` files plus a JSON config, readable on any machine.
6. **CPU test suite** (pytest), so CI and quick checks run without a GPU.

## Repo layout (new)

```
compose.yaml, docker/       # see docker-env.md
lab/
  pyproject.toml, uv.lock
  scripts/gpu_check.py
  lab/
    params.py        # CarParams: per-env tensors (wheelbase, steer max, servo lag, delays, ...)
    dynamics.py      # batched bicycle model + actuator models (servo rate/lag, speed response, cmd delay)
    world.py         # arenas: walls/obstacles as segments; procedural generator; collision checks
    lidar.py         # batched ray-segment raycast -> downsampled ranges (matches the real LiDAR config)
    comms.py         # teammate-pose channel: delay, drop, jitter (plan §7 "most important")
    formation_env.py # 2-car env: observations, actions, reward, reset, termination
    leaders.py       # scripted leader paths (for leader-follower stage + classical baseline)
    mappo/           # our MAPPO: actor, centralised critic, GAE, PPO update, rollout buffer
    baseline.py      # classical baseline: path-following leader + PID follower (plan §8)
    export.py        # ONNX export + parity check vs PyTorch
    sysid.py         # fit CarParams to recorded real runs (gradient-based; the sim is differentiable)
    replay.py        # replay recorded real commands in sim; compare trajectories
    render.py        # top-down frames -> MP4/GIF; plots
  configs/           # yaml: sim params ranges, task, training, randomisation
  tests/
runs/                # git-ignored (repo root): checkpoints, logs, videos
```

## Interface contract (sim ↔ car)

Define this first. The sim env and the future ROS 2 policy node both implement it, so the policy can't tell them apart.

| | Content | Units / notes |
|---|---|---|
| **Action** (per car, 20 Hz) | target speed v, target steering angle δ | m/s, rad; clipped to per-car limits. Maps directly to RRC motor rev/s + servo pulse ([research F2](research/00-overview.md)), not to `cmd_vel` |
| **Observation** (per car) | own v and δ (measured/estimated) · pose error to the formation slot in the car frame (dx, dy, dψ) · teammate relative pose and velocity (delayed, may be stale) + age of that message · formation offset (target slot) · N downsampled LiDAR ranges · *later:* link quality | Everything in the car frame, normalised; the spec is versioned (`obs_v1`) |
| **Control rate** | 20 Hz in sim = 20 Hz on the Pi | Command delay is modelled explicitly |

## Car model: start from research values, randomise widely, then narrow down

We don't have measured data yet. Start with **wide ranges around the documented values**. Training against wide ranges is valid in its own right: the resulting policy is robust to parameters we haven't measured yet. When Phase 2 data arrives, `sysid.py` fits the real values and the ranges narrow.

| Parameter | Initial range (*estimates*) | Source |
|---|---|---|
| Wheelbase L | 0.135–0.150 m | Code 0.145 / URDF 0.137 |
| Max steering δmax | 25–42° | 29° (2024) / 40° (2025) builds |
| Servo slew rate | 200–600 °/s | LD-1501MG 0.16 s/60° ≈ 375 °/s no-load |
| Servo / command latency | 30–150 ms | Wi-Fi + serial + servo; *measure* |
| Max speed | 0.3–1.0 m/s (training cap, not the hardware limit) | Joystick default 0.7 m/s; safety caps |
| Speed response | First-order τ 0.05–0.3 s, accel limit 0.5–3 m/s² | Speed PID on the STM32; *measure* |
| Steering bias (trim error) | ±3° | Servo trim varies per car |
| Teammate pose channel | Delay 20–300 ms, drop 0–30 %, pose noise σ 1–5 cm / 1–3° + rare jumps | Plan §7 |
| LiDAR | 360°, ~0.8–1° resolution, 0.05–12 m, noise σ 1–2 cm, dropouts | MS200/LD19 specs |
| Footprint | 0.213 × 0.159 m box | Hiwonder drawing |

## Build order and exit tests

| Step | Deliverable | Done when |
|---|---|---|
| **L0** Environment | Docker image + compose, uv lock (x86_64 + aarch64), GPU check, smoke tests | ✅ **Done 2026-10-30**: tests green, sm_75 detected, benchmark recorded ([docker-env.md](docker-env.md)) |
| **L1** Dynamics | Batched bicycle + actuator models, per-env params | Unit tests: a circle at constant δ gives radius L/tan δ; straight-line speed response matches the first-order model; CPU and GPU agree; ≥1M car-steps/s on the 2060 (*target*) |
| **L2** World + LiDAR | Arenas (rooms, corridors, random boxes), collisions, raycast | Raycast matches a brute-force reference; collisions detected for both cars and walls |
| **L3** Renderer | Top-down video of any env index; trajectory plots | MP4/GIF of a scripted run |
| **L4** Classical baseline | Scripted leader + PID follower holding an offset, with comm delay | Baseline metrics recorded: slot error, collisions. This is the bar RL must beat |
| **L5** MAPPO | Actor (shared weights, car-ID input), centralised critic, GAE, PPO | **Leader-follower** (scripted leader) converges; beats the baseline on slot error under delay and noise |
| **L6** Formations + obstacles | Formation offset as input, switching mid-episode, obstacles, full reward from plan §7 | One policy holds column, side-by-side and diagonal, and switches on command; collision rate below target |
| **L7** Both cars learn | Both cars run the policy (no scripted leader); curriculum | Stable formation driving with a route-progress reward |
| **L8** Export | ONNX export, parity test (max abs diff < 1e-4), CPU latency benchmark | Inference ≪ 50 ms on CPU (Pi 5 check later) |
| **L9** Real-data tools | `replay.py` + `sysid.py`, using synthetic "real" data from a sim with hidden params | sysid recovers the hidden params within tolerance. Ready for Phase 2 recordings |

Each step ends with the plan §8 report: plots, sim videos, metrics. All runs are logged to TensorBoard with their config and commit.

## What runs in parallel with the cars (no GB10 needed)

Phases 0–2 of the main plan need ROS 2 and network access, not a big GPU. This PC can do them:
- Car bring-up and security fixes (research F6, F7, F9).
- The MCP server and safety node.
- Calibration and data collection.

So the order is: data collection on the real cars feeds L9, sysid narrows the sim ranges, and we retrain.

## Moving to the GB10 when it arrives

1. Clone the repo, `cp docker/env/gb10.env .env`, `docker compose build`, then run `gpu_check.py` and `pytest` ([docker-env.md](docker-env.md#on-the-gb10)).
2. Re-run the L1 throughput benchmark and pick `num_envs`.
3. Re-run the latest config with the same seed. Results should match statistically, not bit-for-bit across GPUs.
4. Move sweeps and long runs there. This PC stays the dev and car-ops machine.

## Decisions taken here (change if you disagree)

- **Our own MAPPO implementation** (~500 lines of PyTorch), not TorchRL or another framework. It's transparent, easy to debug, and has no aarch64 wheel risk.
- **Docker for everything** (one image on both machines) rather than native installs.
- **Steering-angle actions** (v, δ), not (v, ω), matching the car's real actuator.
