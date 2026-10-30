# Virtual lab Docker environment

*2026-10-30. One Docker setup runs the lab on this PC now and on the ASUS GB10 later. Verified on this PC: build, GPU check, tests and TensorBoard all pass.*

## Design

| Choice | Why |
|---|---|
| **One Dockerfile** (`docker/lab.Dockerfile`), built **natively on each machine** | The base image `ubuntu:24.04` is multi-arch. The GB10 builds its own aarch64 image, so no cross-building or registry is needed |
| **CUDA comes from the torch wheel, not the base image** | The host supplies only the NVIDIA driver. This avoids 10+ GB CUDA/NGC base images and driver-version coupling. NGC images also need newer drivers than this PC has |
| **Machine profile via `.env`** (`docker/env/pc.env` or `gb10.env`) | Picks the torch wheel (`TORCH_VARIANT`), the default batch size, shared memory and UID/GID. Everything else is identical |
| **Same torch version everywhere: 2.11.0**, as cu128 on the PC and cu130 on the GB10 | 2.11 is the last torch with cu128 wheels. The PC's driver (576) can't run cu130, which needs driver ≥ 580. One version keeps results comparable across machines |
| **uv + `uv.lock`**, locked for Linux x86_64 **and** aarch64 | Reproducible installs. The lock was generated inside a Linux container and covers both architectures |
| **The repo is bind-mounted** at `/workspace`; the **venv is baked into the image** at `/opt/venv` | Edit code on the host and run it in the container right away. Host and container never share a venv |
| **Non-root user** with the host UID/GID | On the GB10 (Linux), checkpoints and logs in `runs/` stay owned by you, not root |
| **Small build context** (`.dockerignore` allows only `pyproject.toml` + `uv.lock`) | Vendor PDFs and runs don't get sent to the builder; rebuilds happen only when dependencies change |
| **LF line endings** (`.gitattributes`) | Files are executed inside Linux containers; CRLF from a Windows checkout would break scripts |
| **TensorBoard** as a second compose service, bound to `127.0.0.1:6006` | Nothing is exposed beyond the machine (plan §4) |

### Files

```
compose.yaml              services: lab (dev shell / runs), tensorboard
docker/lab.Dockerfile     the image
docker/env/pc.env         TORCH_VARIANT=cu128, LAB_NUM_ENVS=2048,  SHM 4gb
docker/env/gb10.env       TORCH_VARIANT=cu130, LAB_NUM_ENVS=32768, SHM 32gb
.env                      (git-ignored) copy of one of the above
lab/pyproject.toml        dependencies; torch variant chosen by extra (cpu / cu128 / cu130)
lab/uv.lock               the lock
lab/scripts/gpu_check.py  torch/GPU info + bicycle-model throughput benchmark
lab/tests/                smoke tests (torch CPU/GPU, ONNX export parity)
runs/                     (git-ignored) checkpoints, logs, videos
```

## Usage

```bash
cp docker/env/pc.env .env           # on the GB10: cp docker/env/gb10.env .env (and set USER_UID/GID)
docker compose build                # first build: ~5 min, image ~12.6 GB (mostly torch + CUDA libs)
docker compose run --rm lab python scripts/gpu_check.py
docker compose run --rm lab pytest
docker compose run --rm lab         # interactive shell in /workspace/lab
docker compose up -d tensorboard    # http://localhost:6006 ; stop with: docker compose down
```

**Adding a dependency:** edit `lab/pyproject.toml`, re-lock inside Linux, then rebuild.

```bash
docker run --rm -v "$PWD/lab:/w" -w /w ghcr.io/astral-sh/uv:0.9.5-python3.12-bookworm-slim uv lock
docker compose build
```

Don't run `uv sync` inside the container. `UV_NO_SYNC=1` is set so that `uv run` can't swap the torch variant.

## Measured on this PC (2026-10-30)

| Item | Value |
|---|---|
| GPU | RTX 2060 6 GB, sm_75 (in the wheel's arch list) |
| Software | torch 2.11.0+cu128, CUDA 12.8, Python 3.12.3 |
| matmul fp32 | 3.9 TFLOP/s |
| Bicycle model, eager PyTorch, car-steps/s | 1k cars: 2.7 M · 4k: 10.2 M · 16k: 36.7 M · 65k: 184 M |
| Tests | 3 passed (CPU, CUDA vs CPU, ONNX export parity) |

The small batches are bound by kernel-launch overhead, so throughput grows almost linearly with batch size. The real step function adds raycasting and the policy, so the realistic training batch here is a few thousand envs. Re-run `gpu_check.py` after the raycaster lands and set `LAB_NUM_ENVS` from that.

## On the GB10

Prerequisites: Docker with the NVIDIA Container Toolkit. DGX OS ships both; check with `docker info | grep -i nvidia`. You also need driver ≥ 580.

1. `git clone` the repo, then `cp docker/env/gb10.env .env`, and set `USER_UID`/`USER_GID` to the output of `id -u` and `id -g`.
2. `docker compose build`. This builds the aarch64 image with torch 2.11.0+cu130 from the same lock.
3. Run `docker compose run --rm lab python scripts/gpu_check.py`.
   - **Expect `sm_121`.**
   - If the script reports it as missing but `sm_120` is present, that's fine: they're binary compatible (plan §3).
   - If neither is present, raise the torch pin (cu130 wheels exist up to 2.14). The PC must then move to cu130 too (driver ≥ 580) to keep versions equal.
4. Run `pytest`, then re-run the latest training config with the same seed.

## Later: the ROS 2 side (Phase 0–2)

The car link (ROS 2 Humble, MCP server, bag recording) will be a **second image** (`ros:humble-ros-base`, also multi-arch) in the same `compose.yaml`, using `network_mode: host` for DDS. The training image stays free of ROS; the two share data through `runs/` and recorded bags. On this PC, DDS to the cars through Docker Desktop needs host networking (Docker Desktop ≥ 4.34) or WSL mirrored networking. Test that once the cars are on the network.
