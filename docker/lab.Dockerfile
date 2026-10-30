# syntax=docker/dockerfile:1.7
# Virtual-lab image (mjlab + MuJoCo Warp + rsl_rl). One Dockerfile for both machines, built natively:
#   PC   (x86_64, RTX 2060, driver 617) and GB10 (aarch64, sm_121, driver 580+): TORCH_VARIANT=cu130
# CUDA runtime libraries come inside the torch/warp wheels; the host supplies only the
# NVIDIA driver (Docker Desktop WSL2 GPU support / nvidia-container-toolkit).
FROM ubuntu:24.04

ARG TORCH_VARIANT=cu130
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      python3.12 python3.12-venv ca-certificates git ffmpeg tini libegl1 libgl1 libglib2.0-0 \
 && rm -rf /var/lib/apt/lists/*

COPY --from=ghcr.io/astral-sh/uv:0.9.5 /uv /uvx /usr/local/bin/

# Non-root user with the host's UID/GID so files written to the bind mount
# (logs/, checkpoints) stay owned by the host user on Linux (GB10).
RUN if id -u ubuntu >/dev/null 2>&1; then userdel -r ubuntu; fi \
 && groupadd -g ${USER_GID} lab \
 && useradd -m -u ${USER_UID} -g ${USER_GID} -s /bin/bash lab \
 && mkdir -p /opt/venv /workspace/lab && chown -R lab:lab /opt/venv /workspace

# The venv lives outside the bind-mounted repo, so host and container never share it.
# MUJOCO_GL=egl: headless offscreen rendering for training videos.
ENV UV_PROJECT_ENVIRONMENT=/opt/venv \
    UV_PYTHON=python3.12 \
    UV_PYTHON_DOWNLOADS=never \
    UV_LINK_MODE=copy \
    UV_COMPILE_BYTECODE=1 \
    UV_NO_SYNC=1 \
    UV_HTTP_TIMEOUT=600 \
    PATH=/opt/venv/bin:$PATH \
    PYTHONUNBUFFERED=1 \
    MPLBACKEND=Agg \
    MUJOCO_GL=egl \
    LAB_TORCH_VARIANT=${TORCH_VARIANT}

USER lab
WORKDIR /workspace/lab
COPY --chown=lab:lab lab/pyproject.toml lab/uv.lock ./
RUN --mount=type=cache,target=/home/lab/.cache/uv,uid=${USER_UID},gid=${USER_GID} \
    uv sync --locked --no-install-project --extra ${TORCH_VARIANT}

# Our package, installed editable: the .pth points at /workspace/lab/src, which the bind mount
# replaces with the live source at runtime. Rebuild only when pyproject.toml changes (entry points).
COPY --chown=lab:lab lab/src ./src
RUN --mount=type=cache,target=/home/lab/.cache/uv,uid=${USER_UID},gid=${USER_GID} \
    uv sync --locked --extra ${TORCH_VARIANT}

# Warp kernel cache: compose mounts a named volume here so compiled GPU kernels survive
# container restarts (cold compile of the MuJoCo Warp collision kernels takes minutes).
# Created in the image so the volume starts owned by `lab`.
RUN mkdir -p /home/lab/.cache/warp

WORKDIR /workspace
EXPOSE 8080 6006
ENTRYPOINT ["/usr/bin/tini", "--"]
CMD ["bash"]
