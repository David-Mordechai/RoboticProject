"""Environment check for the virtual lab: torch/CUDA/GPU info plus a bicycle-model throughput benchmark.

    python scripts/gpu_check.py [--sizes 1024 4096 16384] [--steps 200]

The benchmark steps N cars of a kinematic bicycle model (the core of the simulator) and
reports car-steps per second. Use it to pick LAB_NUM_ENVS on each machine.
"""
import argparse
import os
import platform
import time

import torch


def info() -> torch.device:
    print(f"platform      {platform.machine()} / Python {platform.python_version()}")
    print(f"torch         {torch.__version__} (variant {os.environ.get('LAB_TORCH_VARIANT', '?')}, CUDA {torch.version.cuda})")
    print(f"profile       {os.environ.get('LAB_PROFILE', '?')}, LAB_NUM_ENVS={os.environ.get('LAB_NUM_ENVS', '?')}")
    if not torch.cuda.is_available():
        print("cuda          NOT available, running on CPU")
        return torch.device("cpu")
    dev = torch.device("cuda")
    props = torch.cuda.get_device_properties(dev)
    cap = f"sm_{props.major}{props.minor}"
    archs = torch.cuda.get_arch_list()
    print(f"gpu           {props.name}, {props.total_memory / 2**30:.1f} GiB, {cap}")
    print(f"wheel archs   {' '.join(archs)}")
    if cap not in archs:
        # sm_121 (GB10) runs sm_120 binaries; anything else missing is a real problem.
        ok = cap == "sm_121" and "sm_120" in archs
        print(f"              {cap} not in wheel arch list -> {'OK (binary compatible with sm_120)' if ok else 'PROBLEM: kernels may fail'}")
    x = torch.randn(4096, 4096, device=dev)
    torch.cuda.synchronize()
    t = time.perf_counter()
    for _ in range(10):
        x @ x
    torch.cuda.synchronize()
    tflops = 10 * 2 * 4096**3 / (time.perf_counter() - t) / 1e12
    print(f"matmul fp32   {tflops:.1f} TFLOP/s")
    return dev


def bicycle_bench(dev: torch.device, n: int, steps: int, dt: float = 0.05) -> float:
    g = torch.Generator(device=dev).manual_seed(0)
    state = torch.zeros(n, 5, device=dev)  # x, y, yaw, v, steer
    wheelbase = 0.135 + 0.015 * torch.rand(n, device=dev, generator=g)
    steer_max = torch.deg2rad(25 + 17 * torch.rand(n, device=dev, generator=g))
    cmd = torch.rand(n, 2, device=dev, generator=g) * 2 - 1

    def step(s):
        v_cmd, d_cmd = cmd[:, 0] * 0.8, cmd[:, 1] * steer_max
        v = s[:, 3] + (v_cmd - s[:, 3]) * (dt / 0.15)                     # first-order speed response
        d = s[:, 4] + torch.clamp(d_cmd - s[:, 4], -6.5 * dt, 6.5 * dt)    # servo slew limit
        yaw = s[:, 2] + v / wheelbase * torch.tan(d) * dt
        return torch.stack([s[:, 0] + v * torch.cos(yaw) * dt, s[:, 1] + v * torch.sin(yaw) * dt, yaw, v, d], 1)

    for _ in range(10):
        state = step(state)
    if dev.type == "cuda":
        torch.cuda.synchronize()
    t = time.perf_counter()
    for _ in range(steps):
        state = step(state)
    if dev.type == "cuda":
        torch.cuda.synchronize()
    return n * steps / (time.perf_counter() - t)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--sizes", type=int, nargs="+", default=[1024, 4096, 16384, 65536])
    ap.add_argument("--steps", type=int, default=200)
    args = ap.parse_args()
    dev = info()
    print("bicycle model (car-steps/s, eager PyTorch, no raycast yet):")
    for n in args.sizes:
        print(f"  {n:>7} cars  {bicycle_bench(dev, n, args.steps) / 1e6:8.1f} M")
