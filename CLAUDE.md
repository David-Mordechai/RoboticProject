# RoboticProject

Two **Hiwonder MentorPi A1 Advanced** robot cars (Ackermann-style steering, regular wheels, no strafing; Raspberry Pi 5 8GB; RRC Lite controller; 2D LiDAR; HP60C depth camera; WonderEcho Pro). The goal is to train them to drive as a team (formations, obstacle avoidance) using RL in a GPU simulator on the ASUS GB10, with Claude Code controlling the cars through a local MCP server. Later goal: building-scale operation over a mesh network.

- **The plan:** `docs/project-plan.md`. Read it first; it holds decisions, safety rules, phases and open items.
- Virtual lab (sim + training) setup plan, written to start before the GB10 arrives: `docs/virtual-lab-plan.md`.
- Lab runs **only in Docker** (same image on this PC and the GB10): `docs/docker-env.md`. Quick start: `cp docker/env/pc.env .env && docker compose build && docker compose run --rm lab pytest`. Don't install Python deps on the host; change `lab/pyproject.toml`, re-lock inside Linux, rebuild.
- Hardware and software research, and the findings that affect the plan: `docs/research/00-overview.md`, then `01-mentorpi-a1-hardware.md` and `02-mentorpi-a1-software.md`.
- Vendor docs (Hiwonder manuals, RRC Lite lessons, datasheets, product images): `docs/vendor/`. Hiwonder code has no license: read and run it, but don't copy it into the repo. Clone it into the git-ignored `docs/vendor/hiwonder/code/`.
- Dimensions: Hiwonder's drawings and code disagree on wheel size, wheelbase, track and steering angle. Mark unmeasured values as estimates until they're measured on the real cars.
