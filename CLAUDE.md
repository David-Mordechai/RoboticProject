# RoboticProject

Two **Hiwonder MentorPi A1 Advanced** robot cars (Ackermann-style steering, regular wheels, no strafing; Raspberry Pi 5 8GB; RRC Lite controller; 2D LiDAR; HP60C depth camera; WonderEcho Pro). The goal is to train them to drive as a team (formations, obstacle avoidance) using RL in a GPU simulator on the ASUS GB10, with Claude Code controlling the cars through a local MCP server. Later goal: building-scale operation over a mesh network.

## Hard rule: check the spec, never assume

Every physical or technical value (dimensions, angles, speeds, torques, limits, sensor specs, protocols, formulas, how a mechanism behaves) must come from a source. Before you use a value, look it up, in this order:

1. Measurements on the real cars, once they exist.
2. Vendor documentation in `docs/vendor/`: Hiwonder manuals, RRC Lite lessons and datasheets.
3. Our research notes, `docs/research/`, which cite their sources.
4. Official upstream docs for software, such as MuJoCo, mjlab and rsl_rl. This includes their units and conventions; for example, MJCF angles are in degrees by default.

Rules for using values:
- **Cite the source** next to the value in code or docs, for example `# Hiwonder manual 9.2`.
- **Missing values:** if no source gives the value, say so explicitly and don't invent it. Either derive it with a documented formula and label it *estimate*, or cover the range of possibilities. For example, randomize between the 29° and 37° steering limits until the build is measured.
- **Conflicting sources:** use the plausible range and flag the conflict.
- **Check the model against the spec.** After building or changing a sim model, measure its behaviour in sim and compare it with the spec before training on it. For example, compare the turning radius with Hiwonder's R = H / tan θ.

- **The plan:** `docs/project-plan.md`. Read it first; it holds decisions, safety rules, phases and open items.
- **Next steps:** `docs/open-issues.md`, in priority order. Start new work from there.
- Virtual lab (sim + training) setup plan, written to start before the GB10 arrives: `docs/virtual-lab-plan.md`.
- Lab runs **only in Docker** (same image on this PC and the GB10): `docs/docker-env.md`. Quick start: `cp docker/env/pc.env .env && docker compose build && docker compose run --rm lab pytest`. Don't install Python deps on the host; change `lab/pyproject.toml`, re-lock inside Linux, rebuild.
- Hardware and software research, and the findings that affect the plan: `docs/research/00-overview.md`, then `01-mentorpi-a1-hardware.md`, `02-mentorpi-a1-software.md` and `03-sensors-mapping-multirobot.md` (sensors, camera, shared map and relative pose).
- Vendor docs (Hiwonder manuals, RRC Lite lessons, datasheets, product images): `docs/vendor/`. Hiwonder code has no license: read and run it, but don't copy it into the repo. Clone it into the git-ignored `docs/vendor/hiwonder/code/`.
- Dimensions: Hiwonder's drawings and code disagree on wheel size, wheelbase, track and steering angle. Mark unmeasured values as estimates until they're measured on the real cars.
