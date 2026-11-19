# Deep research 2026-11-09, track 2: learned local planners that worked on real robots

Source: research agent report (web). "Inference" = not sourced. The BARN 2025/2026 organiser PDFs were unreadable (certificate errors).

## Systems with real-robot results
| System | Inputs | Output | Training | Safety / smoothness | Real result |
|---|---|---|---|---|---|
| LiCS, BARN 2024 winner ([2406.14947](https://arxiv.org/html/2406.14947v2)) | Raw 2D LiDAR (720 beams) + local goal from an A* path | v, w | Behaviour cloning of a classical expert. 234 worlds. Executed actions noised with sigma 0.25; the label is the clean expert action. | Hand-written geometric safety layer on hardware | Tracks 1-2: 5/5; track 3: all failed. BARN 2024: 6/9 |
| DRL-VO ([2301.06512](https://ar5iv.labs.arxiv.org/html/2301.06512)) | LiDAR history grid + sub-goal 2 m ahead on the global path | v, w | PPO in a replica of one lobby | Penalty on angular speed | 5 runs with no collisions; BARN hardware 2/9 |
| ABS, quadruped ([2401.17583](https://arxiv.org/html/2401.17583v3)) | Ray distances predicted from depth + goal | Joint targets | PPO | Learned reach-avoid value switches to a recovery policy | 9-10/10 |
| F1TENTH-class RL racing ([2504.02420](https://arxiv.org/html/2504.02420v2)) | State + track preview | Steering + integrated wheel acceleration | RL with an actuator model | Commanding wheel speed directly failed on the real car; the actuator model mattered most | 20 laps per variant |
| APPL / APPLR ([2105.07620](https://arxiv.org/abs/2105.07620)) | n/a | DWA/TEB parameters | Demonstrations / interventions / RL | Inherits the classical planner's safety | Beat the default parameters on a Jackal |

## Key findings
- **Imitating a classical expert won physically; RL alone has not.**
  - BARN 2024 was "the first physical win by end-to-end learning", and that win was by imitation.
  - Physical winners in 2022/2023 were classical, and so was the 2025 winner: rule-based, no tuning, 5/5 then 4/6 ([MTU news](https://www.mtu.edu/news/2025/07/mtu-robotics-lab-wins-international-autonomous-navigation-competition.html)).
  - Interactive imitation (DAgger) beat plain behaviour cloning on F1TENTH ([2209.15073](https://arxiv.org/abs/2209.15073)).
- **Causal confusion** (de Haan 2019) is fixed by targeted interventions. LiCS sidesteps it by executing noisy actions, so the sensor readings themselves change the labels.
- **A local goal from a global planner is standard** (LiCS, DRL-VO, [2507.04086](https://arxiv.org/html/2507.04086v1)). A LiDAR-only policy managed 40% in confined spaces. The survey (Pfeiffer): end-to-end "cannot completely replace a map-based path planner".
- **Bang-bang steering** comes from bounded Gaussian policies: RL policies drift to the action edges (Seyde 2021) and a Gaussian on a bounded range is biased (Chou 2017). Fixes:
  - a Beta or squashed policy;
  - gSDE, which removes step-wise jitter (Raffin 2021);
  - integrated (rate) actions;
  - L2C2 / LipsNet best in a real-quadruped benchmark ([2410.16632](https://arxiv.org/html/2410.16632)).
- **Sim-to-real:**
  - The actuator model matters most; full dynamics randomisation did worse.
  - Office replica: 46.8% real vs 55.9% sim with a prior map ([2011.03807](https://arxiv.org/pdf/2011.03807)).
  - Modular methods reached 90% real; end-to-end dropped from 77% sim to 23% real ([2212.00922](https://arxiv.org/abs/2212.00922)).
- **Validation practice:** BARN uses 5 trials per course on fixed physical courses, scored by successes then time, always against a classical baseline.
- **BARN 2024 organisers:**
  - "a separate safety check layer is still required";
  - "All teams adopted a hybrid paradigm ... finite-state-machine";
  - "How to remove the reliance on extensive system tuning is still an open question."
- **Ackermann dead-end escape** ([2511.22338](https://arxiv.org/html/2511.22338)): SAC got 71.9% vs TEB 37.3% and Hybrid-A* 7.2%. Simulation only, on a Hiwonder JetAcker model.

## Recipe from the report
1. Global planner -> local goal -> learned local policy -> independent geometric safety check, plus a small state machine for recovery.
2. Behaviour cloning of a classical expert that sees the same sensors as the student: noisy executed actions, clean labels, DAgger corrections.
3. Inputs: LiDAR (+ depth) and the local goal. Outputs: speed and a steering *rate* (integrated), with L2C2/CAPS. Model the actuators first.
4. Sim exam on a twin of the real site.
5. 5-10 floor trials per course per version, always with a classical baseline on the same courses.
