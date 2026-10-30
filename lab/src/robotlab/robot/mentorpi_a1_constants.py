"""Hiwonder MentorPi A1: model files, geometry and limits (each value cites its source), entity configs."""

from __future__ import annotations

import math
from pathlib import Path

import mujoco

from mjlab.actuator.xml_actuator import XmlActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg

_ROBOT_DIR = Path(__file__).resolve().parent / "mentorpi_a1"
MENTORPI_A1_XML = _ROBOT_DIR / "mentorpi_a1.xml"
LEADER_CAR_XML = _ROBOT_DIR / "leader_car.xml"

# Geometry and limits (docs/research/01-mentorpi-a1-hardware.md, docs/vendor/hiwonder/manuals/).
WHEEL_RADIUS = 0.0325            # Ø65 mm (Hiwonder drawing); Hiwonder code uses 67 mm: conflict, measure
WHEELBASE = 0.145                # H, Hiwonder ackermann.py
TRACK = 0.133                    # D, Hiwonder ackermann.py
STEER_MAX = math.radians(35.0)   # nominal, for normalizing; per-car limits in mdp.AckermannActionCfg
STEER_2024 = math.radians(29.0)  # Hiwonder: 2024 build
STEER_CODE = math.radians(37.0)  # Hiwonder ackermann.py stops the motors above this
V_MAX = 0.7                      # m/s: Hiwonder stock joystick limit (research/01 §3); motor no-load ~1.5 m/s (calc.)
STEER_RATE = math.radians(60.0) / 0.16   # rad/s: LD-1501MG servo speed 0.16 s/60 deg (research/01 §4)


# Entities.


def car_cfg() -> EntityCfg:
  return EntityCfg(
    spec_fn=lambda: mujoco.MjSpec.from_file(str(MENTORPI_A1_XML)),
    articulation=EntityArticulationInfoCfg(
      actuators=(
        XmlActuatorCfg(target_names_expr=("steer_.*",)),
        XmlActuatorCfg(target_names_expr=("wheel_rl", "wheel_rr")),
      ),
    ),
    init_state=EntityCfg.InitialStateCfg(pos=(0.0, 0.0, WHEEL_RADIUS + 0.002)),
  )


def leader_cfg() -> EntityCfg:
  return EntityCfg(spec_fn=lambda: mujoco.MjSpec.from_file(str(LEADER_CAR_XML)))
