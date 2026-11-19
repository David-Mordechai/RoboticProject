#!/bin/bash
# Phase A trial to one goal (map frame of goto6), hard limit 120 s, recorded with rec_run.sh.
#   ./nav_a_trial.sh NAME X Y YAW_DEG
# Courses (docs/research/deep-2026-11-09/00-synthesis.md, Gate A): C1 door 2.95 -2.3 -90; C2 hall end 2.95 -7.0 -90;
# C3 charger: 0.3 0 180 (the dock itself is lethal in the map; nav_trip ends 0.3 m out, the car is pushed in by hand).
~/rec_run.sh $1 "timeout 120 python3 nav/nav_trip.py $2 $3 $4"
