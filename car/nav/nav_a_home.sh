#!/bin/bash
# Phase A parking: drive home to the pre-dock pose IN FRONT of the charging spot FACING OUT (0.35, -0.03, -8 deg = how
# the car stood docked, scan_map_match.py), then reverse straight in with dock.py (rear LiDAR stop ~4 cm).
# 2026-11-11: two legs (home_legs.sh) because car #1 steers only ~28 L / ~24 R; one goal with the U-turn next to the
# dock shuffled (bench 10/14 at 24-28 deg; floor 11-09: 58 s of shuffling). Hard limit 180 s. Recorded with rec_run.sh.
#   ./nav_a_home.sh NAME
N=${1:-a_home}
~/rec_run.sh $N "timeout 180 bash nav/home_legs.sh"
