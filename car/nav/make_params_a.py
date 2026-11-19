"""Phase A Nav2 parameters (2026-11-09): the classical baseline, configured from docs/research/deep-2026-11-09.

Standalone (does not read Hiwonder's stock nav2_params.yaml, whose A1 config is not a car config: NavFn, Spin,
robot_radius 0.05 [01-nav2-ackermann-pi.md]). ROS 2 Humble parameter names. Writes nav2_params_a.yaml next to this file.

Fixes against our 11-04 config (car/nav/make_params.py), each verified in the Nav2 Humble source (01-*.md):
  - robot_radius 0.14 -> polygon footprint: with a radius Smac / MPPI check only the centre cell against the
    inscribed cost, so any obstacle within 0.14 m gave "Starting point in lethal space";
  - one turning radius for planner and controller;
  - no Spin in the behaviour tree; bond_timeout 0 (lifecycle bonds caused the idle CPU load, nav2 #5784).

  python car/nav/make_params_a.py [--footprint-rear 0.1065]
"""
import argparse
import os

import yaml

p = argparse.ArgumentParser()
# Car body 0.213 x 0.159 m (research/01 sec. 2, Hiwonder drawing). base_footprint sits under base_link (TF on car #1,
# 2026-11-09: base_footprint -> base_link = (0, 0, 0.07)), midway between the axles (ack.xacro: front wheels x +0.0668,
# rear x -0.0703). The base_link collision mesh (meshes/acker/base_link.STL) spans x -0.1038 .. +0.1096 (length 0.2134 =
# the drawing's 0.213), so 0.104 m of body is behind base_footprint. Measured from the vendor model, not a tape.
p.add_argument("--footprint-rear", type=float, default=0.104, help="m of body behind base_footprint (URDF mesh)")
p.add_argument("--controller", choices=("rpp", "mppi"), default="rpp")
a = p.parse_args()

L, W, MARGIN = 0.213, 0.159, 0.01           # m: body (research/01), + 1 cm (01-nav2 report)
rear, front, half = a.footprint_rear + MARGIN, L - a.footprint_rear + MARGIN, W / 2 + MARGIN
FOOTPRINT = f"[[{front:.3f}, {half:.3f}], [{front:.3f}, {-half:.3f}], [{-rear:.3f}, {-half:.3f}], [{-rear:.3f}, {half:.3f}]]"
TURN_R = 0.40        # m: Smac README default; above R = 0.145 / tan 29 deg = 0.26 m (Hiwonder formula, research/01);
                     # a 1.5 m hall fits a U-turn (2 x 0.4 + 0.16 = 0.96 m). Same value for planner and controller.
# m/s. 0.35 on 2026-11-09 was "too slow" (user). Bench night 2026-11-09 (3 courses x 29/33/40 deg x 3): 0.5 m/s WITH the
# smoother's scale_velocities = 27/27, 0 contacts, ~30 % faster (C1 12.1 s, C2 10.9 s, C3 32.3 s); 0.5 WITHOUT it: C3
# 7/8 (one stuck at the desk legs), full lock 5.6 vs 2.2 %. v6p drove up to 0.7 on the floor. Reverse stays 0.15.
V_MAX, V_REV = 0.5, 0.15
HERE = os.path.dirname(os.path.abspath(__file__))
CAR_NAV = "/home/ubuntu/shared/nav"   # where the files live inside the car's MentorPi container

# Obstacle sources: LiDAR /scan (360 deg, plane ~0.16 m) and the depth camera's below-LiDAR band as a LaserScan
# (/depth_scan from car/perception/depth_scan.py). STVL on the depth point cloud is the research recommendation
# (03-perception); it is added once the camera's point-cloud topic and the Pi CPU are checked (step A3).
# LiDAR topic: /scan_raw. The stock lidar.launch.py has its laser_filters node (scan_raw -> scan) commented out, so
# no /scan exists on car #1 (ros2 topic list, 2026-11-09).
scan_src = {"topic": "/scan_raw", "data_type": "LaserScan", "marking": True, "clearing": True,
            "obstacle_max_range": 2.5, "obstacle_min_range": 0.0, "raytrace_max_range": 3.0, "raytrace_min_range": 0.0,
            "max_obstacle_height": 2.0, "min_obstacle_height": 0.0, "inf_is_valid": False}
depth_src = dict(scan_src, topic="/depth_scan", obstacle_max_range=2.0, raytrace_max_range=2.5)


def costmap(name, rolling, size, freq, infl, scale, layers):
  c = {"update_frequency": freq, "publish_frequency": freq / 2, "global_frame": "odom" if rolling else "map",
       "robot_base_frame": "base_footprint", "use_sim_time": False, "rolling_window": rolling,
       "resolution": 0.05, "footprint": FOOTPRINT, "footprint_padding": 0.0, "track_unknown_space": True,
       "always_send_full_costmap": True, "plugins": layers,
       "inflation_layer": {"plugin": "nav2_costmap_2d::InflationLayer", "inflation_radius": infl,
                           "cost_scaling_factor": scale},
       "obstacle_layer": {"plugin": "nav2_costmap_2d::ObstacleLayer", "enabled": True,
                          "observation_sources": "scan depth_scan", "scan": scan_src, "depth_scan": depth_src}}
  if rolling:
    c.update(width=size, height=size)
  if "static_layer" in layers:
    c["static_layer"] = {"plugin": "nav2_costmap_2d::StaticLayer", "map_subscribe_transient_local": True}
  return {name: {name: {"ros__parameters": c}}}


P = {}
# Inflation 0.40 / 4.0 global, 0.30 / 5.0 local: >= half the body diagonal 0.133 m (Smac README: a smooth potential
# field spread across the hallway), TB4 / Kobuki scales 4-5 (01, 03 reports).
P.update(costmap("global_costmap", False, None, 1.0, 0.40, 4.0, ["static_layer", "obstacle_layer", "inflation_layer"]))
P.update(costmap("local_costmap", True, 3, 5.0, 0.30, 5.0, ["obstacle_layer", "inflation_layer"]))
P["planner_server"] = {"ros__parameters": {
  "expected_planner_frequency": 1.0, "use_sim_time": False, "planner_plugins": ["GridBased"],
  "GridBased": {
    "plugin": "nav2_smac_planner/SmacPlannerHybrid", "tolerance": 0.25, "downsample_costmap": False,
    "allow_unknown": True, "max_iterations": 1000000, "max_planning_time": 3.0,
    "motion_model_for_search": "REEDS_SHEPP", "angle_quantization_bins": 64,
    "analytic_expansion_ratio": 3.5, "analytic_expansion_max_length": 3.0, "minimum_turning_radius": TURN_R,
    "reverse_penalty": 2.1, "change_penalty": 0.0, "non_straight_penalty": 1.2, "cost_penalty": 2.0,
    "retrospective_penalty": 0.015, "lookup_table_size": 20.0, "cache_obstacle_heuristic": True,
    "smooth_path": True}}}     # values: Smac README defaults (01 report)
if a.controller == "rpp":
  follow = {   # RPP README: use_rotate_to_heading must be false for allow_reversing (Ackermann cannot rotate in place)
    "plugin": "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    # look-ahead 0.25-0.9 m, 1.0 s x v; RPP's own costmap collision check OFF. Bench 2026-11-09 night (car/nav/bench,
    # C3 hall end -> office x steering 29/33 deg x 3): with 0.4 m / 1.5 s and the check on, the check's arc cut the
    # S-bend past the charger desk legs -> false "collision ahead" -> patience -> 18 recoveries, 2/6 failed (also on the
    # floor, a_c3_1). Shorter look-ahead alone 6/6 but up to 14 recoveries; + check off 6/6, max 2. Full validation
    # (3 courses x 29/33/40 deg x 3): 27/27, max 1 recovery, 0 contacts. Safety = Collision Monitor "approach" (exact
    # footprint, real LiDAR + depth points, simulated commanded motion) + the cmd watchdog.
    "desired_linear_vel": V_MAX, "lookahead_dist": 0.6, "min_lookahead_dist": 0.25, "max_lookahead_dist": 0.9,
    "lookahead_time": 1.0, "use_velocity_scaled_lookahead_dist": True,
    "transform_tolerance": 0.2, "min_approach_linear_velocity": 0.08,   # x 0.4 SlowZone = 0.032 >= motor dead band 0.03 (open-issues 448)
    "approach_velocity_scaling_dist": 0.6,
    "use_collision_detection": False, "max_allowed_time_to_collision_up_to_carrot": 1.0,
    "use_regulated_linear_velocity_scaling": True, "use_cost_regulated_linear_velocity_scaling": True,
    "regulated_linear_scaling_min_radius": 0.9, "regulated_linear_scaling_min_speed": 0.15,
    "use_fixed_curvature_lookahead": False, "allow_reversing": True, "use_rotate_to_heading": False,
    "max_angular_accel": 3.2, "max_robot_pose_search_dist": 10.0, "use_interpolation": True}
else:
  follow = {   # MPPI Ackermann: min_turning_r = the planner's; consider_footprint true (01 report, nav2 #5714)
    "plugin": "nav2_mppi_controller::MPPIController", "time_steps": 40, "model_dt": 0.05, "batch_size": 1000,
    "iteration_count": 1, "vx_std": 0.2, "vy_std": 0.2, "wz_std": 0.4, "vx_max": V_MAX, "vx_min": -V_REV,
    "vy_max": 0.0, "wz_max": 1.5, "temperature": 0.3, "gamma": 0.015, "motion_model": "Ackermann",
    "visualize": False, "AckermannConstraints": {"min_turning_r": TURN_R},
    "critics": ["ConstraintCritic", "CostCritic", "GoalCritic", "GoalAngleCritic", "PathAlignCritic",
                "PathFollowCritic", "PathAngleCritic", "PreferForwardCritic"],
    "ConstraintCritic": {"enabled": True, "cost_power": 1, "cost_weight": 4.0},
    "CostCritic": {"enabled": True, "cost_power": 1, "cost_weight": 3.81, "critical_cost": 300.0,
                   "consider_footprint": True, "collision_cost": 1000000.0, "near_goal_distance": 1.0},
    "GoalCritic": {"enabled": True, "cost_power": 1, "cost_weight": 5.0, "threshold_to_consider": 1.4},
    "GoalAngleCritic": {"enabled": True, "cost_power": 1, "cost_weight": 3.0, "threshold_to_consider": 0.5},
    "PreferForwardCritic": {"enabled": True, "cost_power": 1, "cost_weight": 5.0, "threshold_to_consider": 0.5},
    "PathAlignCritic": {"enabled": True, "cost_power": 1, "cost_weight": 14.0, "max_path_occupancy_ratio": 0.05,
                        "trajectory_point_step": 3, "threshold_to_consider": 0.5, "offset_from_furthest": 20,
                        "use_path_orientations": True},
    "PathFollowCritic": {"enabled": True, "cost_power": 1, "cost_weight": 5.0, "offset_from_furthest": 5,
                         "threshold_to_consider": 1.4},
    "PathAngleCritic": {"enabled": True, "cost_power": 1, "cost_weight": 2.0, "offset_from_furthest": 4,
                        "threshold_to_consider": 0.5, "max_angle_to_furthest": 1.0, "mode": 2}}
P["controller_server"] = {"ros__parameters": {
  "use_sim_time": False, "controller_frequency": 20.0, "min_x_velocity_threshold": 0.001,
  "min_y_velocity_threshold": 0.5, "min_theta_velocity_threshold": 0.001, "failure_tolerance": 0.3,
  "progress_checker_plugin": "progress_checker", "goal_checker_plugins": ["general_goal_checker"],
  "controller_plugins": ["FollowPath"],
  "progress_checker": {"plugin": "nav2_controller::SimpleProgressChecker", "required_movement_radius": 0.2,
                       "movement_time_allowance": 15.0},
  "general_goal_checker": {"plugin": "nav2_controller::SimpleGoalChecker", "stateful": True,
                           "xy_goal_tolerance": 0.3, "yaw_goal_tolerance": 3.14},
  "FollowPath": follow}}
# Recovery: no Spin (a car cannot spin). BackUp drives straight back (Humble: angular.z = 0, 01 report).
P["behavior_server"] = {"ros__parameters": {
  "costmap_topic": "local_costmap/costmap_raw", "footprint_topic": "local_costmap/published_footprint",
  "cycle_frequency": 10.0, "behavior_plugins": ["backup", "wait"],
  "backup": {"plugin": "nav2_behaviors/BackUp"}, "wait": {"plugin": "nav2_behaviors/Wait"},
  "global_frame": "odom", "robot_base_frame": "base_footprint", "transform_tolerance": 0.2,
  "simulate_ahead_time": 1.0, "max_rotational_vel": 0.0, "min_rotational_vel": 0.0, "rotational_acc_lim": 0.0}}
P["bt_navigator"] = {"ros__parameters": {
  "global_frame": "map", "robot_base_frame": "base_footprint", "odom_topic": "/odom", "bt_loop_duration": 10,
  "default_server_timeout": 20, "default_nav_to_pose_bt_xml": CAR_NAV + "/navigate_ackermann_b.xml",   # tree B: replan only if path invalid (bench 2026-11-09)
  # the through-poses default tree needs a Spin server (none: a car cannot spin) and bt_navigator then fails to
  # activate (car #1, 2026-11-09). We only send single goals, so point it at our tree as well.
  "default_nav_through_poses_bt_xml": CAR_NAV + "/navigate_ackermann_b.xml"}}
P["velocity_smoother"] = {"ros__parameters": {
  # scale_velocities: limit v and w TOGETHER. Hiwonder's driver steers atan(0.145 w / v) (ackermann.py:30), so separate
  # limits changed the steering angle: floor a_c3_1 54 flips at the motors vs 34 asked; 57 % of slow commands at full
  # lock. Bench at 0.5 m/s: needed for 27/27 (see V_MAX).
  "smoothing_frequency": 20.0, "scale_velocities": True, "feedback": "OPEN_LOOP",
  "max_velocity": [V_MAX, 0.0, 1.5], "min_velocity": [-V_REV, 0.0, -1.5],
  "max_accel": [1.0, 0.0, 3.0], "max_decel": [-1.5, 0.0, -3.0], "deadband_velocity": [0.0, 0.0, 0.0]}}
# Collision Monitor before the motors (03 report): stop polygon 0.1 m beyond the footprint, slow-down beyond it.
# Humble 1.1.15 (on car #1): "max_points" = act when MORE than this many points are inside (Iron+ renamed min_points).
# Humble polygon "points" = flat double array [x1, y1, x2, y2, ...] (a string is rejected: "is of type double_array").
def box(f, r, h):
  return [round(v, 3) for v in (f, h, f, -h, -r, -h, -r, h)]


# StopZone -> "approach" (Humble 1.1.15 supports it): it simulates the commanded motion over time_before_collision and
# stops only if the FOOTPRINT would then hit a point. A "stop" box blocks every direction: in trial a_c1_1 (2026-11-09)
# the charger 1-5 cm behind the docked car (LiDAR, x -0.12..-0.155) held the car in the dock while Nav2 drove forward.
# Margin: footprint (body + 1 cm) + 3 cm. With + 0 the bench park course touched the charger-desk legs 5 x in one run
# (the car turns ~6 % tighter than Nav2 thinks: driver L 0.145 vs URDF 0.137); + 3 cm: park 9/9 0 contacts, C1-C3
# 26/26 0 contacts (bench 2026-11-09 night, margin.sh). The docked car's charger (1-5 cm behind) is inside this
# polygon, but undock.py / dock.py drive /controller/cmd_vel directly, outside Nav2 and this monitor.
foot = box(front + 0.03, rear + 0.03, half + 0.03)
slow = box(front + 0.4, rear + 0.2, half + 0.1)
P["collision_monitor"] = {"ros__parameters": {
  "base_frame_id": "base_footprint", "odom_frame_id": "odom", "cmd_vel_in_topic": "cmd_vel_smoothed",
  "cmd_vel_out_topic": "/cmd_vel_guard_in",  # -> nav_guard.py (freshness gate, 2026-11-16) -> /controller/cmd_vel
  "transform_tolerance": 0.2, "source_timeout": 1.0,
  "stop_pub_timeout": 2.0, "polygons": ["FootprintApproach", "SlowZone"],
  "FootprintApproach": {"type": "polygon", "points": foot, "action_type": "approach", "max_points": 3,
                        "time_before_collision": 1.0, "simulation_time_step": 0.05, "visualize": False},
  "SlowZone": {"type": "polygon", "points": slow, "action_type": "slowdown", "max_points": 3,
               "slowdown_ratio": 0.4, "visualize": False},
  "observation_sources": ["scan", "depth_scan"],
  "scan": {"type": "scan", "topic": "/scan_raw"}, "depth_scan": {"type": "scan", "topic": "/depth_scan"}}}
# Localization on the saved car map: map_server + AMCL. Values = Nav2 Humble AMCL defaults (nav2_amcl docs), except:
# scan /scan_raw; initial pose = the charger (0, 0, 0), the origin of the SLAM map built from the charger
# (goto6_map_after.npz, the floor route's frame); max_beams 60 / particles 500-2000 for the Pi CPU (estimate).
P["map_server"] = {"ros__parameters": {"use_sim_time": False, "yaml_filename": ""}}   # set by nav_a.launch.py
P["amcl"] = {"ros__parameters": {
  "use_sim_time": False, "base_frame_id": "base_footprint", "odom_frame_id": "odom", "global_frame_id": "map",
  "scan_topic": "/scan_raw", "robot_model_type": "nav2_amcl::DifferentialMotionModel",
  "alpha1": 0.2, "alpha2": 0.2, "alpha3": 0.2, "alpha4": 0.2, "alpha5": 0.2,
  "laser_model_type": "likelihood_field", "laser_likelihood_max_dist": 2.0, "laser_max_range": 10.0,
  "laser_min_range": 0.15, "max_beams": 60, "min_particles": 500, "max_particles": 2000,
  "pf_err": 0.05, "pf_z": 0.99, "resample_interval": 1, "sigma_hit": 0.2, "z_hit": 0.5, "z_max": 0.05,
  "z_rand": 0.5, "z_short": 0.05, "lambda_short": 0.1, "update_min_a": 0.2, "update_min_d": 0.1,
  "transform_tolerance": 1.0, "tf_broadcast": True, "save_pose_rate": 0.5,
  "set_initial_pose": True, "initial_pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}}}
P["lifecycle_manager_localization"] = {"ros__parameters": {"autostart": True, "bond_timeout": 0.0,
                                                           "node_names": ["map_server", "amcl"]}}
# Lifecycle managers: bond_timeout 0 disables the 0.1 s heartbeats (nav2 #5784: 140% idle CPU with them).
P["lifecycle_manager_navigation"] = {"ros__parameters": {"autostart": True, "bond_timeout": 0.0, "node_names": [
  "controller_server", "planner_server", "behavior_server", "bt_navigator", "velocity_smoother", "collision_monitor"]}}
out = os.path.join(HERE, "nav2_params_a.yaml")
class NoAlias(yaml.SafeDumper):   # rcl's params parser rejects YAML anchors/aliases ("Will not support aliasing")
  def ignore_aliases(self, data):
    return True


yaml.dump(P, open(out, "w", newline=chr(10)), Dumper=NoAlias, sort_keys=False)   # LF: the file runs on the car
print(f"wrote {out}: footprint {FOOTPRINT}, turning radius {TURN_R}, controller {a.controller}")
