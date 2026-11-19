# Write our Nav2 parameter files next to this script: Hiwonder's stock files (read on the car, not copied into this
# repo) with our overrides. Run inside the MentorPi container:  python3 make_params.py
#   nav2_params_ours.yaml  <- navigation/config/nav2_params.yaml
#   teb_ours.yaml          <- navigation/config/nav2_controller_teb.yaml
import os
import yaml

CFG = '/home/ubuntu/ros2_ws/src/navigation/config'
HERE = os.path.dirname(os.path.abspath(__file__))

# Costmap footprint: stock robot_radius is 0.05 m, but the A1 is 213 x 159 mm (research/01 sec. 2), half-diagonal
# 0.133 m. Plan with 0.14 m so paths keep the whole car off walls (the inflation layer then adds its 0.25 m falloff).
ROBOT_RADIUS = 0.14
# Turning radius for planning and control. Stock TEB uses 1.0 m, which made the car reverse in loops in a ~3 m room
# (2026-11-02). Geometry: R = wheelbase / tan(steer) = 0.145 / tan(29..37 deg) = 0.19-0.26 m (Hiwonder's formula,
# research/01). The stock driver clips /cmd_vel to 0.2 m/s and 0.5 rad/s (odom_publisher_node.py), i.e. R >= 0.4 m
# at full speed. Use 0.4 m: reachable at every speed the driver allows.
TURN_RADIUS = 0.4
V_MAX = 0.2          # m/s, the driver's /cmd_vel clip; TEB plans at this speed instead of its stock 0.4
BT_XML = os.path.join(HERE, 'navigate_ackermann.xml')

p = yaml.safe_load(open(f'{CFG}/nav2_params.yaml'))
for cm in ('local_costmap', 'global_costmap'):
  c = p[cm][cm]['ros__parameters']
  c['robot_radius'] = ROBOT_RADIUS
  # Second obstacle source: the depth camera's below-LiDAR obstacles (car/perception/depth_scan.py), e.g. chair bases
  # the LiDAR plane (~0.16 m up) passes over. Added to every LiDAR-fed obstacle/voxel layer.
  for layer in c.values():
    if isinstance(layer, dict) and 'observation_sources' in layer and 'depth_scan' not in layer['observation_sources']:
      layer['observation_sources'] = layer['observation_sources'] + ' depth_scan'
      layer['depth_scan'] = {'topic': '/depth_scan', 'data_type': 'LaserScan', 'marking': True, 'clearing': True,
                             'obstacle_max_range': 2.5, 'obstacle_min_range': 0.0, 'raytrace_max_range': 3.0,
                             'raytrace_min_range': 0.0, 'max_obstacle_height': 2.0, 'min_obstacle_height': 0.0,
                             'inf_is_valid': True}
# Planner: Smac Hybrid-A* plans car-feasible paths (Nav2 docs: "Ackermann, car, and car-like robots"); stock NavFn
# plans paths a car cannot follow. Reeds-Shepp allows short reverses in tight spots (360 deg LiDAR watches behind),
# with a penalty so it prefers driving forward. Parameter names: nav2_smac_planner Humble.
p['planner_server']['ros__parameters']['GridBased'] = {
  'plugin': 'nav2_smac_planner/SmacPlannerHybrid',
  'tolerance': 0.25,
  'downsample_costmap': False,
  'allow_unknown': True,
  'max_iterations': 1000000,
  'max_planning_time': 2.0,
  'motion_model_for_search': 'REEDS_SHEPP',
  'angle_quantization_bins': 72,
  'analytic_expansion_ratio': 3.5,
  'analytic_expansion_max_length': 3.0,
  'minimum_turning_radius': TURN_RADIUS,
  'reverse_penalty': 2.5,
  'change_penalty': 0.0,
  'non_straight_penalty': 1.2,
  'cost_penalty': 2.0,
  'lookup_table_size': 20.0,
  'cache_obstacle_heuristic': False,
  'smooth_path': True,
}
# Behaviour tree without BackUp / Spin: a car cannot spin in place, and repeated back-ups were what the user saw as
# "only driving backward". Recovery = clear costmaps, wait, replan; give up after 4 tries.
p['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = BT_XML
yaml.safe_dump(p, open(os.path.join(HERE, 'nav2_params_ours.yaml'), 'w'), sort_keys=False)

t = yaml.safe_load(open(f'{CFG}/nav2_controller_teb.yaml'))
fp = t['controller_server']['ros__parameters']['FollowPath']
fp['min_turning_radius'] = TURN_RADIUS
fp['max_vel_x'] = V_MAX
fp['max_vel_x_backwards'] = 0.1          # m/s, slow reverses only (TEB default 0.2)
yaml.safe_dump(t, open(os.path.join(HERE, 'teb_ours.yaml'), 'w'), sort_keys=False)

# MPPI with the Ackermann motion model (2026-11-04: TEB reversed in loops / would not follow the Hybrid-A* path). Nav2
# Humble nav2_mppi_controller parameter names (README, docs.nav2.org/configuration/packages/configuring-mppic.html).
# Nav2's output goes to /controller/cmd_vel (uncapped, as our policies) instead of /cmd_vel (driver clip 0.2 m/s):
# see nav_slam.launch.py (CONTROLLER=mppi). Speeds are design choices below the 0.7 m/s joystick limit (research/01).
MPPI_V_MAX, MPPI_V_REV = 0.5, -0.25
MPPI_TURN_R = 0.30       # m: R = 0.145 / tan(29..37 deg) = 0.19-0.26 m (research/01) + margin; ESTIMATE until measured
m = {'controller_server': {'ros__parameters': {
  'use_sim_time': False, 'controller_frequency': 20.0, 'min_x_velocity_threshold': 0.001,
  'min_y_velocity_threshold': 0.5, 'min_theta_velocity_threshold': 0.001, 'failure_tolerance': 0.3,
  'progress_checker_plugin': 'progress_checker', 'goal_checker_plugins': ['general_goal_checker'],
  'controller_plugins': ['FollowPath'],
  'progress_checker': {'plugin': 'nav2_controller::SimpleProgressChecker', 'required_movement_radius': 0.3,
                       'movement_time_allowance': 15.0},
  'general_goal_checker': {'plugin': 'nav2_controller::SimpleGoalChecker', 'stateful': True,
                           'xy_goal_tolerance': 0.3, 'yaw_goal_tolerance': 3.14},
  'FollowPath': {
    'plugin': 'nav2_mppi_controller::MPPIController',
    'time_steps': 40, 'model_dt': 0.05, 'batch_size': 1000, 'iteration_count': 1,   # Pi 5 CPU: below the 56 x 2000 defaults
    'vx_std': 0.2, 'vy_std': 0.2, 'wz_std': 0.4,
    'vx_max': MPPI_V_MAX, 'vx_min': MPPI_V_REV, 'vy_max': 0.0, 'wz_max': 1.9,
    'temperature': 0.3, 'gamma': 0.015, 'motion_model': 'Ackermann', 'visualize': False,
    'reset_period': 1.0, 'regenerate_noises': False,
    'AckermannConstraints': {'min_turning_r': MPPI_TURN_R},
    'critics': ['ConstraintCritic', 'ObstaclesCritic', 'GoalCritic', 'GoalAngleCritic', 'PathAlignCritic',
                'PathFollowCritic', 'PathAngleCritic', 'PreferForwardCritic'],
    'ConstraintCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 4.0},
    'GoalCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 5.0, 'threshold_to_consider': 1.4},
    'GoalAngleCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 3.0, 'threshold_to_consider': 0.5},
    'PreferForwardCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 5.0, 'threshold_to_consider': 0.5},
    'ObstaclesCritic': {'enabled': True, 'cost_power': 1, 'repulsion_weight': 1.5, 'critical_weight': 20.0,
                        'consider_footprint': False, 'collision_cost': 10000.0, 'collision_margin_distance': 0.1,
                        'near_goal_distance': 0.5},
    'PathAlignCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 14.0, 'max_path_occupancy_ratio': 0.05,
                        'trajectory_point_step': 3, 'threshold_to_consider': 0.5, 'offset_from_furthest': 20,
                        'use_path_orientations': False},
    'PathFollowCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 5.0, 'offset_from_furthest': 5,
                         'threshold_to_consider': 1.4},
    'PathAngleCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 2.0, 'offset_from_furthest': 4,
                        'threshold_to_consider': 0.5, 'max_angle_to_furthest': 1.0, 'mode': 0},
  }}}}
yaml.safe_dump(m, open(os.path.join(HERE, 'mppi_ours.yaml'), 'w'), sort_keys=False)
vs = p['velocity_smoother']['ros__parameters']
vs.update({'max_velocity': [MPPI_V_MAX, 0.0, 2.0], 'min_velocity': [MPPI_V_REV, 0.0, -2.0],
           'max_accel': [1.0, 0.0, 3.0], 'max_decel': [-1.5, 0.0, -3.0]})
yaml.safe_dump(p, open(os.path.join(HERE, 'nav2_params_mppi.yaml'), 'w'), sort_keys=False)
print('wrote nav2_params_ours.yaml, teb_ours.yaml, mppi_ours.yaml, nav2_params_mppi.yaml: robot_radius', ROBOT_RADIUS,
      'turning radius', TURN_RADIUS, '(MPPI', MPPI_TURN_R, ')')
