# Phase A (2026-11-09, docs/research/deep-2026-11-09/00-synthesis.md): Nav2 on the SAVED car map.
#   car base (Hiwonder robot.launch.py: drivers, LiDAR /scan_raw, odom, EKF, TF)
#   + depth_scan.py (/depth_scan, obstacles below the LiDAR plane; replaces STVL, which is not installed on the car)
#   + one component_container: map_server, AMCL, controller (RPP), planner (Smac Hybrid REEDS_SHEPP), behaviors (BackUp,
#     Wait), bt_navigator (navigate_ackermann_a.xml), velocity_smoother, collision_monitor; lifecycle bonds off.
# Command path: controller -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed -> collision_monitor
#   -> /cmd_vel_guard_in -> nav_guard.py (stops if TF/scan stale, 2026-11-16) -> /controller/cmd_vel (driver; the cmd watchdog zeroes it 0.5 s after commands stop).
# Run inside the MentorPi container ON TOP of the stock autostart (start_node.service already runs the base: MS200,
# ascamera, odom_publisher, EKF, robot_state_publisher, driver, web_video_server; checked 2026-11-09), like the
# policy runs. BASE=1 adds Hiwonder's robot.launch.py only if the autostart is stopped.
#   MAP=/home/ubuntu/shared/maps/goto6.yaml ros2 launch /home/ubuntu/shared/nav/nav_a.launch.py
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode

HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS = os.path.join(HERE, 'nav2_params_a.yaml')
MAP = os.environ.get('MAP', '/home/ubuntu/shared/maps/goto6.yaml')
ROBOT = '/home/ubuntu/ros2_ws/src/slam/launch/include/robot.launch.py'
DEPTH_SCAN = os.path.join(os.path.dirname(HERE), 'perception', 'depth_scan.py')
NAV_GUARD = os.path.join(HERE, 'nav_guard.py')
REMAP = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


def comp(pkg, plugin, name, extra=None, remap=()):
  return ComposableNode(package=pkg, plugin=plugin, name=name, parameters=[PARAMS] + ([extra] if extra else []),
                        remappings=REMAP + list(remap))


def generate_launch_description():
  loc = LoadComposableNodes(target_container='nav2_container', composable_node_descriptions=[
    comp('nav2_map_server', 'nav2_map_server::MapServer', 'map_server', {'yaml_filename': MAP}),
    comp('nav2_amcl', 'nav2_amcl::AmclNode', 'amcl'),
    comp('nav2_lifecycle_manager', 'nav2_lifecycle_manager::LifecycleManager', 'lifecycle_manager_localization')])
  nav = LoadComposableNodes(target_container='nav2_container', composable_node_descriptions=[
    comp('nav2_controller', 'nav2_controller::ControllerServer', 'controller_server', remap=[('cmd_vel', 'cmd_vel_nav')]),
    comp('nav2_planner', 'nav2_planner::PlannerServer', 'planner_server'),
    comp('nav2_behaviors', 'behavior_server::BehaviorServer', 'behavior_server', remap=[('cmd_vel', 'cmd_vel_nav')]),
    comp('nav2_bt_navigator', 'nav2_bt_navigator::BtNavigator', 'bt_navigator'),
    comp('nav2_velocity_smoother', 'nav2_velocity_smoother::VelocitySmoother', 'velocity_smoother',
         remap=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel_smoothed')]),
    comp('nav2_collision_monitor', 'nav2_collision_monitor::CollisionMonitor', 'collision_monitor'),
    comp('nav2_lifecycle_manager', 'nav2_lifecycle_manager::LifecycleManager', 'lifecycle_manager_navigation')])
  base = [IncludeLaunchDescription(PythonLaunchDescriptionSource(ROBOT),
                                   launch_arguments={'sim': 'false', 'master_name': '/', 'robot_name': '/'}.items())]
  return LaunchDescription([
    *(base if os.environ.get('BASE') == '1' else []),
    ExecuteProcess(cmd=['python3', DEPTH_SCAN], output='screen'),
    ExecuteProcess(cmd=['python3', NAV_GUARD], output='screen'),
    # single-threaded container: the isolated one used 154% CPU idle on the Pi 5 (2026-11-04)
    Node(package='rclcpp_components', executable='component_container', name='nav2_container',
         parameters=[PARAMS], output='screen'),
    TimerAction(period=8.0, actions=[loc]),
    TimerAction(period=14.0, actions=[nav]),
  ])
