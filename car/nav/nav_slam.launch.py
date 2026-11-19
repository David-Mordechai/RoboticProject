# Nav2 while mapping on a MentorPi A1: the car base + slam_toolbox (Hiwonder's stock slam.launch.py), then Nav2
# without AMCL / map server, because slam_toolbox provides map -> odom and /map. This is what frontier exploration
# drives on (exploration plan step 1-2).
#
# Run inside the MentorPi container, after stopping the autostart bringup (it starts its own copy of the base nodes):
#   python3 make_params.py && ros2 launch /home/ubuntu/shared/nav/nav_slam.launch.py
# Nav2 nodes: the same set Hiwonder's navigation_base.launch.py loads (standard Nav2), but with our parameter files
# (make_params.py: Smac Hybrid-A* planner, 0.4 m turning radius, TEB at 0.2 m/s, no back-up recoveries). Hiwonder's
# file hard-codes its TEB parameter path, so the nodes are declared here.
# Speed: Nav2 publishes /cmd_vel, which the stock driver clips to 0.2 m/s and 0.5 rad/s (odom_publisher_node.py
# app_cmd_vel_callback). The drive command watchdog (car/watchdog) stops the motors if commands stop.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode

SLAM = '/home/ubuntu/ros2_ws/src/slam/launch/slam.launch.py'
HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS = os.path.join(HERE, 'nav2_params_ours.yaml')
TEB = os.path.join(HERE, 'teb_ours.yaml')
# CONTROLLER=mppi (2026-11-04): MPPI Ackermann (make_params.py) and Nav2's output on /controller/cmd_vel (uncapped)
MPPI = os.environ.get('CONTROLLER') == 'mppi'
if MPPI:
  TEB = os.path.join(HERE, 'mppi_ours.yaml')
  PARAMS = os.path.join(HERE, 'nav2_params_mppi.yaml')
OUT = '/controller/cmd_vel' if MPPI else 'cmd_vel'
DEPTH_SCAN = os.path.join(os.path.dirname(HERE), 'perception', 'depth_scan.py')
REMAP = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
LIFECYCLE = ['controller_server', 'smoother_server', 'planner_server', 'behavior_server', 'bt_navigator',
             'waypoint_follower', 'velocity_smoother']


def nav2_nodes():
  def node(pkg, plugin, name, params, remap=()):
    return ComposableNode(package=pkg, plugin=plugin, name=name, parameters=params, remappings=REMAP + list(remap))
  return LoadComposableNodes(target_container='nav2_container', composable_node_descriptions=[
    node('nav2_controller', 'nav2_controller::ControllerServer', 'controller_server', [TEB], [('cmd_vel', 'cmd_vel_nav')]),
    node('nav2_smoother', 'nav2_smoother::SmootherServer', 'smoother_server', [PARAMS]),
    node('nav2_planner', 'nav2_planner::PlannerServer', 'planner_server', [PARAMS]),
    node('nav2_behaviors', 'behavior_server::BehaviorServer', 'behavior_server', [PARAMS]),
    node('nav2_bt_navigator', 'nav2_bt_navigator::BtNavigator', 'bt_navigator', [PARAMS]),
    node('nav2_waypoint_follower', 'nav2_waypoint_follower::WaypointFollower', 'waypoint_follower', [PARAMS]),
    node('nav2_velocity_smoother', 'nav2_velocity_smoother::VelocitySmoother', 'velocity_smoother', [PARAMS],
         [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', OUT)]),
    ComposableNode(package='nav2_lifecycle_manager', plugin='nav2_lifecycle_manager::LifecycleManager',
                   name='lifecycle_manager_navigation',
                   parameters=[{'use_sim_time': False, 'autostart': True, 'node_names': LIFECYCLE}]),
  ])


def generate_launch_description():
  return LaunchDescription([
    IncludeLaunchDescription(PythonLaunchDescriptionSource(SLAM), launch_arguments={'enable_save': 'true'}.items()),
    # The dashboard (car/dashboard) needs rosbridge and the video server; the stock autostart starts them, but it is
    # stopped for this launch. slam.launch.py starts neither (checked 2026-11-02).
    *([] if MPPI else [IncludeLaunchDescription(AnyLaunchDescriptionSource(os.path.join(   # MPPI: CPU for Nav2
      get_package_share_directory('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml')))]),
    Node(package='web_video_server', executable='web_video_server', name='web_video_server', output='screen'),
    # Depth camera obstacles below the LiDAR plane -> /depth_scan (Nav2 costmaps) and /depth_sectors (policies).
    ExecuteProcess(cmd=['python3', DEPTH_SCAN], output='screen'),
    # Same container Hiwonder's navigation bringup.launch.py creates.
    # MPPI: the single-threaded container. component_container_isolated (one executor thread per node) used 154% CPU
    # with Nav2 idle on the Pi 5 (2026-11-04: one thread at 100%, five at 30-45%, load 18 on 4 cores; the first goal
    # was never acknowledged).
    Node(package='rclcpp_components', executable='component_container' if MPPI else 'component_container_isolated',
         name='nav2_container',
         parameters=[PARAMS, {'autostart': True}], output='screen'),
    # slam.launch.py starts slam_toolbox after 10 s; give it a map before Nav2's costmaps come up.
    TimerAction(period=18.0, actions=[nav2_nodes()]),
  ])
