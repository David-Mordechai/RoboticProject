# Nav2 on a SAVED map, the stock Hiwonder way (navigation.launch.py: robot base + map_server + AMCL, then Nav2), but
# with our Nav2 nodes: Smac Hybrid-A* + MPPI Ackermann (make_params.py). Stock TEB on hallway_01 (2026-11-04): "trajectory
# is not feasible", then Spin recoveries a car cannot do; stock navigation_base.launch.py hard-codes its TEB / DWB file.
# Run inside the MentorPi container, with the autostart stopped (it starts its own base):
#   python3 make_params.py && MAP=hallway_01 CONTROLLER=mppi ros2 launch /home/ubuntu/shared/nav/nav_map.launch.py
import importlib.util
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
SLAM_PKG = '/home/ubuntu/ros2_ws/src/slam'
NAV_PKG = '/home/ubuntu/ros2_ws/src/navigation'
_spec = importlib.util.spec_from_file_location('nav_slam', os.path.join(HERE, 'nav_slam.launch.py'))
NS = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(NS)   # PARAMS, nav2_nodes() (CONTROLLER=mppi)


def generate_launch_description():
  return LaunchDescription([
    IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(SLAM_PKG, 'launch/include/robot.launch.py')),
                             launch_arguments={'sim': 'false', 'master_name': '/', 'robot_name': '/'}.items()),
    TimerAction(period=8.0, actions=[IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(NAV_PKG, 'launch/include/localization.launch.py')),
      launch_arguments={'map': os.path.join(SLAM_PKG, 'maps', os.environ.get('MAP', 'hallway_01') + '.yaml'),
                        'params_file': NS.PARAMS, 'use_sim_time': 'false', 'autostart': 'true',
                        'namespace': '', 'use_namespace': 'false'}.items())]),
    Node(package='rclcpp_components', executable='component_container_mt', name='nav2_container',   # one thread: map_server missed its 4 s bond heartbeat
         parameters=[NS.PARAMS, {'autostart': True}], output='screen'),
    TimerAction(period=12.0, actions=[NS.nav2_nodes()]),
  ])
