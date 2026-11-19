# Send one Nav2 goal in the map frame and report until it ends. Run inside the MentorPi container:
#   python3 goal.py X Y YAW_DEG [TIMEOUT_S]     |     python3 goal.py cancel
# Prints distance remaining and the robot pose every second. Ctrl-C or a timeout cancels the goal (the car stops:
# Nav2 publishes zero, and the drive command watchdog stops the motors if commands end).
import math, sys, time
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

rclpy.init()
if sys.argv[1] == 'cancel':
  # Cancel every navigate_to_pose goal, whoever sent it: zero goal id + zero stamp = all goals (action_msgs CancelGoal).
  from action_msgs.srv import CancelGoal
  n = rclpy.create_node('goal_cancel')
  cli = n.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
  if not cli.wait_for_service(timeout_sec=5.0):
    print('no navigate_to_pose action server'); sys.exit(1)
  f = cli.call_async(CancelGoal.Request()); rclpy.spin_until_future_complete(n, f, timeout_sec=5.0)
  print('cancel all:', f.result()); sys.exit(0)
nav = BasicNavigator()
x, y, yaw = float(sys.argv[1]), float(sys.argv[2]), math.radians(float(sys.argv[3]))
timeout = float(sys.argv[4]) if len(sys.argv) > 4 else 60.0
g = PoseStamped(); g.header.frame_id = 'map'; g.header.stamp = nav.get_clock().now().to_msg()
g.pose.position.x, g.pose.position.y = x, y
g.pose.orientation.z, g.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
nav.goToPose(g)
t0 = time.monotonic(); last = 0
try:
  while not nav.isTaskComplete():
    fb = nav.getFeedback()
    if fb and time.monotonic() - last > 1.0:
      p = fb.current_pose.pose
      print(f'{time.monotonic() - t0:5.1f} s  remaining {fb.distance_remaining:.2f} m  at ({p.position.x:.2f}, '
            f'{p.position.y:.2f})  recoveries {fb.number_of_recoveries}', flush=True)
      last = time.monotonic()
    if time.monotonic() - t0 > timeout:
      nav.cancelTask(); print('TIMEOUT: cancelled'); break
    time.sleep(0.1)
except KeyboardInterrupt:
  nav.cancelTask(); print('interrupted: cancelled')
r = nav.getResult()
print({TaskResult.SUCCEEDED: 'SUCCEEDED', TaskResult.CANCELED: 'CANCELED', TaskResult.FAILED: 'FAILED'}.get(r, r),
      f'after {time.monotonic() - t0:.1f} s')
rclpy.shutdown()
