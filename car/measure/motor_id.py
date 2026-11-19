# Wheels-up motor identification: +0.3 rps on M2 alone for 2 s, pause, then M4 alone. The watcher reports which wheel
# turned and which way. Bypasses cmd_vel: publishes straight to the board driver, always ending with all zeros.
import time, rclpy
from ros_robot_controller_msgs.msg import MotorState, MotorsState

rclpy.init(); n = rclpy.create_node('motor_id')
pub = n.create_publisher(MotorsState, '/ros_robot_controller/set_motor', 10)
for _ in range(30):
  if pub.get_subscription_count(): break
  rclpy.spin_once(n, timeout_sec=0.1)


def motors(rps):
  m = MotorsState(); m.data = []
  for i, r in enumerate(rps):
    s = MotorState(); s.id = i + 1; s.rps = float(r); m.data.append(s)
  return m


def hold(rps, s):
  end = time.monotonic() + s
  while time.monotonic() < end:
    pub.publish(motors(rps)); time.sleep(0.1)


try:
  print('3 s countdown'); hold([0, 0, 0, 0], 3.0)
  print('M2 +0.3 rps'); hold([0, 0.3, 0, 0], 2.0)
  print('pause'); hold([0, 0, 0, 0], 3.0)
  print('M4 +0.3 rps'); hold([0, 0, 0, 0.3], 2.0)
finally:
  hold([0, 0, 0, 0], 1.0)
  print('stopped')
n.destroy_node(); rclpy.shutdown()
