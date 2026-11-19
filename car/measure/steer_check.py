# Wheels-up steering check: PWM servo id 3 (board port S5, which the stock driver steers) to left, centre, right,
# centre, 2 s each. Pulse = 1500 + 2000 * deg / 180, + = left (ackermann.py set_velocity). 25 deg is inside the
# 29-37 deg steering range (research/01), so no end-stop strain.
import time, rclpy
from ros_robot_controller_msgs.msg import SetPWMServoState, PWMServoState

DEG = 25
rclpy.init(); n = rclpy.create_node('steer_check')
pub = n.create_publisher(SetPWMServoState, '/ros_robot_controller/pwm_servo/set_state', 10)
for _ in range(30):
  if pub.get_subscription_count(): break
  rclpy.spin_once(n, timeout_sec=0.1)


def steer(deg, label):
  s = PWMServoState(); s.id = [3]; s.position = [int(1500 + 2000 * deg / 180)]
  m = SetPWMServoState(); m.state = [s]; m.duration = 0.3
  pub.publish(m); print(f'{label}: pulse {s.position[0]}', flush=True); time.sleep(2.0)


steer(0, 'centre'); steer(DEG, 'LEFT'); steer(0, 'centre'); steer(-DEG, 'RIGHT'); steer(0, 'centre')
n.destroy_node(); rclpy.shutdown()
