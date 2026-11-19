# Wheels-up watchdog test: drive at 0.10 m/s on /controller/cmd_vel at 20 Hz for 1.5 s, then go silent WITHOUT a
# stop message (as a dropped Wi-Fi link would). The watchdog must zero the motors about 0.5 s later. Logs set_motor
# and IMU vibration (the board reports no wheel speed). Exits with zeros sent as a safety net either way.
import math, statistics, time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from ros_robot_controller_msgs.msg import MotorsState

rclpy.init(); n = rclpy.create_node('watchdog_test')
pub = n.create_publisher(Twist, '/controller/cmd_vel', 10)
T0 = time.monotonic(); now = lambda: time.monotonic() - T0
motor, imu = [], []
n.create_subscription(MotorsState, '/ros_robot_controller/set_motor',
                      lambda m: motor.append((now(), [round(d.rps, 3) for d in m.data])), 50)
n.create_subscription(Imu, '/ros_robot_controller/imu_raw', lambda m: imu.append((now(), math.sqrt(
    m.linear_acceleration.x ** 2 + m.linear_acceleration.y ** 2 + m.linear_acceleration.z ** 2))), qos_profile_sensor_data)
ex = SingleThreadedExecutor(); ex.add_node(n)


def spin(s):
  end = time.monotonic() + s
  while time.monotonic() < end: ex.spin_once(timeout_sec=0.01)


def vib(t0, t1):
  v = [a for t, a in imu if t0 <= t < t1]
  return statistics.pstdev(v) if len(v) > 3 else float('nan')


spin(2.0)
t_go = now()
t = Twist(); t.linear.x = 0.10
end = time.monotonic() + 1.5; nxt = time.monotonic()
while time.monotonic() < end:
  if time.monotonic() >= nxt: pub.publish(t); nxt += 0.05
  ex.spin_once(timeout_sec=0.01)
t_silent = now(); print(f'{t_go:4.1f} s drive 0.10 m/s at 20 Hz; {t_silent:4.1f} s went silent (no stop sent)')
spin(3.0)
zero_at = next((tt for tt, v in motor if tt > t_silent and all(abs(x) < 1e-6 for x in v)), None)
print(f'watchdog zero arrived {zero_at - t_silent:.2f} s after silence' if zero_at else 'NO ZERO from the watchdog')
tt = 0.0
while tt < now():
  mark = ' <- go' if tt <= t_go < tt + 0.5 else ' <- silent' if tt <= t_silent < tt + 0.5 else ''
  print(f'  {tt:4.1f} vib {vib(tt, tt + 0.5):.4f}{mark}')
  tt += 0.5
for _ in range(5): pub.publish(Twist()); spin(0.05)
n.destroy_node(); rclpy.shutdown()
