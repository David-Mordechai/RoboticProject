# Wheels-up stop check at a realistic command rate, with the IMU as the wheel-motion sensor (the board reports no
# wheel speed). Usage: python3 stop_test.py SPEED SECONDS
# Sends SPEED on /controller/cmd_vel at 20 Hz (Nav2 / policy rate) for SECONDS, then ONE zero message (as the stock
# joystick does on release). Logs every /ros_robot_controller/set_motor message and the IMU vibration (std of
# |accel| per 0.5 s window). If the vibration has not dropped to the idle level 2 s after the zero, it sends zeros at
# 20 Hz as a backup and says so.
import math, statistics, sys, time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from ros_robot_controller_msgs.msg import MotorsState

speed, secs = float(sys.argv[1]), float(sys.argv[2])
rclpy.init(); n = rclpy.create_node('stop_test')
pub = n.create_publisher(Twist, '/controller/cmd_vel', 10)
T0 = time.monotonic(); now = lambda: time.monotonic() - T0
motor, imu = [], []
n.create_subscription(MotorsState, '/ros_robot_controller/set_motor',
                      lambda m: motor.append((now(), [round(d.rps, 3) for d in m.data])), 50)
n.create_subscription(Imu, '/ros_robot_controller/imu_raw', lambda m: imu.append((now(), math.sqrt(
    m.linear_acceleration.x ** 2 + m.linear_acceleration.y ** 2 + m.linear_acceleration.z ** 2))), qos_profile_sensor_data)
ex = SingleThreadedExecutor(); ex.add_node(n)


def run(v, s, rate=20.0):
  t = Twist(); t.linear.x = float(v)
  end = time.monotonic() + s
  nxt = time.monotonic()
  while time.monotonic() < end:
    if time.monotonic() >= nxt:
      pub.publish(t); nxt += 1.0 / rate
    ex.spin_once(timeout_sec=max(0.0, min(0.01, nxt - time.monotonic())))


def idle(s):
  end = time.monotonic() + s
  while time.monotonic() < end: ex.spin_once(timeout_sec=0.01)


def vib(t0, t1):
  v = [a for t, a in imu if t0 <= t < t1]
  return statistics.pstdev(v) if len(v) > 3 else float('nan')


for _ in range(30):
  if pub.get_subscription_count(): break
  idle(0.1)
idle(2.0)
base = vib(0, now())
t_go = now(); print(f'{t_go:5.1f} s forward {speed} m/s at 20 Hz for {secs} s')
run(speed, secs)
t_stop = now(); pub.publish(Twist()); print(f'{t_stop:5.1f} s ONE zero message')
idle(2.0)
backup = vib(now() - 0.5, now()) > 3 * base
if backup:
  print(f'{now():5.1f} s still vibrating 2 s after the zero: BACKUP zeros at 20 Hz'); run(0.0, 1.0)
idle(2.0)
print(f'idle vibration {base:.4f} m/s^2; per 0.5 s window from start:')
t = 0.0
while t < now():
  mark = ' <- go' if t <= t_go < t + 0.5 else ' <- zero' if t <= t_stop < t + 0.5 else ''
  print(f'  {t:4.1f} {vib(t, t + 0.5):.4f}{mark}')
  t += 0.5
print('set_motor changes (t, rps per motor 1-4):')
last = None
for t_, v in motor:
  if v != last: print(f'  {t_:5.2f}  {v}')
  last = v
print(f'{len(motor)} set_motor messages; last {motor[-1][1] if motor else None} at {motor[-1][0] if motor else 0:.2f} s;'
      f' backup needed: {backup}')
pub.publish(Twist())
n.destroy_node(); rclpy.shutdown()
