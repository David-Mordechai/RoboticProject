"""Measure car #1's real steering limit (2026-11-09 night: research/01 gives 29 deg for the 2024 build and 40 deg for the
2025 build; ours is NOT measured, and the Nav2 bench fails 1 in 6 times on the way back only at 29 deg).

Drives at 0.15 m/s with a full-lock command (driver: delta = atan(0.145 w / v) clipped to 40 deg, ackermann.py:30),
first left then right, ARC_S seconds each, and reads the gyro yaw rate (/imu angular_velocity.z). Turning radius of the
axle midpoint R = v / omega; steering angle delta = atan(L / R_rear) with L = 0.137 m (URDF) and R_rear = sqrt(R^2 -
(L/2)^2). v is the measured /odom speed if it moves, else the command. Stops at once if the LiDAR sees anything closer
than STOP_M in any direction. Needs a free circle of about 1 m diameter around the car.
  python3 steer_limit.py [ARC_S=2.0]
"""
import math, sys, time
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan

ARC_S = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
V, W_FULL, L, STOP_M = 0.15, 2.0, 0.137, 0.30     # w 2.0 at v 0.15 -> atan(0.145*2/0.15) = 62 deg -> driver clips to 40
rclpy.init(); n = rclpy.create_node('steer_limit'); st = {'gz': [], 'v': []}
pub = n.create_publisher(Twist, '/controller/cmd_vel', 10)
n.create_subscription(Imu, '/imu', lambda m: st['gz'].append((time.time(), m.angular_velocity.z)), qos_profile_sensor_data)
n.create_subscription(Odometry, '/odom', lambda m: st['v'].append((time.time(), m.twist.twist.linear.x)), 10)
n.create_subscription(LaserScan, '/scan_raw', lambda m: st.__setitem__('scan', m), qos_profile_sensor_data)
# Servo pulses the driver really sends (2026-11-11: first floor run read ~0 yaw; check the command reached the servo).
try:
  from ros_robot_controller_msgs.msg import SetPWMServoState
  st['servo'] = []
  n.create_subscription(SetPWMServoState, 'ros_robot_controller/pwm_servo/set_state',
                        lambda m: st['servo'].extend(p for s in m.state for p in s.position), 10)
except ImportError:
  pass


def spin_all(t=0.05):
  """Process EVERY pending message for t seconds (2026-11-11 bug: one spin_once per loop handled 1 of ~85 msgs/s,
  so the IMU samples were sparse and stale)."""
  end = time.time() + t
  while time.time() < end: rclpy.spin_once(n, timeout_sec=max(0.0, min(0.01, end - time.time())))


def near():
  m = st.get('scan')
  return min((r for r in m.ranges if m.range_min < r < m.range_max), default=9.0) if m else 9.0


def stop():
  for _ in range(5): pub.publish(Twist()); time.sleep(0.05)


t = time.time()
while 'scan' not in st and time.time() - t < 3: rclpy.spin_once(n, timeout_sec=0.05)
if near() < STOP_M:
  print(f'not started: something {near():.2f} m from the LiDAR (< {STOP_M})'); sys.exit(1)
res = {}
for side, w in (('left', W_FULL), ('right', -W_FULL)):
  st['gz'].clear(); st['v'].clear(); t0 = time.time(); why = 'done'
  while time.time() - t0 < ARC_S:
    spin_all(0.05)
    if near() < STOP_M: why = f'LiDAR stop {near():.2f} m'; break
    c = Twist(); c.linear.x = V; c.angular.z = w; pub.publish(c)
  stop(); spin_all(0.5)
  gz = np.array([g for (tt, g) in st['gz'] if tt - t0 > 0.6])          # skip the servo / speed transient
  vv = np.array([v for (tt, v) in st['v'] if tt - t0 > 0.6])
  sv = st.get('servo', [])
  print(f'{side}: {len(gz)} gyro samples, servo pulses sent: {sorted(set(sv))[:6]}', flush=True)
  if 'servo' in st: st['servo'] = []
  if len(gz) < 5:
    print(f'{side}: too short ({why})'); continue
  om = abs(float(np.median(gz))); v = float(np.median(np.abs(vv))) if len(vv) and np.median(np.abs(vv)) > 0.02 else V
  R = v / om if om > 1e-3 else float('inf')
  Rr = math.sqrt(max(R * R - (L / 2) ** 2, 1e-6))
  res[side] = math.degrees(math.atan(L / Rr))
  print(f'{side}: yaw rate {math.degrees(om):.1f} deg/s at v {v:.3f} m/s -> radius {R:.3f} m (axle midpoint) -> '
        f'steering {res[side]:.1f} deg  ({why})', flush=True)
if res:
  print(f'STEER_LIMIT_DEG {np.mean(list(res.values())):.1f}  (bench assumes 29-40; Hiwonder 2024 build 29, 2025 build 40)')
