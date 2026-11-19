"""Steering limit WITHOUT the stock driver's rear-wheel differential (2026-11-11).

Why: steer_limit.py drives through odom_publisher_node, which also sets the rear wheels to v -/+ w*track/2. With the
full-lock command (w 2.0 at v 0.15) that is 0.283 / 0.017 m/s, and on car #1 the car barely moved and did not turn
(camera, runs/steer2). Suspect: since the 11-02 motor-direction fix ([0, -vl, 0, vr], ackermann.py:50) the faster wheel
is on the OUTSIDE... i.e. the wrong side, so the rear wheels fight the steering. This test sets the servo directly
(pulse as the driver computes it: 1500 + 2000*40/180 = 1944 left, 1056 right; trim -9 is applied by the board config)
and both rear wheels to the SAME forward speed, then reads the gyro. Ackermann: yaw rate = v * tan(delta) / L.
Car #1 motor ids: M2 = left rear (+rps = backward), M4 = right rear (+rps = forward)  (memory / 11-02 wheels-up test).
Run in the MentorPi container with ~/.zshrc sourced. Stops if the LiDAR sees anything within STOP_M.
  python3 steer_direct.py [ARC_S=1.5] [V=0.15]
"""
import math, sys, time
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from ros_robot_controller_msgs.msg import MotorState, MotorsState, SetPWMServoState, PWMServoState

ARC_S = float(sys.argv[1]) if len(sys.argv) > 1 else 1.5
V = float(sys.argv[2]) if len(sys.argv) > 2 else 0.15
L, WHEEL_D, STOP_M = 0.137, 0.067, 0.30          # URDF wheelbase; driver wheel diameter (ackermann.py)
rclpy.init(); n = rclpy.create_node('steer_direct'); st = {'gz': []}
mot = n.create_publisher(MotorsState, 'ros_robot_controller/set_motor', 10)
srv = n.create_publisher(SetPWMServoState, 'ros_robot_controller/pwm_servo/set_state', 10)
n.create_subscription(Imu, '/imu', lambda m: st['gz'].append((time.time(), m.angular_velocity.z)), qos_profile_sensor_data)
n.create_subscription(LaserScan, '/scan_raw', lambda m: st.__setitem__('scan', m), qos_profile_sensor_data)


def spin_all(t):
  end = time.time() + t
  while time.time() < end: rclpy.spin_once(n, timeout_sec=max(0.0, min(0.01, end - time.time())))


def near():
  m = st.get('scan')
  return min((r for r in m.ranges if m.range_min < r < m.range_max), default=9.0) if m else 9.0


def motors(v):
  rps = v / (math.pi * WHEEL_D)
  m = MotorsState(); m.data = [MotorState(id=i, rps=float(r)) for i, r in ((1, 0.0), (2, -rps), (3, 0.0), (4, rps))]
  mot.publish(m)


def servo(pos):
  s = PWMServoState(); s.id = [3]; s.position = [int(pos)]; m = SetPWMServoState(); m.state = [s]; m.duration = 0.02
  srv.publish(m)


t = time.time()
while (mot.get_subscription_count() == 0 or 'scan' not in st) and time.time() - t < 5: spin_all(0.1)
if near() < STOP_M:
  print(f'not started: something {near():.2f} m from the LiDAR'); sys.exit(1)
res = {}
for side, pos in (('left', 1944), ('right', 1056)):
  servo(pos); spin_all(0.4)                      # let the wheels reach full lock before driving
  st['gz'].clear(); t0 = time.time(); why = 'done'
  while time.time() - t0 < ARC_S:
    if near() < STOP_M: why = f'LiDAR stop {near():.2f} m'; break
    servo(pos); motors(V); spin_all(0.05)
  for _ in range(5): motors(0.0); spin_all(0.05)
  servo(1500); spin_all(0.5)
  gz = np.array([g for (tt, g) in st['gz'] if tt - t0 > 0.5])
  if len(gz) < 5:
    print(f'{side}: too short ({why})'); continue
  om = float(np.median(gz)); R = V / abs(om) if abs(om) > 1e-3 else float('inf')
  d = math.degrees(math.atan(L / math.sqrt(max(R * R - (L / 2) ** 2, 1e-6))))
  res[side] = d
  print(f'{side}: {len(gz)} gyro samples, yaw rate {math.degrees(om):+.1f} deg/s (+ = left) at commanded v {V} -> '
        f'radius {R:.3f} m -> steering {d:.1f} deg ({why})', flush=True)
print('NOTE: v is the COMMANDED wheel speed (odom_raw is dead reckoning from commands, not encoders): estimate only')
if res: print(f'STEER_LIMIT_DEG {np.mean(list(res.values())):.1f}')
