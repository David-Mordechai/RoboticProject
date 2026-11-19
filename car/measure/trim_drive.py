# Floor test for steering trim: drive "straight" (angular.z = 0, servo at centre + saved trim) and measure the turn rate
# with the IMU gyro. Usage (in the MentorPi container): python3 trim_drive.py [SPEED] [SECONDS]
# Steering error from the bicycle model the stock driver uses (ackermann.py: theta = atan(wheelbase * w / v)):
#   delta = atan(WHEELBASE * yaw_rate / v), with v measured by the LiDAR range to whatever is straight ahead.
# Pulse per degree: 2000 / 180 (ackermann.py: servo = 1500 + 2000 * deg / 180; + = left).
# Safety: stops if anything is within STOP_M ahead or SIDE_STOP_M at the sides (+-100 deg), sends zeros at 20 Hz on
# exit, logs every set_motor message.
import math, statistics, sys, time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan
from ros_robot_controller_msgs.msg import MotorsState

WHEELBASE = 0.145     # m, ackermann.py default (Hiwonder code; drawings disagree, research/01 sec. 3)
STOP_M = 0.35         # m from the LiDAR, straight ahead (+-35 deg)
SIDE_STOP_M = 0.25    # m from the LiDAR, sides (35-100 deg each way); the car is ~0.16 m wide (research/01 sec. 2)
FRONT, ARC = math.radians(35), math.radians(100)
# Ignore hits inside the car's own outline: body x -0.104..+0.110, |y| <= 0.0795 (URDF mesh, fake_car.py BODY), LiDAR at
# x -0.010 -> farthest body point ~0.14 m. 11-11 trim3 stopped at 0.0 s on a 0.05 m 'side obstacle' = self/noise hit.
SELF_M = 0.14
speed = float(sys.argv[1]) if len(sys.argv) > 1 else 0.2
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0

rclpy.init(); n = rclpy.create_node('trim_drive')
pub = n.create_publisher(Twist, '/controller/cmd_vel', 10)
T0 = time.monotonic(); now = lambda: time.monotonic() - T0
gyro, motor, scan = [], [], {}
n.create_subscription(Imu, '/ros_robot_controller/imu_raw', lambda m: gyro.append((now(), m.angular_velocity.z)),
                      qos_profile_sensor_data)
n.create_subscription(MotorsState, '/ros_robot_controller/set_motor',
                      lambda m: motor.append((now(), [round(d.rps, 3) for d in m.data])), 50)
n.create_subscription(LaserScan, '/scan_raw', lambda m: scan.__setitem__('m', m), qos_profile_sensor_data)
ex = SingleThreadedExecutor(); ex.add_node(n)


def spin(s):
  end = time.monotonic() + s
  while time.monotonic() < end: ex.spin_once(timeout_sec=0.01)


def ranges(centre, half):
  m = scan['m']; out = []
  for i, r in enumerate(m.ranges):
    a = m.angle_min + i * m.angle_increment
    if abs(math.atan2(math.sin(a - centre), math.cos(a - centre))) <= half and max(m.range_min, SELF_M) < r < m.range_max:
      out.append(r)
  return out


def ahead():
  v = ranges(0.0, math.radians(3))
  return statistics.median(v) if v else float('nan')


def send(v):
  t = Twist(); t.linear.x = float(v); pub.publish(t)


for _ in range(10):                # DDS discovery can take a few s (11-11: 1 s was not enough)
  spin(0.5)
  if 'm' in scan: break
if 'm' not in scan: print('NO SCAN - not moving'); sys.exit(1)
def blocked(front_m, side_m):
  f = min(ranges(0.0, FRONT) or [99]); s = min(ranges(0.0, ARC) or [99])
  if f < front_m: return f'obstacle {f:.2f} m ahead'
  if s < side_m: return f'obstacle {s:.2f} m at the side'
  return None


def corridor(length, half_w):
  """Nearest LiDAR hit inside the straight path ahead: 0 < x < length, |y| < half_w (LiDAR frame, x forward)."""
  m = scan['m']; best = 99.0
  for i, r in enumerate(m.ranges):
    if not (max(m.range_min, SELF_M) < r < m.range_max): continue
    a = m.angle_min + i * m.angle_increment; x, y = r * math.cos(a), r * math.sin(a)
    if 0 < x < length and abs(y) < half_w: best = min(best, x)
  return best


# Start check = the PATH, not a cone: 11-11 a +-35 deg cone at 1.2 m refused a clear hall because of a pillar 0.6 m to
# the side. Path length = run distance + stop distance + 0.2 m; half width 0.25 m (car ~0.16 m wide, research/01 sec. 2,
# + drift margin). Side clearance as before (0.40 m within +-100 deg); the in-run stops are unchanged.
need = speed * secs + STOP_M + 0.2
f = corridor(need, 0.25); s = min(ranges(0.0, ARC) or [99])
why = (f'obstacle {f:.2f} m ahead in the path (need {need:.2f} m)' if f < need else
       f'obstacle {s:.2f} m at the side' if s < 0.40 else None)
if why: print(f'NOT STARTING: {why}'); sys.exit(1)
t_b = now(); spin(2.0)
bias = statistics.mean(g for t, g in gyro if t >= t_b)
d0, t_go = ahead(), now()
print(f'gyro bias {bias:+.4f} rad/s; {d0:.2f} m clear ahead; driving {speed} m/s for {secs} s')
reason = 'done'
try:
  end = time.monotonic() + secs; nxt = time.monotonic()
  while time.monotonic() < end:
    if time.monotonic() >= nxt: send(speed); nxt += 0.05
    ex.spin_once(timeout_sec=0.01)
    why = blocked(STOP_M, SIDE_STOP_M)
    if why: reason = f'STOPPED: {why}'; break
finally:
  t_stop = now()
  for _ in range(10): send(0.0); spin(0.05)
spin(1.0)
d1 = ahead()
run = [(t, g - bias) for t, g in gyro if t_go + 0.7 <= t <= t_stop]       # skip the acceleration
yaw = sum((g - bias) * 0.0206 for t, g in gyro if t_go <= t <= t_stop + 0.5)  # 48.5 Hz IMU (measurements sec. 2)
rate = statistics.mean(g for t, g in run) if run else float('nan')
v_meas = (d0 - d1) / (t_stop - t_go) if math.isfinite(d0 - d1) else float('nan')
v_use = v_meas if v_meas > 0.05 else speed
delta = math.degrees(math.atan(WHEELBASE * rate / v_use))
print(f'{reason}: {t_stop - t_go:.1f} s, LiDAR distance {d0 - d1:.2f} m (mean {v_meas:.2f} m/s)')
print(f'heading change {math.degrees(yaw):+.1f} deg; steady yaw rate {rate:+.4f} rad/s ({"left" if rate > 0 else "right"})')
print(f'steering error {delta:+.2f} deg -> trim change {-round(delta * 2000 / 180):+d} pulse units' if math.isfinite(delta)
      else 'steering error: not enough data (run too short)')
print(f'set_motor: {len(motor)} msgs, last {motor[-1][1] if motor else None}')
n.destroy_node(); rclpy.shutdown()
