"""Nav2 test bench: a fake MentorPi A1 for Nav2 (ROS 2 Humble), so Phase A settings are tested off the floor.

Subscribes /controller/cmd_vel (Twist) like the stock driver; publishes /odom, TF odom->base_footprint, /scan_raw
(ray-cast in a saved car map), static TFs (map->odom identity: no AMCL; base_footprint->lidar_frame). Every value is
from a source or marked estimate:
  - command -> wheels, as Hiwonder's driver (ackermann.py:25-45 on car #1, read 2026-11-09): steering
    delta = atan(0.145 w / v), clipped to +-40 deg; |v| below the motor dead band 0.03 m/s does not move the car
    (open-issues 448, measured). Steering limit: vendor 29-40 deg (research/01 sec. table); car #1 MEASURED 2026-11-11 ~28 L / ~24 R
    (arc_icp.py) -> --steer-max / --steer-max-right.
  - kinematics: bicycle model, wheelbase 0.137 m (URDF ack.xacro axles x +0.0668 / -0.0703), reference point
    base_footprint = midway between the axles (TF on car #1).
  - delays: command latency 0.045 s (middle of lab randomization.py LATENCY_RANGE_S 0.020-0.070); servo
    60 deg / 0.16 s x 0.8 (randomization.py rating x STEER_RATE_SCALE 0.6-1.0 middle).
  - speed response: first-order, time constant 0.15 s (ESTIMATE, not measured).
  - LiDAR: MS200 at base_footprint +(-0.010, 0, 0.163) (TF car #1), 0.05-12 m, 10 Hz (measured 9-10 Hz), 1 deg steps
    (ESTIMATE: car #1 scans had ~430-450 valid hits per turn), range noise sigma 0.01 m (ESTIMATE).
  - contact: the body box (URDF mesh x -0.104..+0.110, |y| <= 0.0795) overlapping an occupied map cell -> the car
    stops there and /bench/contacts counts it.
  python3 fake_car.py MAP.npz X Y YAW_DEG [--steer-max 33]
"""
import argparse
import collections
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Int32
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

p = argparse.ArgumentParser()
p.add_argument("map"); p.add_argument("x", type=float); p.add_argument("y", type=float); p.add_argument("yaw", type=float)
p.add_argument("--steer-max", type=float, default=33.0, help="deg, physical limit LEFT (and right unless --steer-max-right)")
p.add_argument("--steer-max-right", type=float, default=None,
               help="deg, right limit if not symmetric; car #1 MEASURED 2026-11-11: ~28 left / ~24 right (arc_icp.py)")
a, _ = p.parse_known_args()

L_DRV, CLIP_DRV = 0.145, math.radians(40)          # driver constants (ackermann.py)
WHEELBASE = 0.137                                   # URDF
V_DEAD, LAT, SERVO_RATE, TAU_V = 0.03, 0.045, math.radians(60) / 0.16 * 0.8, 0.15
LIDAR = (-0.010, 0.0)
BODY = (-0.104, 0.110, 0.0795)
DT = 0.01

d = np.load(a.map); G, RES, ORG = d["grid"], float(d["res"]), d["origin"]
OCC = G == 100


def occ(x, y):
  i, j = int((y - ORG[1]) / RES), int((x - ORG[0]) / RES)
  return not (0 <= i < G.shape[0] and 0 <= j < G.shape[1]) or OCC[i, j]


class FakeCar(Node):
  def __init__(self):
    super().__init__("fake_car")
    self.x, self.y, self.th = a.x, a.y, math.radians(a.yaw)
    self.v, self.delta, self.cmd = 0.0, 0.0, collections.deque()
    self.contacts, self.in_contact = 0, False
    self.create_subscription(Twist, "/controller/cmd_vel", self.on_cmd, 10)
    self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
    self.scan_pub = self.create_publisher(LaserScan, "/scan_raw", 10)
    self.cpub = self.create_publisher(Int32, "/bench/contacts", 10)
    self.imu_pub = self.create_publisher(Imu, "/imu", 10)          # gyro yaw rate only (for steer_limit.py tests)
    self.tf = TransformBroadcaster(self); st = StaticTransformBroadcaster(self)
    st.sendTransform([self.tfm("map", "odom", 0, 0, 0), self.tfm("base_footprint", "lidar_frame", *LIDAR, 0, z=0.163),
                      self.tfm("base_footprint", "base_link", 0, 0, 0, z=0.07)])
    self.x0, self.y0, self.th0 = self.x, self.y, self.th
    self.create_timer(DT, self.step); self.create_timer(0.1, self.scan); self.create_timer(0.02, self.pub_odom)
    self.target = (0.0, 0.0)

  def tfm(self, parent, child, x, y, th, z=0.0):
    t = TransformStamped(); t.header.stamp = self.get_clock().now().to_msg()
    t.header.frame_id, t.child_frame_id = parent, child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = float(x), float(y), z
    t.transform.rotation.z, t.transform.rotation.w = math.sin(th / 2), math.cos(th / 2)
    return t

  def on_cmd(self, m):
    now = self.get_clock().now().nanoseconds / 1e9
    v, w = m.linear.x, m.angular.z
    delta = None                                   # driver: no new steering when w == 0 or v == 0 keeps servo
    if abs(w) >= 1e-8 and abs(v) > 1e-9:
      delta = max(-CLIP_DRV, min(CLIP_DRV, math.atan(L_DRV * w / v)))
    elif abs(w) < 1e-8:
      delta = 0.0
    self.cmd.append((now + LAT, v, delta))

  def body_hits(self, x, y, th):
    c, s = math.cos(th), math.sin(th)
    for bx in np.linspace(BODY[0], BODY[1], 6):
      for by in np.linspace(-BODY[2], BODY[2], 4):
        if occ(x + c * bx - s * by, y + s * bx + c * by): return True
    return False

  def step(self):
    now = self.get_clock().now().nanoseconds / 1e9
    while self.cmd and self.cmd[0][0] <= now:
      _, v, dl = self.cmd.popleft()
      self.target = (0.0 if abs(v) < V_DEAD else v, self.target[1] if dl is None else dl)
    tv, td = self.target
    td = max(-math.radians(a.steer_max if a.steer_max_right is None else a.steer_max_right),
             min(math.radians(a.steer_max), td))      # delta > 0 = left (w > 0)
    self.delta += max(-SERVO_RATE * DT, min(SERVO_RATE * DT, td - self.delta))
    self.v += (tv - self.v) * DT / TAU_V
    # bicycle about the midpoint of the axles: slip angle beta = atan(tan(delta) / 2)
    beta = math.atan(math.tan(self.delta) / 2)
    nx = self.x + self.v * math.cos(self.th + beta) * DT
    ny = self.y + self.v * math.sin(self.th + beta) * DT
    nth = self.th + self.v * math.cos(beta) * math.tan(self.delta) / WHEELBASE * DT
    if self.body_hits(nx, ny, nth):
      if not self.in_contact: self.contacts += 1; self.cpub.publish(Int32(data=self.contacts))
      self.in_contact = True; self.v = 0.0
    else:
      self.in_contact = False; self.x, self.y, self.th = nx, ny, nth

  def pub_odom(self):
    # odom frame = map frame (identity map->odom); the real odom drifts, ignored here
    o = Odometry(); o.header.stamp = self.get_clock().now().to_msg(); o.header.frame_id = "odom"
    o.child_frame_id = "base_footprint"
    o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
    o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.th / 2), math.cos(self.th / 2)
    o.twist.twist.linear.x = self.v
    o.twist.twist.angular.z = self.v * math.cos(math.atan(math.tan(self.delta) / 2)) * math.tan(self.delta) / WHEELBASE
    im = Imu(); im.header = o.header; im.header.frame_id = "imu_link"; im.angular_velocity.z = o.twist.twist.angular.z
    self.imu_pub.publish(im)
    self.odom_pub.publish(o); self.tf.sendTransform(self.tfm("odom", "base_footprint", self.x, self.y, self.th))

  def scan(self):
    c, s = math.cos(self.th), math.sin(self.th)
    lx, ly = self.x + c * LIDAR[0], self.y + s * LIDAR[0]
    angs = np.radians(np.arange(0, 360, 1.0))
    r = np.full(len(angs), np.inf); steps = np.arange(0.05, 12.0, RES / 2)
    for k, aa in enumerate(angs):
      wx, wy = lx + steps * math.cos(self.th + aa), ly + steps * math.sin(self.th + aa)
      i, j = ((wy - ORG[1]) / RES).astype(int), ((wx - ORG[0]) / RES).astype(int)
      ok = (i >= 0) & (i < G.shape[0]) & (j >= 0) & (j < G.shape[1])
      hit = np.where(~ok | OCC[np.clip(i, 0, G.shape[0] - 1), np.clip(j, 0, G.shape[1] - 1)])[0]
      if len(hit): r[k] = steps[hit[0]] + np.random.normal(0, 0.01)
    m = LaserScan(); m.header.stamp = self.get_clock().now().to_msg(); m.header.frame_id = "lidar_frame"
    m.angle_min, m.angle_max, m.angle_increment = 0.0, float(angs[-1]), math.radians(1.0)
    m.range_min, m.range_max, m.scan_time = 0.05, 12.0, 0.1
    m.ranges = [float(v) for v in r]; self.scan_pub.publish(m)


rclpy.init(); rclpy.spin(FakeCar())
