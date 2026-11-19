"""Freshness gate between Nav2 and the motor driver (car #1, Phase A). Nav2's collision_monitor publishes to
/cmd_vel_guard_in; this node relays each command to /controller/cmd_vel ONLY while the robot's picture of the world is
fresh, otherwise it sends zero (stop) instead and publishes /nav_guard/blocked = True.

Why (2026-11-16, docs/progress-live.md, lessons-learned.md): with the Pi overloaded, TF went stale, Nav2 kept steering
on an old pose and costmap and car #1 hit a table leg; Nav2 itself then crashed. Nav2's own transform_tolerance did not
stop the car. A gate on the command path cannot be overridden by Nav2 retries or recoveries.
If this node dies or starves, no commands reach the driver and cmd_watchdog.py zeroes the motors after 0.5 s.

Checks (arrival age = time since the last message arrived; lag = receive time - header stamp):
  /scan_raw                   arrival <= 0.35 s, lag <= 0.35 s   (measured 9.7 Hz, lag ~100 ms, max 116 ms)
  /tf odom -> base_footprint  arrival <= 0.25 s, lag <= 0.30 s   (measured 28.5 Hz, lag ~8 ms, max 144 ms)
  /tf map -> odom (AMCL)      arrival <= 0.50 s, lag + AMCL_TOL <= 0.50 s (measured 9.3 Hz, stamped ~0.9 s ahead:
                                                                   AMCL transform_tolerance 1.0, nav2_params_a.yaml)
Nominal values measured standing on 2026-11-16 (Nav2 + recorders running). Limits are OUR DESIGN ESTIMATES, about 3x the
nominal worst case. A block lifts only after everything has been fresh for RELEASE_S.
No tf2 TransformListener on purpose: a listener plus another subscription on one rclpy node busy-looped the executor
(depth_scan.py, 70-86 % of a core); /tf is read as a plain topic.
  python3 nav_guard.py"""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage

LIMITS = {'scan': (0.35, 0.35), 'odom': (0.25, 0.30), 'amcl': (0.50, 0.50)}   # (arrival s, lag s), design estimates
AMCL_TOL = 1.0      # s, AMCL transform_tolerance in nav2_params_a.yaml: map->odom is stamped this far ahead
RELEASE_S = 0.5     # s all-fresh before a block lifts (design)
CHECK_HZ = 20.0


class NavGuard(Node):
  def __init__(self):
    super().__init__('nav_guard')
    self.out = self.create_publisher(Twist, '/controller/cmd_vel', 10)
    self.blk_pub = self.create_publisher(Bool, '/nav_guard/blocked', 10)
    self.create_subscription(Twist, '/cmd_vel_guard_in', self.on_cmd, 10)
    self.create_subscription(LaserScan, '/scan_raw', self.on_scan, qos_profile_sensor_data)
    self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
    self.seen = {k: (-1e9, 1e9) for k in LIMITS}          # key -> (arrival monotonic, lag s)
    self.blocked, self.fresh_since, self.n_blocks = True, None, 0
    self.create_timer(1.0 / CHECK_HZ, self.check)
    self.get_logger().info(f'gating /cmd_vel_guard_in -> /controller/cmd_vel, limits {LIMITS}')

  def lag(self, h):
    return self.get_clock().now().nanoseconds * 1e-9 - (h.stamp.sec + h.stamp.nanosec * 1e-9)

  def on_scan(self, m):
    self.seen['scan'] = (time.monotonic(), self.lag(m.header))

  def on_tf(self, m):
    for t in m.transforms:
      p = (t.header.frame_id, t.child_frame_id)
      if p == ('odom', 'base_footprint'):
        self.seen['odom'] = (time.monotonic(), self.lag(t.header))
      elif p == ('map', 'odom'):
        self.seen['amcl'] = (time.monotonic(), self.lag(t.header) + AMCL_TOL)

  def stale(self):
    now = time.monotonic(); bad = []
    for k, (arr_max, lag_max) in LIMITS.items():
      t, lag = self.seen[k]
      if now - t > arr_max: bad.append(f'{k} silent {now - t:.2f} s')
      elif lag > lag_max: bad.append(f'{k} lag {lag:.2f} s')
    return bad

  def check(self):
    bad = self.stale(); now = time.monotonic()
    if bad:
      if not self.blocked:
        self.n_blocks += 1
        self.get_logger().warn(f'BLOCK #{self.n_blocks}: {", ".join(bad)} -> stopping')
        self.out.publish(Twist())
      self.blocked, self.fresh_since = True, None
    elif self.blocked:
      self.fresh_since = self.fresh_since or now
      if now - self.fresh_since >= RELEASE_S:
        self.blocked = False
        self.get_logger().info('all fresh again: commands pass')
    self.blk_pub.publish(Bool(data=self.blocked))

  def on_cmd(self, m):
    # check here too, not only in the 20 Hz timer: a command must never pass on data that went stale since the last tick
    self.out.publish(Twist() if self.blocked or self.stale() else m)


def main():
  rclpy.init(); rclpy.spin(NavGuard())


if __name__ == '__main__':
  main()
