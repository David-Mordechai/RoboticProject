# Drive command watchdog for the MentorPi A1 (our code, runs next to the stock stack, inside the MentorPi container).
#
# Why: the stock driver keeps the last motor speed until a new command arrives. odom_publisher_node subscribes to
# cmd_vel with queue depth 1 and has no timeout, so one lost or missing stop message leaves the car driving (seen on
# car #1, 2026-11-02, docs/measurements/car1-2026-11-02.md). Any remote control over the flaky hotspot Wi-Fi needs this.
#
# How: every drive source (joystick, phone app, Nav2, our scripts) ends up as a /ros_robot_controller/set_motor
# message, so that is the one place to watch. If the last motor command was non-zero and none has arrived for
# TIMEOUT_S, publish a zero Twist on /controller/cmd_vel (stops the motors and resets odom_publisher's speed state, so
# dead-reckoned odometry stops too) and, as a backup, zero rps straight to set_motor. Then idle until the next
# non-zero command.
#
# Caveat: senders must repeat their command faster than TIMEOUT_S. Nav2 and our tools send at 20 Hz. The stock joystick
# node only publishes on joystick events, so holding a stick perfectly still for 0.5 s stops the car (move it again to
# drive on).
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from ros_robot_controller_msgs.msg import MotorState, MotorsState

# 2026-11-11: the stock joystick node sends a drive command only when the stick MOVES, so a stick held steady (e.g. straight
# reverse) was stopped every 0.5 s (15 stops in 30 s, user drive). joy_node republishes /joy at 20 Hz (autorepeat_rate) while
# the gamepad is connected, so: a held stick = a /joy message younger than JOY_FRESH_S with any axis beyond JOY_DEAD.
# If the gamepad or its 2.4 GHz link dies, /joy stops and the normal timeout applies again.
JOY_FRESH_S = 0.3
JOY_DEAD = 0.1      # the joystick node's own dead zone (min_value 0.1, joystick_control.py)
TIMEOUT_S = 0.5      # our design: 10 missed messages at the 20 Hz policy / Nav2 rate
CHECK_HZ = 20.0


class CmdWatchdog(Node):
  def __init__(self):
    super().__init__('cmd_watchdog')
    self.cmd_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)
    self.motor_pub = self.create_publisher(MotorsState, '/ros_robot_controller/set_motor', 10)
    self.create_subscription(MotorsState, '/ros_robot_controller/set_motor', self.on_motor, 50)
    self.joy_t = -1e9
    self.create_subscription(Joy, '/joy', self.on_joy, 10)
    self.last_t = time.monotonic()
    self.moving = False
    self.create_timer(1.0 / CHECK_HZ, self.check)
    self.get_logger().info(f'watching set_motor, timeout {TIMEOUT_S} s')

  def on_motor(self, msg):
    self.last_t = time.monotonic()
    self.moving = any(abs(m.rps) > 1e-6 for m in msg.data)

  def on_joy(self, msg):
    if any(abs(a) > JOY_DEAD for a in msg.axes):
      self.joy_t = time.monotonic()

  def check(self):
    now = time.monotonic(); age = now - self.last_t
    if not self.moving or age < TIMEOUT_S:
      return
    if now - self.joy_t < JOY_FRESH_S:     # stick held on a live gamepad: the last joystick command is still wanted
      return
    self.get_logger().warn(f'no drive command for {age:.2f} s while moving: stopping')
    self.cmd_pub.publish(Twist())
    zero = MotorsState()
    for i in range(4):
      m = MotorState(); m.id = i + 1; m.rps = 0.0; zero.data.append(m)
    self.motor_pub.publish(zero)
    self.moving = False


def main():
  rclpy.init()
  rclpy.spin(CmdWatchdog())


if __name__ == '__main__':
  main()
