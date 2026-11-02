# Wheels-up actuator step test: logs command times, the driver's outgoing board messages and raw IMU, as JSON.
import time, json, rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from ros_robot_controller_msgs.msg import SetPWMServoState, PWMServoState, MotorsState
rclpy.init(); n = rclpy.create_node('step_test')
cmd = n.create_publisher(Twist, '/controller/cmd_vel', 10)
srv = n.create_publisher(SetPWMServoState, '/ros_robot_controller/pwm_servo/set_state', 10)
log = dict(imu=[], cmds=[], motor_msgs=[], servo_msgs=[])
T0 = time.monotonic(); now = lambda: time.monotonic() - T0
n.create_subscription(Imu, '/ros_robot_controller/imu_raw', lambda m: log['imu'].append(
    [now(), m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z,
     m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]), qos_profile_sensor_data)
n.create_subscription(MotorsState, '/ros_robot_controller/set_motor', lambda m: log['motor_msgs'].append(
    [now()] + [d.rps for d in m.data]), 10)
n.create_subscription(SetPWMServoState, '/ros_robot_controller/pwm_servo/set_state', lambda m: log['servo_msgs'].append(
    [now(), list(m.state[0].position)]), 10)
for _ in range(50):
    if cmd.get_subscription_count() and srv.get_subscription_count() > 1: break
    rclpy.spin_once(n, timeout_sec=0.1)
def wait(s):
    t = time.monotonic() + s
    while time.monotonic() < t: rclpy.spin_once(n, timeout_sec=0.005)
def steer(deg):
    s = PWMServoState(); s.id = [3]; s.position = [int(1500 + 2000 * deg / 180)]
    m = SetPWMServoState(); m.state = [s]; m.duration = 0.02
    log['cmds'].append([now(), 'steer', deg]); srv.publish(m)
def drive(v):
    t = Twist(); t.linear.x = float(v)
    log['cmds'].append([now(), 'drive', v]); cmd.publish(t)
try:
    steer(0); wait(2.0)                      # baseline
    for d in [30, 0, -30, 0, 30, 0, -30, 0]:
        steer(d); wait(1.2)
    for v in [0.2, 0.0, 0.4, 0.0, 0.2, 0.0, 0.4, 0.0]:
        drive(v); wait(1.5)
finally:
    drive(0.0); steer(0); wait(0.3)
    print(json.dumps(log))
    n.destroy_node(); rclpy.shutdown()
