import sys, time, rclpy
from ros_robot_controller_msgs.msg import SetPWMServoState, PWMServoState
off = int(sys.argv[1]); assert -100 <= off <= 100
rclpy.init(); n = rclpy.create_node('servo_trim')
pub = n.create_publisher(SetPWMServoState, '/ros_robot_controller/pwm_servo/set_state', 10)
for _ in range(50):
    if pub.get_subscription_count() > 0: break
    time.sleep(0.1)
def send(**kw):
    s = PWMServoState(); s.id = [3]
    for k, v in kw.items(): setattr(s, k, [v])
    m = SetPWMServoState(); m.state = [s]; m.duration = 0.3
    pub.publish(m); time.sleep(0.5)
send(offset=off)
send(position=1500)
print("servo 3 offset", off, "-> centred", flush=True)
n.destroy_node(); rclpy.shutdown()
