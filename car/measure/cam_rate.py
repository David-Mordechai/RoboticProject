# Camera publish rate: A) camera_info only (cheap), B) plus full images. Prints Hz and inter-frame gaps.
import sys, time, numpy as np, rclpy
from rclpy.qos import qos_profile_sensor_data as Q
from sensor_msgs.msg import CameraInfo, Image, LaserScan, Imu
from nav_msgs.msg import Odometry
P = '/ascamera/camera_publisher/'
def run(with_images, dur=10.0):
    n = rclpy.create_node('cam_rate'); st = {}
    def rec(k): return lambda m: st.setdefault(k, []).append(time.monotonic())
    n.create_subscription(CameraInfo, P + 'rgb0/camera_info', rec('rgb_info'), Q)
    n.create_subscription(CameraInfo, P + 'depth0/camera_info', rec('depth_info'), Q)
    n.create_subscription(LaserScan, '/scan_raw', rec('scan'), Q)
    n.create_subscription(Imu, '/ros_robot_controller/imu_raw', rec('imu_raw'), Q)
    n.create_subscription(Odometry, '/odom_raw', rec('odom_raw'), Q)
    if with_images:
        n.create_subscription(Image, P + 'rgb0/image', rec('rgb_img'), Q)
        n.create_subscription(Image, P + 'depth0/image_raw', rec('depth_img'), Q)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 2.0: rclpy.spin_once(n, timeout_sec=0.05)   # let discovery settle
    st.clear(); t0 = time.monotonic()
    while time.monotonic() - t0 < dur: rclpy.spin_once(n, timeout_sec=0.05)
    for k, v in sorted(st.items()):
        d = np.diff(v) * 1e3
        print(f"  {k:10} {(len(v)-1)/(v[-1]-v[0]):5.1f} Hz  n {len(v):4d}  gap p50 {np.median(d):5.0f} ms  p95 {np.percentile(d,95):5.0f} ms  max {d.max():5.0f} ms", flush=True)
    n.destroy_node()
rclpy.init()
print('A) camera_info only'); run(False)
print('B) + rgb and depth images'); run(True)
rclpy.shutdown()
