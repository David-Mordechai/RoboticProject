# Static sensor characterisation: rates, LiDAR geometry/noise, IMU noise/bias, camera intrinsics. Car must be still.
import time, math, json, numpy as np, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Imu, CameraInfo, Image
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
DUR = 20.0
rclpy.init(); n = rclpy.create_node('sensor_stats')
stamps = {}; scans = []; imus = {}; info = {}; static_tf = []
def rec(name):
    stamps.setdefault(name, []).append(time.monotonic())
def on_scan(m):
    rec('scan_raw')
    if len(scans) < 400: scans.append((m, np.array(m.ranges, dtype=np.float64)))
def on_imu(name):
    def f(m):
        rec(name)
        imus.setdefault(name, []).append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z,
                                          m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
    return f
def on_info(name):
    def f(m):
        rec(name); info[name] = dict(w=m.width, h=m.height, K=list(m.k), D=list(m.d), model=m.distortion_model, frame=m.header.frame_id)
    return f
def on_img(name):
    def f(m):
        rec(name); info[name] = dict(w=m.width, h=m.height, enc=m.encoding, frame=m.header.frame_id)
    return f
def on_static(m): static_tf.extend(m.transforms)
q = qos_profile_sensor_data
n.create_subscription(LaserScan, '/scan_raw', on_scan, q)
for t in ['/imu', '/ros_robot_controller/imu_raw', '/imu_corrected']:
    n.create_subscription(Imu, t, on_imu(t), q)
for t in ['/ascamera/camera_publisher/rgb0/camera_info', '/ascamera/camera_publisher/depth0/camera_info']:
    n.create_subscription(CameraInfo, t, on_info(t), q)
for t in ['/ascamera/camera_publisher/rgb0/image', '/ascamera/camera_publisher/depth0/image_raw']:
    n.create_subscription(Image, t, on_img(t), q)
n.create_subscription(Odometry, '/odom_raw', lambda m: rec('odom_raw'), q)
n.create_subscription(Odometry, '/odom', lambda m: rec('odom'), q)
n.create_subscription(TFMessage, '/tf_static', on_static, QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL))
t0 = time.monotonic()
while time.monotonic() - t0 < DUR: rclpy.spin_once(n, timeout_sec=0.05)
out = {}
for k, v in stamps.items():
    v = np.array(v); dt = np.diff(v) if len(v) > 1 else np.array([0])
    out[k] = dict(count=len(v), hz=round((len(v) - 1) / (v[-1] - v[0]), 2) if len(v) > 1 else 0,
                  dt_ms_p50=round(1e3 * float(np.median(dt)), 1), dt_ms_p95=round(1e3 * float(np.percentile(dt, 95)), 1))
if scans:
    from collections import Counter
    lens = Counter(len(r) for _, r in scans); L = lens.most_common(1)[0][0]
    keep = [(mm, r) for mm, r in scans if len(r) == L]; m = keep[0][0]
    R = np.stack([r for _, r in keep])
    valid = np.isfinite(R) & (R >= m.range_min) & (R <= m.range_max)
    Rn = np.where(valid, R, np.nan)
    per_ray_std = np.nanstd(Rn, axis=0); per_ray_mean = np.nanmean(Rn, axis=0)
    always = valid.all(axis=0)
    bins = [(0, 0.5), (0.5, 1), (1, 2), (2, 4), (4, 12)]
    std_by_range = {f'{a}-{b}m': (round(float(np.nanmedian(per_ray_std[always & (per_ray_mean >= a) & (per_ray_mean < b)])) * 1000, 1),
                                 int((always & (per_ray_mean >= a) & (per_ray_mean < b)).sum())) for a, b in bins}
    out['lidar'] = dict(frame=m.header.frame_id, n_rays=len(m.ranges), angle_min_deg=round(math.degrees(m.angle_min), 2),
        angle_max_deg=round(math.degrees(m.angle_max), 2), increment_deg=round(math.degrees(m.angle_increment), 4),
        range_min=m.range_min, range_max=m.range_max, scan_time=m.scan_time, time_increment=m.time_increment,
        has_intensities=len(m.intensities) > 0, scans_used=len(keep), n_rays_hist=dict(sorted(lens.items())),
        valid_fraction=round(float(valid.mean()), 3),
        flicker_fraction=round(float((valid.any(axis=0) & ~valid.all(axis=0)).mean()), 3),
        std_mm_median_by_range_and_nrays=std_by_range,
        nearest_mean_m=round(float(np.nanmin(per_ray_mean)), 3))
for k, v in imus.items():
    a = np.array(v)
    out['imu ' + k] = dict(mean=[round(float(x), 4) for x in a.mean(0)], std=[round(float(x), 5) for x in a.std(0)],
                           acc_norm=round(float(np.linalg.norm(a[:, :3].mean(0))), 4))
out['info'] = info
out['tf_static'] = [dict(parent=t.header.frame_id, child=t.child_frame_id,
    xyz=[round(t.transform.translation.x, 4), round(t.transform.translation.y, 4), round(t.transform.translation.z, 4)],
    q=[round(t.transform.rotation.x, 4), round(t.transform.rotation.y, 4), round(t.transform.rotation.z, 4), round(t.transform.rotation.w, 4)])
    for t in static_tf]
print(json.dumps(out, indent=1))
n.destroy_node(); rclpy.shutdown()
