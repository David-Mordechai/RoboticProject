# Save the RGB camera as JPEG files, RATE per second, named by seconds since T0 (the run's start, epoch s), until the
# STOP file appears. Run inside the MentorPi container: python3 cam_saver.py OUT_DIR T0 STOP_FILE. Independent of
# web_video_server, which segfaulted mid-run (hall4, 2026-11-04).
import os, sys, time
import numpy as np, cv2, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
RATE = 2.0
out, t0, stop = sys.argv[1], float(sys.argv[2]), sys.argv[3]
os.makedirs(out, exist_ok=True)
rclpy.init(); n = rclpy.create_node('cam_saver'); last = [0.0]


def on_img(m):
  now = time.time()
  if now - last[0] < 1.0 / RATE: return
  last[0] = now
  a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
  if m.encoding == 'rgb8': a = a[:, :, ::-1]
  cv2.imwrite(os.path.join(out, f'{now - t0:06.1f}.jpg'), a, [cv2.IMWRITE_JPEG_QUALITY, 80])


n.create_subscription(Image, '/ascamera/camera_publisher/rgb0/image', on_img, qos_profile_sensor_data)
while not os.path.exists(stop):
  rclpy.spin_once(n, timeout_sec=0.2)
