"""Summarise a joystick drive recording (2026-11-11): commanded speed / steering (from /controller/cmd_vel, driver rule
delta = atan(0.145 w / v), clip 40 deg) vs measured yaw rate (/imu), per 0.5 s window, plus straight-run continuity.
Turn check: for an Ackermann car yaw = v tan(delta) / L, so measured / predicted yaw should be ~1 (v is COMMANDED; the
real speed is lower at low commands, so <1 is expected; ~0 or a wrong sign = the old rear-wheel fight).
  python3 joy_drive.py RUN/bag"""
import math, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
cmd, imu = [], []
while r.has_next():
  tp, data, t = r.read_next(); t *= 1e-9
  if tp == '/controller/cmd_vel': m = deserialize_message(data, Twist); cmd.append((t, m.linear.x, m.angular.z))
  elif tp == '/imu': imu.append((t, deserialize_message(data, Imu).angular_velocity.z))
cmd, imu = np.array(cmd), np.array(imu); t0 = imu[0, 0]
print(f'{len(cmd)} drive commands over {imu[-1,0]-t0:.0f} s; commands/s while driving: '
      f'{np.sum(np.abs(cmd[:,1])>1e-3)/max(1e-9, np.ptp(cmd[np.abs(cmd[:,1])>1e-3,0])):.1f}')
# hold-last-command state at each imu sample
idx = np.searchsorted(cmd[:, 0], imu[:, 0]) - 1
ok = idx >= 0
v = np.where(ok, cmd[np.clip(idx, 0, None), 1], 0); w = np.where(ok, cmd[np.clip(idx, 0, None), 2], 0)
with np.errstate(divide='ignore', invalid='ignore'):
  delta = np.where(np.abs(v) > 1e-3, np.clip(np.arctan(0.145 * w / v), -math.radians(40), math.radians(40)), 0)
pred = v * np.tan(delta) / 0.145
gz = imu[:, 1]
moving = np.abs(v) > 0.05
print(f'driving {moving.mean()*100:.0f}% of the time; forward {np.mean(v>0.05)*100:.0f}%, reverse {np.mean(v<-0.05)*100:.0f}%; '
      f'max cmd speed {v.max():.2f} / {v.min():.2f} m/s')
for lo, hi in ((5, 15), (15, 25), (25, 41)):
  s = moving & (np.abs(np.degrees(delta)) >= lo) & (np.abs(np.degrees(delta)) < hi)
  if s.sum() > 20:
    ratio = gz[s] / pred[s]
    print(f'steer {lo:2d}-{hi:2d} deg: {s.sum()/50:5.1f} s, measured/predicted yaw median {np.median(ratio):+.2f} '
          f'(same sign {np.mean(np.sign(gz[s])==np.sign(pred[s]))*100:.0f}%), |yaw| median {np.degrees(np.median(np.abs(gz[s]))):.0f} deg/s')
# straight runs: |delta| < 3 deg, moving, longer than 1.5 s; yaw drift
st = moving & (np.abs(np.degrees(delta)) < 3)
edges = np.flatnonzero(np.diff(np.r_[0, st.astype(int), 0])); runs = edges.reshape(-1, 2)
for a, b in runs:
  if imu[b-1, 0] - imu[a, 0] < 1.5: continue
  print(f'straight {"FWD" if v[a]>0 else "REV"} t={imu[a,0]-t0:6.1f} s, {imu[b-1,0]-imu[a,0]:4.1f} s at {np.median(v[a:b]):+.2f} m/s: '
        f'yaw drift {np.degrees(np.mean(gz[a:b])):+.1f} deg/s, cmd gaps > 0.3 s: '
        f'{int(np.sum(np.diff(cmd[(cmd[:,0]>=imu[a,0])&(cmd[:,0]<=imu[b-1,0]),0])>0.3))}')
