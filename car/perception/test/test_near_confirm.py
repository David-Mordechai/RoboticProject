"""Synthetic + replay check of depth_scan.confirm_near (run where rclpy is importable, e.g. in the MentorPi container).
Synthetic: a one-frame phantom at 0.30 m must be dropped; an obstacle the car approaches (0.60 -> 0.30 m at 5 cm/frame,
bearing drifting 1 deg/frame) must never be dropped; the very first frame's near bins are dropped (one-frame delay).
Replay (optional): python3 test_near_confirm.py BAG_DIR ... -> per bag, near bins before/after the rule."""
import os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import depth_scan as D

N = D.SCAN_BINS
far = np.full(N, 1.2, dtype=np.float32)

# 1. single-frame phantom
cur = far.copy(); cur[70:73] = 0.30
assert D.confirm_near(cur, far).sum() == 3, 'phantom not dropped'
# 2. approaching obstacle, bearing drifting 1 bin per frame
prev = None; dropped = 0
for k, r in enumerate(np.arange(0.60, 0.29, -0.05)):
  cur = far.copy(); cur[60 + k:63 + k] = r
  if prev is not None: dropped += int(D.confirm_near(cur, prev).sum())
  prev = cur
assert dropped == 0, f'approaching obstacle dropped in {dropped} bins'
# 3. first frame: no history -> near bins dropped (one frame delay, by design)
cur = far.copy(); cur[10] = 0.35
assert D.confirm_near(cur, None).sum() == 1
# 4. obstacle that stays near (standing in front of a wall at 0.35 m) is kept every frame after the first
assert D.confirm_near(cur, cur.copy()).sum() == 0
print('synthetic: ok')

if len(sys.argv) > 1:
  import rosbag2_py
  from rclpy.serialization import deserialize_message
  from sensor_msgs.msg import LaserScan
  for B in sys.argv[1:]:
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=B, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/depth_scan'])); prev = None; fr = before = after = kept_frames = 0
    while r.has_next():
      _, d, _ = r.read_next(); m = deserialize_message(d, LaserScan)
      cur = np.array(m.ranges, dtype=np.float32); cur[~(cur > m.range_min)] = np.inf
      near = cur < D.NEAR_R; drop = D.confirm_near(cur, prev); prev = cur; fr += 1
      before += int(near.any()); after += int((near & ~drop).any()); kept_frames += int((near & ~drop).sum() > 0)
    print(f'{B.split("/")[-2]}: {fr} frames, frames with a near (< {D.NEAR_R} m) bin: {before} -> {after} after the rule')
