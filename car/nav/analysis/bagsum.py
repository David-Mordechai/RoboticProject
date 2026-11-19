import sys, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
want = ['/cmd_vel_nav', '/cmd_vel_smoothed', '/controller/cmd_vel', '/collision_monitor_state']
cnt = {}; t0 = None; out = []
while r.has_next():
  topic, data, t = r.read_next()
  if t0 is None: t0 = t
  cnt[topic] = cnt.get(topic, 0) + 1
  if topic in want and cnt[topic] % 5 == 1:
    m = deserialize_message(data, get_message(types[topic]))
    s = f'v {m.linear.x:+.2f} w {m.angular.z:+.2f}' if hasattr(m, 'linear') else f'action {m.action_type} poly {m.polygon_name}'
    out.append(f'{(t - t0) / 1e9:6.1f} {topic}: {s}')
print({k: v for k, v in sorted(cnt.items())})
print('\n'.join(out[:80]))
