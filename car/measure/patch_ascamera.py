import sys
p = "/home/ubuntu/ros2_ws/src/peripherals/launch/include/ascamera.launch.py"
fps = sys.argv[1]
s = open(p).read()
for old, new in [('{"color_pcl": True},', '{"color_pcl": False},  # was True (camera fps fix 2026-11-02)'),
                 ('{"fps": 15},', '{"fps": %s},  # was 15 (camera fps fix 2026-11-02)' % fps)]:
    assert s.count(old) == 1, old
    s = s.replace(old, new)
open(p, "w").write(s); print("patched")
