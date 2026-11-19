# Apply or revert the low-load settings inside the MentorPi container (called by lowload_apply.sh / revert.sh).
#   python3 lowload_patch.py apply|revert
import os, re, shutil, sys

BACKUP = '/home/ubuntu/shared/lowload_backup'
WS = '/home/ubuntu/ros2_ws'
FILES = {  # backup name -> file
  'bringup.launch.py': f'{WS}/src/bringup/launch/bringup.launch.py',
  'bringup.launch.py.install': f'{WS}/install/bringup/share/bringup/launch/bringup.launch.py',
  'ascamera.launch.py': f'{WS}/src/peripherals/launch/include/ascamera.launch.py',
}
APP_OLD = '            start_app_launch,\n'
APP_NEW = '            # start_app_launch,  # off: unused app nodes (lowload 2026-11-02)\n'


def real_files():
  """The files that exist, skipping the install copy when it is the src file (symlink install)."""
  out = {}
  for name, p in FILES.items():
    if not os.path.exists(p):
      continue
    if name.endswith('.install') and os.path.samefile(p, FILES['bringup.launch.py']):
      continue
    out[name] = p
  return out


if sys.argv[1] == 'apply':
  os.makedirs(BACKUP, exist_ok=True)
  for name, p in real_files().items():
    b = os.path.join(BACKUP, name)
    if not os.path.exists(b):
      shutil.copy2(p, b)
    s = open(p).read()
    if name.startswith('bringup') and APP_NEW not in s:
      assert s.count(APP_OLD) == 1, 'start_app_launch entry not found in ' + p
      s = s.replace(APP_OLD, APP_NEW)
    if name.startswith('ascamera'):
      s = re.sub(r'\{"fps": \d+\},', '{"fps": 15},', s, count=1)
    open(p, 'w').write(s)
    print('patched', p)
elif sys.argv[1] == 'revert':
  for name, p in real_files().items():
    b = os.path.join(BACKUP, name)
    if os.path.exists(b):
      shutil.copy2(b, p); print('restored', p)
