"""Synthetic check of depth_scan.height_above_floor (run where rclpy is importable, e.g. robotlab-nav2:humble):
floor seen with the car #1 error (pitched up so the floor rises ~2 cm per metre, measured 0.04-0.06 m at 1-4 m by
dsz.py 2026-11-09), 1 cm depth noise; obstacles: a chair-base bar 6 cm high at 1.0 m, a 4 cm cable bundle at 1.5 m.
Old rule (z > FLOOR_Z 0.02) vs new (height above fitted plane > FLOOR_H 0.03)."""
import math, os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import depth_scan as D

rng = np.random.default_rng(0)
x = rng.uniform(0.3, 4.0, 20000); y = rng.uniform(-1.0, 1.0, 20000)
pitch = math.radians(1.2)
z = x * math.tan(pitch) + 0.005 + rng.normal(0, 0.01, len(x))           # floor as the mis-calibrated camera sees it
floor = np.c_[x, y, z]
bx = rng.uniform(0.98, 1.02, 400); by = rng.uniform(-0.3, 0.3, 400); bz = rng.uniform(0.0, 0.06, 400) + bx * math.tan(pitch)
cx = rng.uniform(1.48, 1.52, 200); cy = rng.uniform(0.2, 0.4, 200); cz = rng.uniform(0.0, 0.04, 200) + cx * math.tan(pitch)
pts = np.r_[floor, np.c_[bx, by, bz], np.c_[cx, cy, cz]].astype(np.float32)
lab = np.r_[np.zeros(len(floor)), np.ones(400), 2 * np.ones(200)]
valid = np.ones(len(pts), bool)
h, plane = D.height_above_floor(pts, valid)
old = (pts[:, 2] > D.FLOOR_Z) & (pts[:, 2] < D.BAND_TOP)
new = (h > (D.FLOOR_H if plane else D.FLOOR_Z)) & (h < D.BAND_TOP)
print('plane', None if plane is None else tuple(round(v, 4) for v in plane), '(true a = %.4f)' % math.tan(pitch))
for name, k in (('old', old), ('new', new)):
  fl = k[lab == 0]; leak_near = fl[(x < 1.0)].mean() * 100; leak_far = fl[(x >= 1.0)].mean() * 100
  print(f'{name}: floor kept {fl.mean() * 100:5.1f}% (<1 m {leak_near:4.1f}%, >=1 m {leak_far:4.1f}%), '
        f'chair bar kept {k[lab == 1].mean() * 100:5.1f}%, cable kept {k[lab == 2].mean() * 100:5.1f}%')
# What Nav2 sees: the /depth_scan bins (1 deg). Phantom = a bin whose nearest kept point is floor;
# obstacle covered = share of the bar's / cable's bins that have a kept point of that obstacle.
ang = np.degrees(np.arctan2(pts[:, 1], pts[:, 0])).astype(int)
for name, k in (('old', old), ('new', new)):
  near = {}                      # same rule as depth_scan.robust_bin_min: the BIN_MIN_PTS-th nearest point per bin
  by = {}
  for i in np.where(k)[0]:
    by.setdefault(ang[i], []).append((math.hypot(pts[i, 0], pts[i, 1]), lab[i]))
  for bb, lst in by.items():
    lst.sort()
    if len(lst) >= D.BIN_MIN_PTS: near[bb] = lst[D.BIN_MIN_PTS - 1]
  phantom = sum(1 for v in near.values() if v[1] == 0)
  bar_bins = set(ang[lab == 1]); cab_bins = set(ang[lab == 2])
  cov = lambda bins, L: np.mean([b in near and near[b][1] == L for b in bins]) * 100
  print(f'{name} scan: {phantom} phantom floor bins of {len(set(ang))}; chair bar bins covered {cov(bar_bins, 1):.0f}%, '
        f'cable bins covered {cov(cab_bins, 2):.0f}%')
  if name == 'new':
    assert plane is not None, 'no floor plane'
    assert phantom == 0, 'floor still makes phantom bins'
    assert cov(bar_bins, 1) >= 90, 'chair base not seen in the scan'

# Case 2 (2026-11-11, car #1 in the office): the laminate floor gives no depth at all; only furniture bases are seen,
# chair bases / table legs 0-8 cm high at 1.15-3 m. No plane may be accepted there, and the bases must stay obstacles.
ox = rng.uniform(1.15, 3.0, 300); oy = rng.uniform(-1.0, 1.0, 300); oz = rng.uniform(0.0, 0.08, 300)
opts = np.c_[ox, oy, oz].astype(np.float32)
h2, plane2 = D.height_above_floor(opts, np.ones(len(opts), bool))
kept2 = ((h2 > (D.FLOOR_H if plane2 else D.FLOOR_Z)) & (h2 < D.BAND_TOP)).mean() * 100
print('office (no floor return): plane', plane2, f'furniture-base points kept {kept2:.0f}% (fixed cut keeps z > 2 cm)')
assert plane2 is None, 'plane accepted through furniture bases'
print('PASS')
