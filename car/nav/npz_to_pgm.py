"""Saved car map (.npz from car/measure/map_dump.py: grid 0 free / 100 occupied / -1 unknown, res, origin) -> Nav2
map_server .pgm + .yaml (trinary). Same conversion as car/nav/offline/plan_check_a.py.

  python car/nav/npz_to_pgm.py car/maps/office_goto/goto6_map_after.npz car/maps/office_goto/goto6
"""
import sys

import numpy as np
import yaml

src, out = sys.argv[1], sys.argv[2]
d = np.load(src)
g, res, org = d["grid"], float(d["res"]), d["origin"]
img = np.full(g.shape, 205, np.uint8)
img[g == 0] = 254
img[g == 100] = 0
with open(out + ".pgm", "wb") as f:      # map_server: row 0 = top (max y)
  f.write(f"P5\n{g.shape[1]} {g.shape[0]}\n255\n".encode())
  f.write(img[::-1].tobytes())
name = out.replace("\\", "/").split("/")[-1]
yaml.safe_dump({"image": name + ".pgm", "mode": "trinary", "resolution": res,
                "origin": [float(org[0]), float(org[1]), 0.0], "negate": 0, "occupied_thresh": 0.65,
                "free_thresh": 0.25}, open(out + ".yaml", "w"))
print(f"wrote {out}.pgm/.yaml: {g.shape[1]}x{g.shape[0]} cells, res {res}, origin {org}")
