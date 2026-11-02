import json, sys, numpy as np
L = json.load(open(sys.argv[1]))
I = np.array(L['imu']); t = I[:, 0]
base = (t > 0.3) & (t < 1.9)
mu, sd = I[base, 1:].mean(0), I[base, 1:].std(0)
score = np.abs((I[:, 1:] - mu) / sd).max(1)
M = np.array(L['motor_msgs']) if L['motor_msgs'] else None
cmds = L['cmds']
for i, (tc, kind, val) in enumerate(cmds):
    tn = cmds[i+1][0] if i+1 < len(cmds) else t[-1]
    w = (t > tc) & (t < tn); tt, sc = t[w], score[w]
    above = np.where(sc > 6)[0]
    onset = round((tt[above[0]] - tc) * 1e3) if len(above) else None
    last = round((tt[above[-1]] - tc) * 1e3) if len(above) else None
    lat = None
    if kind == 'drive' and M is not None:
        after = M[M[:, 0] >= tc]
        if len(after): lat = round((after[0, 0] - tc) * 1e3, 1)
    tail = sc[tt > tn - 0.8]
    print(f"{tc:6.2f}s {kind:5} {val:5}  imu onset {onset} ms  last>6sd {last} ms  peak z {sc.max() if len(sc) else 0:6.1f}  tail med z {np.median(tail) if len(tail) else 0:5.1f}  ros->set_motor {lat} ms")
if M is not None: print('motor msgs', len(M), M[:2].round(3).tolist())
