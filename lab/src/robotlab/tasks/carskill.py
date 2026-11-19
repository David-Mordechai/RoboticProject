"""The car's skill layer, vectorised for the sim (2026-11-09): a port of car/policy/explore_policy.py (kturn_step:
multi-point turn, pure pursuit near U corners / sharp bends, gentle exit; steer-around AV_*; remembered-depth checks;
the model's steering smoothing before the skill). Used by the honest exam (lab/scripts/tmp_goto_check.py carskill:)
and by lab/scripts/record_clip.py --carskill, so both run the same code. Audited line by line against the car code
(2026-11-09); any change here must be made in the car node too, and the other way round.

make_policy(env, stu) -> (pol, KS): pol(obs) -> actions for the drive term; KS: the skill state per car.
"""
import math
import os as _os

import torch

from robotlab.tasks import mentorpi_explore_env_cfg as T
from robotlab.tasks import mentorpi_navigate_env_cfg as N
from robotlab.tasks.mdp import car_pose


def make_policy(env, stu):
  KT_ON, KT_OFF, KT_V, KT_EXIT_V, KT_COOL = 100.0, 40.0, 0.25, 0.30, 1.0
  KT_NEAR, PP_T, PP_LD, PP_ALAT = 0.8, 1.5, (0.6, 0.9), 0.8
  KT_BEND, KT_HELD_S, KT_HELD_DEG, KT_STUCK = 60.0, 0.1, 45.0, 1.0
  _n, _dev = env.num_envs, env.device
  import os as _os
  KS = {"on": torch.zeros(_n, dtype=torch.bool, device=_dev), "dir": torch.ones(_n, device=_dev),
        "cool": torch.zeros(_n, device=_dev), "stuck": torch.zeros(_n, device=_dev),
        "av": torch.zeros(_n, device=_dev), "avs": torch.zeros(_n, device=_dev)}
  AV_ON = _os.environ.get("ROBOTLAB_CARSKILL_AVOID", "1") == "1"
  AV_V, AV_T = 0.25, 0.6
  MEM_REV = _os.environ.get("ROBOTLAB_CARSKILL_MEMREV", "1") == "1"
  MEM_FWD = _os.environ.get("ROBOTLAB_CARSKILL_MEMFWD", "1") == "1"
  AV_CAND = (-1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0)   # 0.0: straight but slowly (reach 0.40 m vs 0.58 m at 0.7 m/s)
  env._robotlab_manoeuvre = KS["on"]
  _drv = env.action_manager.get_term("drive")
  MC = torch.zeros(_n, device=_dev)                  # the model's smoothed steering command (car: self.steer_cmd)
  def _blocked(v, st, man):                        # car blocked(): turn reaches while turning, else the driving ones
    env._robotlab_manoeuvre = man
    return T.shield(env, v, st, pure=True)
  def pol(o):
    dt = env.step_dt
    rs = env.episode_length_buf <= 1                 # new episode: the car node starts with the skill off
    KS["on"] = KS["on"] & ~rs; KS["cool"] = torch.where(rs, 0.0, KS["cool"]); KS["stuck"] = torch.where(rs, 0.0, KS["stuck"]); KS["av"] = torch.where(rs, 0.0, KS["av"])
    a = stu(o).clone().clamp(-1.0, 1.0)              # = car action_to_twist: clipped to [-1, 1]
    vmax, vrev, lim = _drv.cfg.v_max, max(_drv.cfg.v_reverse, 1e-3), _drv.steer_max
    # = car: the MODEL's steering is smoothed (steer_ema, then steer_cmd_rate) BEFORE the skill; the skill's own
    # steering goes straight to the servo (audit 2026-11-09: the sim smoothed the skill's locks too, 1 s per flip)
    MC[:] = torch.where(rs, 0.0, MC)
    tgt = a[:, 1]
    if 0.0 < _drv.cfg.steer_ema < 1.0:
      tgt = MC + _drv.cfg.steer_ema * (tgt - MC)
    r_ = _drv.cfg.steer_cmd_rate
    MC[:] = MC + ((tgt - MC).clamp(-r_, r_) if r_ > 0 else (tgt - MC))
    v = torch.where(a[:, 0] >= 0, a[:, 0] * vmax, a[:, 0] * vrev); st = MC * lim
    if _drv.cfg.forward_only:
      v = v.clamp(min=0.0)                            # = car: forward_only before the skill
    xy, yaw = car_pose(env); nv_ = N.nav(env)
    pts_w = N._points_at(nv_, nv_.s[:, None] + 0.25 * torch.arange(1, 9, device=_dev))     # [n, 8, 2], 0.25..2 m on
    d = pts_w - xy[:, None]; c, s_ = torch.cos(yaw)[:, None], torch.sin(yaw)[:, None]
    pts = torch.stack([c * d[..., 0] + s_ * d[..., 1], -s_ * d[..., 0] + c * d[..., 1]], -1)   # car frame
    tx, ty = pts[:, 2, 0], pts[:, 2, 1]
    ang = torch.atan2(ty, tx).abs()
    held = getattr(env, "_robotlab_held", torch.zeros(_n, device=_dev))
    KS["cool"] = (KS["cool"] - dt).clamp(min=0)
    off = ~KS["on"]
    start = off & (KS["cool"] <= 0) & ((ang > math.radians(KT_ON)) | ((held > KT_HELD_S) & (ang > math.radians(KT_HELD_DEG))))
    KS["on"] = KS["on"] | start; KS["dir"] = torch.where(start, 1.0, KS["dir"])
    # not turning: pure pursuit near a U corner / sharp bend, gentle exit during the cooldown
    near = torch.zeros(_n, dtype=torch.bool, device=_dev)
    if hasattr(env, "_robotlab_turn_xy"):
      near = (torch.linalg.norm(env._robotlab_turn_xy - xy[:, None], dim=-1).amin(-1) < KT_NEAR) & env._robotlab_midturn_open
    d1, d2 = pts[:, 1] - pts[:, 0], pts[:, 2] - pts[:, 1]
    bend = torch.atan2(d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0], (d1 * d2).sum(-1)).abs()
    pp = ~KS["on"] & (near | (bend > math.radians(KT_BEND)))
    vx = env.scene["car"].data.root_link_lin_vel_b[:, 0]
    ld = (PP_T * vx.clamp(min=0)).clamp(*PP_LD)
    dist = torch.linalg.norm(pts, dim=-1); ge = dist >= ld[:, None]
    k = torch.where(ge.any(-1), ge.float().argmax(-1), 7)
    p_ = pts[torch.arange(_n, device=_dev), k]
    kap = 2 * p_[:, 1] / (p_ ** 2).sum(-1).clamp(min=1e-4)
    st_pp = torch.max(torch.min(torch.atan(T.WHEELBASE * kap), lim), -lim)
    v_pp = torch.sqrt(PP_ALAT / kap.abs().clamp(min=1e-3)).clamp(max=vmax)
    v = torch.where(pp, v_pp, v); st = torch.where(pp, st_pp, st)
    v = torch.where(~KS["on"] & (KS["cool"] > 0) & (v > KT_EXIT_V), KT_EXIT_V, v)
    nobody = torch.zeros(_n, dtype=torch.bool, device=_dev)
    # steer around (2026-11-09, = car AV_*): held by the stop with the route ahead -> the free steering candidate
    # (sensor arc test, driving reaches) closest to the driver's steering, slowly, committed AV_T s. Trace: worst
    # blocked cars drove straight at the obstacle, the back-up rule backed off and they drove straight back (floor goto18).
    if AV_ON:
      want = ~KS["on"] & (held > KT_HELD_S) & (ang <= math.radians(KT_HELD_DEG)) & (KS["av"] <= 0)
      vav = torch.full((_n,), AV_V, device=_dev)
      best, bcost = torch.zeros(_n, device=_dev), torch.full((_n,), 1e9, device=_dev)
      for cc in AV_CAND:
        sc = torch.full((_n,), cc, device=_dev) * lim
        free = ~_blocked(vav, sc, nobody)
        if MEM_FWD:
          free &= ~T.depth_mem_rev_blocked(env, sc, forward=True)
        cost = torch.where(free, (sc - st).abs(), 1e9)
        best = torch.where(cost < bcost, sc, best); bcost = torch.minimum(cost, bcost)
      go_av = want & (bcost < 1e8)
      # no free forward candidate -> the multi-point turn (it can reverse) starts now (2026-11-09: 37% of idle time was
      # "held with the route ahead, nothing acts" until the 0.3 s back-up)
      no_av = want & (bcost >= 1e8) & (KS["cool"] <= 0)
      KS["on"] = KS["on"] | no_av; KS["dir"] = torch.where(no_av, 1.0, KS["dir"])
      KS["av"] = torch.where(go_av, AV_T, (KS["av"] - dt).clamp(min=0)); KS["avs"] = torch.where(go_av, best, KS["avs"])
      still = (KS["av"] > 0) & ~KS["on"] & ~_blocked(vav, KS["avs"], nobody)
      KS["av"] = torch.where(still, KS["av"], 0.0)
      v = torch.where(still, AV_V, v); st = torch.where(still, KS["avs"], st)
    # turning
    model_ok = (v > 0) & ~_blocked(v, st, nobody)
    done = KS["on"] & (ang < math.radians(KT_OFF)) & model_ok
    side = torch.where(ty >= 0, 1.0, -1.0)
    kv = torch.full((_n,), KT_V, device=_dev); allon = torch.ones(_n, dtype=torch.bool, device=_dev)
    fwd_ok = ~_blocked(kv, side * lim, allon); rev_ok = ~_blocked(-kv, -side * lim, allon)
    if MEM_REV:   # = car: legs also check remembered depth hits (chair legs behind / too close for the camera)
      rev_ok &= ~T.depth_mem_rev_blocked(env, -side * lim)
      if MEM_FWD:
        fwd_ok &= ~T.depth_mem_rev_blocked(env, side * lim, forward=True)
    both = ~fwd_ok & ~rev_ok
    f2, r2 = ~_blocked(kv, -side * lim, allon), ~_blocked(-kv, side * lim, allon)
    if MEM_REV:
      r2 &= ~T.depth_mem_rev_blocked(env, side * lim)
      if MEM_FWD:
        f2 &= ~T.depth_mem_rev_blocked(env, -side * lim, forward=True)
    swap = both & (f2 | r2)
    side = torch.where(swap, -side, side); fwd_ok = torch.where(swap, f2, fwd_ok); rev_ok = torch.where(swap, r2, rev_ok)
    KS["stuck"] = torch.where(KS["on"] & ~fwd_ok & ~rev_ok, KS["stuck"] + dt, 0.0)
    give_up = KS["on"] & ~done & (KS["stuck"] > KT_STUCK)
    KS["dir"] = torch.where((KS["dir"] > 0) & ~fwd_ok & rev_ok, -1.0, torch.where((KS["dir"] < 0) & ~rev_ok & fwd_ok, 1.0, KS["dir"]))
    go = ((KS["dir"] > 0) & fwd_ok) | ((KS["dir"] < 0) & rev_ok)
    v_t = torch.where(go, KS["dir"] * KT_V, 0.0); st_t = torch.where(go, KS["dir"], 1.0) * side * lim   # = car: kt_dir * side * lim (waiting: side * lim)
    turning = KS["on"] & ~done & ~give_up
    v = torch.where(done, v.clamp(max=KT_EXIT_V), v); v = torch.where(give_up, 0.0, v)
    v = torch.where(turning, v_t, v); st = torch.where(turning, st_t, st)
    KS["cool"] = torch.where(done | give_up, KT_COOL, KS["cool"]); KS["stuck"] = torch.where(done | give_up, 0.0, KS["stuck"])
    KS["on"] = turning
    env._robotlab_manoeuvre = KS["on"]
    env._carskill_dbg = dict(ang=ang, held=held, fwd_ok=fwd_ok, rev_ok=rev_ok, v=v, st=st, vx=vx)
    # back to normalised actions
    a[:, 0] = torch.where(v >= 0, v / vmax, v / vrev); a[:, 1] = st / lim
    _drv._steer_cmd[:] = a[:, 1]                      # so the action term's smoothing / rate limit pass it unchanged
    return a
  return pol, KS
