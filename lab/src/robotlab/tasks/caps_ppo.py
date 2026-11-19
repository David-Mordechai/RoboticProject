"""PPO + CAPS action smoothness on the policy MEAN (goto v7b, 2026-11-08).

Why: the goto policy's deterministic output steered bang-bang. On a straight hall the ONNX steering output was +-4..10
(valid +-1) and flipped sign every ~0.6 s; the car's rate limit turned that into a +-15 deg zig-zag (floor goto17, sim
v6p: steering std on straights 10.8 deg). Reward penalties on the EXECUTED steering (goto v7: steer_flip, action_bound)
left it flat for 200 iterations: with an action std of 0.6-0.9 the sampled noise made almost all of the flips, so the
penalty mostly taught the policy to explore less (std 0.87 -> 0.61), not to steer smoothly.

CAPS (Mysore et al., "Regularizing Action Policies for Smooth Control with Reinforcement Learning", ICRA 2021,
arXiv 2012.06644) regularizes the policy mean directly:
  temporal: || mu(s_t) - mu(s_t+1) ||^2 over consecutive steps of the same episode
  spatial:  || mu(s) - mu(s~) ||^2, s~ = s + small noise (here: on the normalized observation)
We add a third, ours: || relu(|mu| - 1) ||^2, the mean outside the +-1 the drive clamps to (bang-bang lives there).

Implementation (ours): before rsl_rl's PPO update, caps_steps gradient steps on these losses over the rollout storage
(each on a random caps_frac of the envs), with the same optimizer and gradient clipping. Coefficients are design
choices; check with tmp_goto_check.py (steering reversals / std on straights, policy reverses).
"""

from __future__ import annotations

import torch
from torch import nn

from rsl_rl.algorithms import PPO


class CapsPPO(PPO):
  def __init__(self, *args, caps_temporal_coef: float = 0.0, caps_spatial_coef: float = 0.0,
               caps_spatial_std: float = 0.05, caps_bound_coef: float = 0.0, caps_steps: int = 4,
               caps_frac: float = 0.25, **kwargs):
    super().__init__(*args, **kwargs)
    self.caps = (caps_temporal_coef, caps_spatial_coef, caps_spatial_std, caps_bound_coef, caps_steps, caps_frac)

  def _caps_step(self) -> dict[str, float]:
    ct, cs, sd, cb, _, frac = self.caps
    obs, dones = self.storage.observations, self.storage.dones          # [T, N, ...], [T, N, 1]
    T, N = dones.shape[:2]
    envs = torch.randperm(N, device=dones.device)[: max(1, int(N * frac))]
    o = obs[:, envs]
    mu = self.actor(o.reshape(-1)).view(T, len(envs), -1)                # deterministic mean (stochastic_output False)
    same = (dones[:-1, envs, 0] == 0).float()                             # step t did not end the episode
    temporal = ((mu[1:] - mu[:-1]).pow(2).sum(-1) * same).sum() / same.sum().clamp(min=1.0)
    bound = (mu.abs() - 1.0).clamp(min=0.0).pow(2).sum(-1).mean()
    spatial = torch.zeros((), device=mu.device)
    if cs > 0:
      # perturb in normalized-observation units: the actor normalizes inside, so add noise scaled by the running std
      flat = o.reshape(-1)
      noisy = flat.clone()
      for k in noisy.keys():
        x = noisy[k]
        noisy[k] = x + sd * x.std(0, keepdim=True).clamp(min=1e-3) * torch.randn_like(x)
      spatial = (self.actor(noisy) - mu.reshape(-1, mu.shape[-1])).pow(2).sum(-1).mean()
    loss = ct * temporal + cs * spatial + cb * bound
    self.optimizer.zero_grad()
    loss.backward()
    nn.utils.clip_grad_norm_(self.actor.parameters(), self.max_grad_norm)
    self.optimizer.step()
    return {"caps_temporal": temporal.item(), "caps_spatial": spatial.item(), "caps_bound": bound.item()}

  def update(self) -> dict[str, float]:
    ct, cs, _, cb, steps, _ = self.caps
    info: dict[str, float] = {}
    if steps > 0 and (ct > 0 or cs > 0 or cb > 0):
      for _ in range(steps):
        r = self._caps_step()
        for k, v in r.items():
          info[k] = info.get(k, 0.0) + v / steps
    out = super().update()
    out.update(info)
    return out
