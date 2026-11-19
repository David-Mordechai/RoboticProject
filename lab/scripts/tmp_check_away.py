"""tmp_goto_check.py with the goto v6k "moving away from the route" back-up trigger switched on (evaluation only)."""
import runpy
from robotlab.tasks import mentorpi_goto_env_cfg as G
from mjlab.tasks import registry
_load = registry.load_env_cfg
def load(task, *a, **k):
  cfg = _load(task, *a, **k); cfg.actions["drive"].unstick_away = G.moving_away; return cfg
registry.load_env_cfg = load
runpy.run_path("lab/scripts/tmp_goto_check.py", run_name="__main__")
