"""tmp_goto_check.py with a wider shield corridor (evaluation only, 2026-11-06): SHIELD_HALF_W from argv (default 0.14)."""
import runpy, sys
from robotlab.tasks import mentorpi_explore_env_cfg as X
X.SHIELD_HALF_W = float(sys.argv.pop()) if len(sys.argv) > 3 else 0.14
runpy.run_path("lab/scripts/tmp_goto_check.py", run_name="__main__")
