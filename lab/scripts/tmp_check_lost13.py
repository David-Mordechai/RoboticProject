"""tmp_goto_check.py with MIDTURN_LOST = 1.3 m instead of 1.0 (evaluation only, 2026-11-05): how many mid-hallway
failures are late turns that would finish with a little more room (still short of the nearest junction, >= 1.4 m)."""
import runpy, sys
from robotlab.tasks import mentorpi_goto_env_cfg as G
G.MIDTURN_LOST = float(sys.argv.pop()) if len(sys.argv) > 3 else 1.3
runpy.run_path("lab/scripts/tmp_goto_check.py", run_name="__main__")
