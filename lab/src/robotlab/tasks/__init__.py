"""Task registry for mjlab, loaded through the `mjlab.tasks` entry point (pyproject.toml).

Layout follows Microduck's mjlab_microduck: robot/<robot>_constants.py, tasks/mdp.py for shared terms,
one tasks/<robot>_<task>_env_cfg.py per task, all registered here.
"""

from mjlab.tasks.registry import register_mjlab_task

from .mentorpi_follow_env_cfg import MentorPiFollowRlCfg, make_mentorpi_follow_env_cfg
from .mentorpi_office_env_cfg import MentorPiOfficeRlCfg, make_mentorpi_office_env_cfg

register_mjlab_task(
  task_id="Robotlab-Follow-MentorPiA1",
  env_cfg=make_mentorpi_follow_env_cfg(),
  play_env_cfg=make_mentorpi_follow_env_cfg(play=True),
  rl_cfg=MentorPiFollowRlCfg,
)

register_mjlab_task(
  task_id="Robotlab-Office-MentorPiA1",
  env_cfg=make_mentorpi_office_env_cfg(),
  play_env_cfg=make_mentorpi_office_env_cfg(play=True),
  rl_cfg=MentorPiOfficeRlCfg,
)
