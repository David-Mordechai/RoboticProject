from mjlab.tasks.registry import register_mjlab_task

from robotlab.tasks.office.office_env_cfg import office_env_cfg, office_ppo_runner_cfg

register_mjlab_task(
  task_id="Robotlab-Office-MentorPiA1",
  env_cfg=office_env_cfg(),
  play_env_cfg=office_env_cfg(play=True),
  rl_cfg=office_ppo_runner_cfg(),
)
