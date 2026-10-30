from mjlab.tasks.registry import register_mjlab_task

from robotlab.tasks.follow.follow_env_cfg import follow_env_cfg, follow_ppo_runner_cfg

register_mjlab_task(
  task_id="Robotlab-Follow-MentorPiA1",
  env_cfg=follow_env_cfg(),
  play_env_cfg=follow_env_cfg(play=True),
  rl_cfg=follow_ppo_runner_cfg(),
)
