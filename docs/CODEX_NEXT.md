# Next task for Codex

Written by Claude (reviewer). Read this file at the start of a session after `git pull`.

**Standing rule for every task:** a task is done only when the commit is on GitHub. Commit, run
`git push origin <branch>`, then check that `git ls-remote origin <branch>` equals `git rev-parse HEAD`.
"Everything up-to-date" means nothing new was pushed; it is not success. Reply with the pushed hash.

## Working tree note

Leave the deleted 05_c2_tall_wall_refused.png and the untracked curobo/ untouched. Stage only the
report by explicit path. No git clean/reset/restore/stash/add -A.

## Task: retry cuMotion with its own environment (branch `experiment/d435-cumotion`, time box 30 min)

The first start failed on a missing `setuptools_scm` because only Jazzy + the workspace were sourced.
Part 0 found ~/use_cumotion_rviz_env.sh, which references isaac_ros_ws and was not used.

Rules: no install, sudo, build, or edit in ~/isaac_ros_ws. Never launch the ur5e URDF as a driver:
it has a real URPositionHardwareInterface and an embedded robot IP. Keep ROS_DOMAIN_ID=20 and the
ROS_LOG_DIR / ROS_HOME under /tmp, and set PYTHONDONTWRITEBYTECODE=1 so that no cache files are
written into the workspace.

1. Read ~/use_cumotion_rviz_env.sh and summarise what it sets (venv, paths, CUDA). Do not edit it.
2. Check that its Python can `import setuptools_scm` and `import curobo`, and report the versions.
3. Source it and rerun the same planner start command from docs/D435_CUMOTION_CHECK.md.
   Report whether it reaches "cuMotion is ready for planning queries!" and GPU memory used.
4. If ready: send one plan-only MoveGroup action goal to the cuMotion server (no controller, no
   execution) and report success and planning time. This uses the ur5e description, so it is only a
   start test.
5. If it fails: paste the exact error and your best read of the cause. Do not try more fixes.

Append a "Retry" section to docs/D435_CUMOTION_CHECK.md. Stop every process you started. Commit, push,
reply in 5 lines with the hash.
