# Next task for Codex

Written by Claude (reviewer). Read this file at the start of a session after `git pull`.

**Standing rule for every task:** a task is done only when the commit is on GitHub. Commit, run
`git push origin <branch>`, then check that `git ls-remote origin <branch>` equals `git rev-parse HEAD`.
"Everything up-to-date" means nothing new was pushed; it is not success. Reply with the pushed hash.

## Local pre-merge status

No pending task

## Working tree note

Leave the deleted 05_c2_tall_wall_refused.png and the untracked curobo/ untouched. Stage files by
explicit path only. No git clean/reset/restore/stash/add -A.

## Task: last cuMotion attempt, clean upstream Python in our copy (branch `experiment/d435-cumotion`, time box 30 min)

Extensions now build and load. The planner stops in a local, uncommitted edit of update_kinematics.py
in ~/isaac_ros_ws. Previous rules still apply: no sudo, no pip, no change to ~/curobo_ros_env or
~/isaac_ros_ws, never launch the ur5e URDF as a driver, ROS_DOMAIN_ID=20, logs under /tmp.

1. First check a side effect: the JIT wrote to the shared ~/.cache/torch_extensions/py312_cu128/.
   Using the env from scripts/run_curobo_v2_viser.sh, import the project's cuRobo 0.8 and construct its
   MotionPlanner as our demo does (no viewer). Report whether it still works, or recompiles, and whether it
   uses the same cache names. Do not delete the cache.
2. Save `git -C ~/isaac_ros_ws/src/isaac_ros_cumotion diff` to the report in a collapsed section. It is
   someone's work, so read only, and summarise what the two edits try to do.
3. Copy the Python package to ~/cumotion_copy/isaac_ros_cumotion_py/isaac_ros_cumotion using committed
   HEAD content (`git archive` or `git show HEAD:<file>`), so it has neither local edit. Add it first on
   PYTHONPATH in scripts/use_cumotion_copy_env.sh. Confirm which file Python imports.
4. Rerun the planner start command. Report whether it reaches "cuMotion is ready for planning queries!"
   and GPU memory used. If ready, send one plan-only MoveGroup goal (no execution) and report success and time.
5. On failure, paste the exact error and your read of the cause, then stop. No further fixes: Claude will
   decide whether to keep this path or move to a clean Isaac ROS 4.6.

Append a "Clean Python attempt" section to docs/D435_CUMOTION_CHECK.md. Stop every process you started.
Commit by path, push, reply in 5 lines with the hash.
