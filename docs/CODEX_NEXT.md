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

## Task: resume the copy build (branch `experiment/d435-cumotion`, time box 30 min)

The copy failed on version detection only, because its `.git` pointer is dead. Claude added
SETUPTOOLS_SCM_PRETEND_VERSION=0.7.7 to scripts/use_cumotion_copy_env.sh. Do not edit the copy's `.git`.
All rules of the previous task still apply: no sudo, no pip, no change to ~/curobo_ros_env or
~/isaac_ros_ws, never launch the ur5e URDF as a driver, ROS_DOMAIN_ID=20, logs under /tmp.

1. Rerun the same timed import command with this env. Confirm curobo.__file__ is under ~/cumotion_copy
   and let JIT compile there. Report compile time and any .so files created (paths only).
2. Check `git -C ~/isaac_ros_ws/src/isaac_ros_cumotion status --short` still shows only the two
   modifications listed in the Copy build section.
3. Rerun the planner start command. Report whether it reaches "cuMotion is ready for planning
   queries!" and GPU memory used. If ready, send one plan-only MoveGroup goal (no execution) and
   report success and planning time.
4. On failure, paste the exact error and your read of the cause. Do not try further fixes.

Append a "Copy build retry" section to docs/D435_CUMOTION_CHECK.md. Stop every process you started.
Commit by path, push, reply in 5 lines with the hash.
