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

## Task: compile cuRobo extensions in OUR OWN copy (branch `experiment/d435-cumotion`, time box 40 min)

The retry failed with an undefined torch symbol in geom_cu: the prebuilt CUDA extensions do not match
the venv's PyTorch, and the fallback JIT compiles inside ~/isaac_ros_ws, which we must not modify.
Fix it in a copy, leaving the original untouched.

Rules: no sudo; no pip install or any change to ~/curobo_ros_env (our demos use it); no write to
~/isaac_ros_ws or its scripts. Never launch the ur5e URDF as a driver. ROS_DOMAIN_ID=20, logs under /tmp.

1. Report the venv's torch version and its CUDA version.
2. Copy only `~/isaac_ros_ws/src/isaac_ros_cumotion/curobo_core/curobo` to `~/cumotion_copy/curobo`.
3. Add scripts/use_cumotion_copy_env.sh: source ~/use_cumotion_rviz_env.sh, then put
   ~/cumotion_copy/curobo/src first on PYTHONPATH and set TORCH_CUDA_ARCH_LIST="8.6" (RTX 3080).
4. With that env, `import curobo`, confirm its path is in ~/cumotion_copy, and let JIT compile there.
   Report compile time, and confirm with `git -C ~/isaac_ros_ws/src/isaac_ros_cumotion status --short`
   that the original shows no new changes.
5. Rerun the same planner start command. Report whether it reaches "cuMotion is ready for planning
   queries!" and the GPU memory used. If ready, send one plan-only MoveGroup goal (no execution) and
   report success and planning time.
6. On failure, paste the exact error and your read of the cause. Do not try further fixes.

Append a "Copy build" section to docs/D435_CUMOTION_CHECK.md. Stop every process you started.
Commit by path, push, reply in 5 lines with the hash.
