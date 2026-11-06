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

## Task: none. Work stopped for today (2026-11-06)

Do not start anything new. Next session, Claude will write the moving-obstacle task here. It must include the simulated camera's live depth image panel in the
Viser view, as scripts/newton_sim_depth_cloud_viser.py had; the sixth-pass viewer dropped it.
