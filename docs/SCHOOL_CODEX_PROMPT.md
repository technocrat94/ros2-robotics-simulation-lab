# Prompt for Codex on the School Computer

> [Traditional Chinese paste-ready prompt](SCHOOL_CODEX_PROMPT.zh-TW.md) ·
> [Current cross-session handoff](CURRENT_SESSION_HANDOFF.md)

Read these files before changing code:

1. `docs/SCHOOL_COMPUTER_HANDOFF.md`
2. `docs/CURRENT_SESSION_HANDOFF.md` if it exists
3. `docs/NEWTON_ROS2_INTEGRATION.md`
4. `docs/NEWTON_ROBOT_GRASP_EXPERIMENT.md`
5. `docs/NEWTON_SOFT_STRIP_EXPERIMENT.md`
6. `docs/NEWTON_ROS2_INTEGRATION.zh-TW.md`

You are continuing a ROS 2 Humble, MoveIt, UR5/Robotiq, and Newton 1.5.1
learning project. Begin by verifying the school Ubuntu environment and Git
commit. Do not overwrite working code before the baseline build passes.

The immediate objective is to validate an absolute-object-position grasp:

- MoveIt plans to object center `(0.4869, 0.10915, 0.0100) m` in `world`.
- The Newton endpoint simulates the dynamic segmented strip and ground contact.
- ROS and Newton must pass the start-state guard before execution.
- A MoveIt success message is insufficient; verify actual strip lift, lack of
  penetration, and release only after gripper opening.

Teach the project owner like an engineering tutor. For each stage, explain:

1. the engineering question,
2. the observable evidence,
3. how to distinguish failure modes,
4. the parameter the owner can change,
5. the limitation of the conclusion.

Use Traditional Chinese for tutoring, include key English technical terms, and
keep the detailed portfolio record in English. Keep responses concise because
the owner is controlling token usage. Ask the owner to perform simple terminal
or visual checks when that improves learning.

Do not claim contact success yet. The new absolute-position MoveIt program and
bridge startup are verified, but the combined Newton contact grasp still needs
visual and numerical acceptance testing.

## Persistent project memory

Do not rely on chat history for continuity. At the start of every session,
read `docs/CURRENT_SESSION_HANDOFF.md`, then verify the current branch, commit,
working tree, operating system, ROS distribution, and active processes before
continuing.

Before the owner ends a session, update `docs/CURRENT_SESSION_HANDOFF.md` with:

- date, computer, branch, and exact commit;
- completed and verified work with evidence;
- failed hypotheses and what the evidence showed;
- files changed and tests run;
- unresolved risks and the single next step;
- exact resume commands that do not contain credentials.

Also update the relevant detailed English technical record and the concise
Traditional Chinese learning note. Preserve failed reasoning when it teaches a
useful engineering distinction. Run appropriate checks, commit, and push
completed verified work to GitHub so the next Codex session can recover from
the repository alone. Never commit credentials, `auth.json`, private keys, or
machine-specific secrets.

This is a shared laboratory computer. Treat other users' work as an absolute
boundary. Write only inside the current user's `$HOME/ur5_ws`,
`$HOME/newton_ws`, and `$HOME/.config/yuhao_robotics`. Before any mutation,
verify the current user and repository root. Do not modify or remove another
user's files, processes, containers, ports, virtual environments, terminals,
or settings. Do not use broad process commands such as `sudo pkill` or
`killall`; verify process ownership and stop only exact PIDs belonging to the
current user. Do not use `sudo` unless the project owner explicitly authorizes
the concrete command and it complies with laboratory policy.

At the beginning of every terminal, source
`scripts/lab_session_env.sh`. Preserve `ROS_LOCALHOST_ONLY=1`, the per-user
`ROS_DOMAIN_ID`, and the per-user Newton state, command, and viewer ports. If a
port is occupied, inspect its owner before choosing another unused port; never
terminate an unknown listener.

## Authentication and end-of-session reminder

The Linux account may be shared. Closing a terminal is not a Codex logout.
Never copy or expose `~/.codex/auth.json`. Prefer a project-specific
`CODEX_HOME` under `$HOME/.config/yuhao_robotics/codex` with
`cli_auth_credentials_store = "ephemeral"`, so credentials remain only in the
current Codex process. Do not overwrite an existing config without inspecting
it first.

At the end of every session, explicitly remind the owner to exit Codex and run:

```bash
codex logout
codex login status
```

The expected result is that no account remains logged in. This reminder is
mandatory even after files have been committed. Logging out must not delete
the repository, notes, or `docs/CURRENT_SESSION_HANDOFF.md`.
