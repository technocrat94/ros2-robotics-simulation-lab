#!/usr/bin/env python3
"""Verify that Newton runtime copies match the tracked prototypes."""

import argparse
import hashlib
from pathlib import Path


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--bridge-runtime", type=Path, required=True)
    parser.add_argument("--lesson-runtime", type=Path, required=True)
    args = parser.parse_args()

    repo = args.repo.resolve()
    expected = {
        args.bridge_runtime.resolve(): (
            repo
            / "docs/experiments/newton-ros2-bridge/prototype"
            / "newton_moveit_grasp_endpoint.py"
        )
    }

    lesson_sources = (
        repo / "docs/experiments/newton-robot-grasp/prototype",
        repo / "docs/experiments/newton-soft-strip/prototype",
    )
    expected_lesson_names: set[str] = set()
    for source_dir in lesson_sources:
        for source in sorted(source_dir.glob("*.py")):
            if source.name in expected_lesson_names:
                raise SystemExit(f"duplicate lesson prototype name: {source.name}")
            expected_lesson_names.add(source.name)
            expected[(args.lesson_runtime / source.name).resolve()] = source

    mismatches = []
    for runtime, prototype in expected.items():
        if not runtime.is_file():
            mismatches.append(f"missing runtime copy: {runtime}")
        elif digest(runtime) != digest(prototype):
            mismatches.append(
                f"runtime differs: {runtime} != {prototype.relative_to(repo)}"
            )

    if args.lesson_runtime.is_dir():
        actual_lesson_names = {
            path.name for path in args.lesson_runtime.glob("*.py") if path.is_file()
        }
        for name in sorted(actual_lesson_names - expected_lesson_names):
            mismatches.append(f"unexpected runtime lesson: {args.lesson_runtime / name}")

    if mismatches:
        print("NEWTON_RUNTIME_SYNC_FAILED")
        for mismatch in mismatches:
            print(f"  - {mismatch}")
        return 1

    print(f"NEWTON_RUNTIME_SYNC_OK files={len(expected)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
