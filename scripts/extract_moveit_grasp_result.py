#!/usr/bin/env python3
"""Extract and validate the final MOVEIT_GRASP_RESULT from an endpoint log."""

import argparse
import json
from pathlib import Path


PREFIX = "MOVEIT_GRASP_RESULT "


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--tested-commit", required=True)
    parser.add_argument("--device", required=True)
    args = parser.parse_args()

    result_lines = [
        line[len(PREFIX):]
        for line in args.log.read_text(encoding="utf-8").splitlines()
        if line.startswith(PREFIX)
    ]
    if not result_lines:
        raise SystemExit(
            f"ERROR: {args.log} contains no {PREFIX.strip()} line"
        )

    result = json.loads(result_lines[-1])
    if result.get("tested_commit") != args.tested_commit:
        raise SystemExit(
            "ERROR: result tested_commit does not match the launched revision: "
            f"{result.get('tested_commit')!r} != {args.tested_commit!r}"
        )
    if result.get("device") != args.device:
        raise SystemExit(
            "ERROR: result device does not match the selected runtime: "
            f"{result.get('device')!r} != {args.device!r}"
        )

    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(f"MOVEIT_GRASP_RESULT_JSON {args.output}")


if __name__ == "__main__":
    main()
