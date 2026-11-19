#!/usr/bin/env python3
"""v5 gates with remembered rod height and a sampled braking safety plane."""
import curobo_moving_rod_v42 as base
from curobo_moving_rod_v5_rules import FloorPlane, HeightMemory


def run_case(config, settings, baseline, mode, **options):
    memory = HeightMemory(baseline['border_rule']['fixed_unknown_top_m']) if mode.startswith('E') else None
    return base.run_case(config, settings, baseline, mode, height_memory=memory,
                         plane_factory=FloorPlane, **options)


if __name__ == '__main__':
    base.REPORT = base.v2.RESULTS_DIR / 'moving_rod_v5.gates.json'
    base.BATCH = base.v2.RESULTS_DIR / 'moving_rod_v5.json'
    expected = {mode: 55 if mode.startswith('E') else 35 for mode in base.v4.GROUPS}
    def watch(args):
        return base.watch(args, memory_factory=HeightMemory, plane_factory=FloorPlane, version='5')

    raise SystemExit(base.main(run=run_case, resume=False, version='5', expected_wall_s=expected, watch_modes=watch))
