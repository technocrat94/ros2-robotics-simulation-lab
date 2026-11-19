#!/usr/bin/env python3
"""Short bar poke checkpoint and Viser demo."""
import curobo_moving_bar_poke_v5_scene as poke
from curobo_moving_bar_v5 import main, v2


if __name__ == '__main__':
    from curobo_moving_bar_v5_viewer import LivePoke

    raise SystemExit(main(scene=poke, result=v2.RESULTS_DIR/'moving_bar_poke_v5.checkpoint.json',
                          live_class=LivePoke))
