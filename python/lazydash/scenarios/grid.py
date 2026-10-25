"""Grid placement of blocks, as in the paper's scene generator."""
from __future__ import annotations

import random

import numpy as np

GRID_STEP = 0.095      # spacing between block centres
FOOTPRINT = 0.04       # block footprint used for the exclusion test


def grid_points(rng: random.Random, n: int, include, exclude=(), step=GRID_STEP, footprint=FOOTPRINT):
    """`n` distinct grid points from the `include` rectangles, avoiding `exclude`.
    Rectangles are ((xmin, xmax), (ymin, ymax))."""
    candidates = []
    for (x0, x1), (y0, y1) in include:
        for x in np.arange(x0, x1 + step, step):
            for y in np.arange(y0, y1 + step, step):
                h = footprint / 2
                if any(x + h > ex0 and x - h < ex1 and y + h > ey0 and y - h < ey1
                       for (ex0, ex1), (ey0, ey1) in exclude):
                    continue
                candidates.append((float(x), float(y)))
    if len(candidates) < n:
        raise ValueError(f"only {len(candidates)} grid points for {n} blocks")
    return rng.sample(candidates, n)


def variant_suffix(joint_limits: str, grasp_symmetry: bool, grasp_set: str = "paper") -> str:
    # Scene-name suffix for the non-default options.
    return (("" if joint_limits == "paper" else f"-{joint_limits}-limits") + ("-sym" if grasp_symmetry else "")
            + ("" if grasp_set == "paper" else f"-{grasp_set}"))


def round_robin(n: int, slots: int) -> list[int]:
    """How many blocks each of `slots` regions gets when n blocks are dealt in turn."""
    return [n // slots + (1 if i < n % slots else 0) for i in range(slots)]
