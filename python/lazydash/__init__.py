"""Lazy-DaSH: hypergraph-based multi-robot task and motion planning on CoMotion.

Scene tools are pure Python; the planner is the compiled module `lazydash._core`.
"""
import json

__version__ = "1.0.0"

try:
    from ._core import Params, Result, Scene, result_json, solve  # noqa: F401
except ImportError:  # not built (e.g. on Windows): scene tools still work
    pass


def stats(result) -> dict:
    """The run's statistics (stage clocks named as in the paper's runs)."""
    return json.loads(result.stats_json)
