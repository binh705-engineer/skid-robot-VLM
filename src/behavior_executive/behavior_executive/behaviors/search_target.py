#!/usr/bin/env python3
"""
search_target.py

STUB - NOT YET IMPLEMENTED.

Visiting a list of candidate search waypoints needs a decision on where
that list comes from (a hardcoded list of room-center poses set at map-
recording time? a cheap frontier pick from the existing occupancy map?) -
this was flagged as an open question and deliberately deferred.

For now this behavior fails immediately, so a task tree's on_exhausted
chain (e.g. RETURN_HOME) still runs predictably instead of the executor
hanging on an unimplemented step.
"""

from behavior_executive.behavior_types import BehaviorContext, BehaviorTask, BehaviorStatus
from behavior_executive.behaviors.base import BaseBehavior


class SearchTarget(BaseBehavior):
    def on_enter(self, ctx: BehaviorContext, task: BehaviorTask) -> None:
        ctx.node.get_logger().error(
            "[SEARCH_TARGET] not yet implemented (waypoint source undecided). "
            "Failing immediately, falling through to on_exhausted if defined.")

    def tick(self, ctx: BehaviorContext, task: BehaviorTask) -> BehaviorStatus:
        return BehaviorStatus.FAILED
