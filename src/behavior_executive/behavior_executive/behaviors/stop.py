#!/usr/bin/env python3
"""
stop.py

Immediate halt: cancel any in-flight Nav2 goal AND publish a direct zero
cmd_vel (Nav2's own cancel is not guaranteed to be instantaneous). Always
succeeds - this is a best-effort safety action, not something that can fail.
"""

from behavior_executive.behavior_types import BehaviorContext, BehaviorTask, BehaviorStatus
from behavior_executive.behaviors.base import BaseBehavior
from behavior_executive.motion_primitives import cancel_current_goal, publish_cmd_vel_zero


class Stop(BaseBehavior):
    def on_enter(self, ctx: BehaviorContext, task: BehaviorTask) -> None:
        cancel_current_goal(ctx)
        publish_cmd_vel_zero(ctx)

    def tick(self, ctx: BehaviorContext, task: BehaviorTask) -> BehaviorStatus:
        # Keep publishing zero velocity for at least one more tick in case
        # Nav2's controller was mid-cycle when the goal was cancelled.
        publish_cmd_vel_zero(ctx)
        return BehaviorStatus.SUCCEEDED
