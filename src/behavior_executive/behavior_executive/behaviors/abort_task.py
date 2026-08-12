#!/usr/bin/env python3
"""
abort_task.py

Unconditional give-up: cancel everything, publish zero cmd_vel, and always
"succeed" immediately (it's a terminal safety action itself, not something
that can fail). Reaching ABORT_TASK ends the whole task tree with an
overall FAILED result reported to /kinematics/goal_status.
"""

from behavior_executive.behavior_types import BehaviorContext, BehaviorTask, BehaviorStatus
from behavior_executive.behaviors.base import BaseBehavior
from behavior_executive.motion_primitives import cancel_current_goal, publish_cmd_vel_zero


class AbortTask(BaseBehavior):
    def on_enter(self, ctx: BehaviorContext, task: BehaviorTask) -> None:
        cancel_current_goal(ctx)
        publish_cmd_vel_zero(ctx)

    def tick(self, ctx: BehaviorContext, task: BehaviorTask) -> BehaviorStatus:
        return BehaviorStatus.SUCCEEDED
