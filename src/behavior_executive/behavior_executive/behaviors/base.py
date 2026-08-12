#!/usr/bin/env python3
"""
base.py

Common interface every behavior implements. behavior_executor_node only
ever calls these 3 methods on a behavior instance - it never needs to know
the internals of any specific behavior.
"""

from behavior_executive.behavior_types import BehaviorContext, BehaviorTask, BehaviorStatus


class BaseBehavior:
    def on_enter(self, ctx: BehaviorContext, task: BehaviorTask) -> None:
        """Called exactly once when the behavior starts. Send the first goal here if needed."""
        raise NotImplementedError

    def tick(self, ctx: BehaviorContext, task: BehaviorTask) -> BehaviorStatus:
        """Called periodically (see check_period_sec param). Must return
        RUNNING / SUCCEEDED / FAILED - never blocks."""
        raise NotImplementedError

    def on_exit(self, ctx: BehaviorContext, task: BehaviorTask) -> None:
        """Called exactly once when leaving the behavior, for ANY reason
        (success, failure, or preemption by a new incoming command).
        Default: cancel any goal still in flight so the next behavior
        starts from a clean slate. Override only if a behavior needs extra
        cleanup beyond this."""
        from behavior_executive.motion_primitives import cancel_current_goal
        cancel_current_goal(ctx)
