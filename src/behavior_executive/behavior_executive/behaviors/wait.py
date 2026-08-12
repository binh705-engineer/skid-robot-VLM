#!/usr/bin/env python3
"""
wait.py

Stand still, send no Nav2 goal. Succeeds when either:
  - `timeout_sec` elapses (if set and until_target_visible is not set), or
  - `until_target_visible` is true and ctx.target_id reappears in the
    tracks cache before `timeout_sec` (if set) elapses.
Fails only if `until_target_visible` is set AND `timeout_sec` is also set
and elapses without the target reappearing. If neither param is set, this
behavior runs forever (waits until externally preempted).
"""

import time

from behavior_executive.behavior_types import BehaviorContext, BehaviorTask, BehaviorStatus
from behavior_executive.behaviors.base import BaseBehavior
from behavior_executive.motion_primitives import find_track


class Wait(BaseBehavior):
    def on_enter(self, ctx: BehaviorContext, task: BehaviorTask) -> None:
        self._start_time = time.time()

    def tick(self, ctx: BehaviorContext, task: BehaviorTask) -> BehaviorStatus:
        timeout_sec = ctx.get_param(task, "timeout_sec", None)
        until_target_visible = ctx.get_param(task, "until_target_visible", False)

        if until_target_visible and find_track(ctx, ctx.target_id) is not None:
            return BehaviorStatus.SUCCEEDED

        if timeout_sec is not None and (time.time() - self._start_time) > timeout_sec:
            return BehaviorStatus.FAILED if until_target_visible else BehaviorStatus.SUCCEEDED

        return BehaviorStatus.RUNNING
