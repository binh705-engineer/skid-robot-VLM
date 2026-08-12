#!/usr/bin/env python3
"""
rotate_scan.py

Spin in place using Nav2's Spin action to widen the camera's field of view
and try to re-acquire ctx.target_id. Succeeds as soon as the target
reappears in the tracks cache (even mid-spin - there's no need to wait for
the whole rotation to finish). Fails if the spin completes without the
target reappearing.
"""

import math

from behavior_executive.behavior_types import BehaviorContext, BehaviorTask, BehaviorStatus
from behavior_executive.behaviors.base import BaseBehavior
from behavior_executive.motion_primitives import find_track, send_spin_goal, cancel_current_goal


class RotateScan(BaseBehavior):
    def on_enter(self, ctx: BehaviorContext, task: BehaviorTask) -> None:
        self._spin_done = False
        scan_angle_deg = ctx.get_param(task, "scan_angle_deg", 360.0)
        send_spin_goal(ctx, math.radians(scan_angle_deg), self._on_spin_result)

    def tick(self, ctx: BehaviorContext, task: BehaviorTask) -> BehaviorStatus:
        if find_track(ctx, ctx.target_id) is not None:
            return BehaviorStatus.SUCCEEDED  # reacquired mid-spin, good enough - stop early in on_exit()

        if self._spin_done:
            return BehaviorStatus.FAILED  # finished the full spin, still not found

        return BehaviorStatus.RUNNING

    def _on_spin_result(self, status_code: int):
        self._spin_done = True

    def on_exit(self, ctx: BehaviorContext, task: BehaviorTask) -> None:
        cancel_current_goal(ctx)  # stop the spin immediately if we succeeded mid-rotation
