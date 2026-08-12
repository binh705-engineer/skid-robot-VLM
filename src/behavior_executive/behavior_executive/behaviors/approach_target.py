#!/usr/bin/env python3
"""
approach_target.py

Drive to `stop_distance_m` from ctx.target_id, facing it, and STOP
(SUCCEEDED) as soon as Nav2 reports the goal reached. This is a TERMINAL
behavior - it does not keep following afterwards even if the target keeps
moving. See follow_target.py for the non-terminal version.

This is exactly the logic that used to be the entire coordinate_mapper_node.py.
"""

import math
import time

from behavior_executive.behavior_types import BehaviorContext, BehaviorTask, BehaviorStatus
from behavior_executive.behaviors.base import BaseBehavior
from behavior_executive.motion_primitives import find_track, compute_stop_pose_toward, send_nav2_goal


class ApproachTarget(BaseBehavior):
    def on_enter(self, ctx: BehaviorContext, task: BehaviorTask) -> None:
        self._result_status = None    # None = pending, 4 = SUCCEEDED, other = failed
        self._last_sent_xy = None
        self._last_seen_time = time.time()

    def tick(self, ctx: BehaviorContext, task: BehaviorTask) -> BehaviorStatus:
        if self._result_status == 4:
            return BehaviorStatus.SUCCEEDED
        if self._result_status is not None:
            return BehaviorStatus.FAILED

        obj = find_track(ctx, ctx.target_id)
        lost_grace_period_sec = ctx.get_param(task, "lost_grace_period_sec", 2.0)

        if obj is not None:
            self._last_seen_time = time.time()
        elif time.time() - self._last_seen_time > lost_grace_period_sec:
            ctx.node.get_logger().warn(
                f"[APPROACH_TARGET] target_id={ctx.target_id} lost for "
                f"> {lost_grace_period_sec}s. Failing.")
            return BehaviorStatus.FAILED

        if obj is None:
            return BehaviorStatus.RUNNING  # still within the grace period, keep waiting

        stop_distance_m = ctx.get_param(task, "stop_distance_m", 2.5)
        goal_update_min_distance_m = ctx.get_param(task, "goal_update_min_distance_m", 0.3)

        goal_pose = compute_stop_pose_toward(ctx, obj, stop_distance_m)
        if goal_pose is None:
            return BehaviorStatus.RUNNING  # TF hiccup, retry next tick

        gx, gy = goal_pose.pose.position.x, goal_pose.pose.position.y
        ctx.last_target_xy_map = (gx, gy)  # so GO_TO_LAST_SEEN/REPLAN can use it if this behavior fails later

        if self._last_sent_xy is not None:
            moved = math.hypot(gx - self._last_sent_xy[0], gy - self._last_sent_xy[1])
            if moved < goal_update_min_distance_m:
                return BehaviorStatus.RUNNING  # goal is close enough to the last one sent, don't spam Nav2

        self._last_sent_xy = (gx, gy)
        send_nav2_goal(ctx, goal_pose, self._on_nav2_result)
        return BehaviorStatus.RUNNING

    def _on_nav2_result(self, status_code: int):
        self._result_status = status_code
