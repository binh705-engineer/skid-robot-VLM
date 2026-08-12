#!/usr/bin/env python3
"""
keep_distance.py

Maintain distance to ctx.target_id within [min_distance_m, max_distance_m]:
back off if the person gets closer than min_distance_m, approach if they
move farther than max_distance_m, do nothing if already within the band.
Non-terminal - same lifecycle as FollowTarget (ends via FAILED on lost
target, or external preemption).
"""

import math
import time

from behavior_executive.behavior_types import BehaviorContext, BehaviorTask, BehaviorStatus
from behavior_executive.behaviors.base import BaseBehavior
from behavior_executive.motion_primitives import (
    find_track, compute_stop_pose_toward, compute_backoff_pose, send_nav2_goal,
)


class KeepDistance(BaseBehavior):
    def on_enter(self, ctx: BehaviorContext, task: BehaviorTask) -> None:
        self._last_sent_xy = None
        self._last_seen_time = time.time()

    def tick(self, ctx: BehaviorContext, task: BehaviorTask) -> BehaviorStatus:
        obj = find_track(ctx, ctx.target_id)
        lost_grace_period_sec = ctx.get_param(task, "lost_grace_period_sec", 2.0)

        if obj is not None:
            self._last_seen_time = time.time()
        elif time.time() - self._last_seen_time > lost_grace_period_sec:
            ctx.node.get_logger().warn(
                f"[KEEP_DISTANCE] target_id={ctx.target_id} lost for "
                f"> {lost_grace_period_sec}s. Failing.")
            return BehaviorStatus.FAILED

        if obj is None:
            return BehaviorStatus.RUNNING

        min_distance_m = ctx.get_param(task, "min_distance_m", 1.0)
        max_distance_m = ctx.get_param(task, "max_distance_m", 3.0)
        goal_update_min_distance_m = ctx.get_param(task, "goal_update_min_distance_m", 0.3)

        distance = math.hypot(obj.position.x, obj.position.y)

        if distance < min_distance_m:
            goal_pose = compute_backoff_pose(ctx, obj, min_distance_m)
        elif distance > max_distance_m:
            goal_pose = compute_stop_pose_toward(ctx, obj, max_distance_m)
        else:
            return BehaviorStatus.RUNNING  # already within the acceptable band, nothing to do

        if goal_pose is None:
            return BehaviorStatus.RUNNING

        gx, gy = goal_pose.pose.position.x, goal_pose.pose.position.y
        ctx.last_target_xy_map = (gx, gy)

        if self._last_sent_xy is not None:
            moved = math.hypot(gx - self._last_sent_xy[0], gy - self._last_sent_xy[1])
            if moved < goal_update_min_distance_m:
                return BehaviorStatus.RUNNING

        self._last_sent_xy = (gx, gy)
        send_nav2_goal(ctx, goal_pose, lambda status: None)
        return BehaviorStatus.RUNNING
