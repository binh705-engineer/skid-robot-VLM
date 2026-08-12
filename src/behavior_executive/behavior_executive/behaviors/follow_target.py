#!/usr/bin/env python3
"""
follow_target.py

Same geometry as ApproachTarget, but NEVER succeeds on its own - it keeps
re-issuing goals as the target moves, indefinitely. It only ends via
FAILED (target lost) or external preemption (the executor starts a new
task, which calls on_exit()). This is the "follow me" behavior, as opposed
to the one-shot "walk up to me" behavior (ApproachTarget).
"""

import math
import time

from behavior_executive.behavior_types import BehaviorContext, BehaviorTask, BehaviorStatus
from behavior_executive.behaviors.base import BaseBehavior
from behavior_executive.motion_primitives import find_track, compute_stop_pose_toward, send_nav2_goal


class FollowTarget(BaseBehavior):
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
                f"[FOLLOW_TARGET] target_id={ctx.target_id} lost for "
                f"> {lost_grace_period_sec}s. Failing.")
            return BehaviorStatus.FAILED

        if obj is None:
            return BehaviorStatus.RUNNING

        stop_distance_m = ctx.get_param(task, "stop_distance_m", 2.5)
        goal_update_min_distance_m = ctx.get_param(task, "goal_update_min_distance_m", 0.3)

        goal_pose = compute_stop_pose_toward(ctx, obj, stop_distance_m)
        if goal_pose is None:
            return BehaviorStatus.RUNNING

        gx, gy = goal_pose.pose.position.x, goal_pose.pose.position.y
        ctx.last_target_xy_map = (gx, gy)

        if self._last_sent_xy is not None:
            moved = math.hypot(gx - self._last_sent_xy[0], gy - self._last_sent_xy[1])
            if moved < goal_update_min_distance_m:
                return BehaviorStatus.RUNNING

        self._last_sent_xy = (gx, gy)
        # Deliberately ignore Nav2's SUCCEEDED here (unlike ApproachTarget) -
        # reaching the stop point once does not end this behavior, since the
        # target may keep moving and we need to keep following.
        send_nav2_goal(ctx, goal_pose, lambda status: None)
        return BehaviorStatus.RUNNING
