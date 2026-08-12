#!/usr/bin/env python3
"""
replan.py

Cancel the current Nav2 goal and immediately resend the SAME pose
(ctx.last_target_xy_map), forcing Nav2 to recompute a path - useful when
stuck against a moving obstacle that Nav2's own internal recovery
behaviors haven't cleared. Only meaningful as a recovery step after a
tracking behavior already computed a goal there.
"""

from geometry_msgs.msg import PoseStamped

from behavior_executive.behavior_types import BehaviorContext, BehaviorTask, BehaviorStatus
from behavior_executive.behaviors.base import BaseBehavior
from behavior_executive.motion_primitives import send_nav2_goal


class Replan(BaseBehavior):
    def on_enter(self, ctx: BehaviorContext, task: BehaviorTask) -> None:
        self._result_status = None

        if ctx.last_target_xy_map is None:
            ctx.node.get_logger().warn("[REPLAN] no previous goal position to resend. Failing.")
            self._result_status = 6
            return

        map_frame = ctx.default_params.get("map_frame", "map")
        x, y = ctx.last_target_xy_map
        pose = PoseStamped()
        pose.header.frame_id = map_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        send_nav2_goal(ctx, pose, self._on_nav2_result)

    def tick(self, ctx: BehaviorContext, task: BehaviorTask) -> BehaviorStatus:
        if self._result_status is None:
            return BehaviorStatus.RUNNING
        return BehaviorStatus.SUCCEEDED if self._result_status == 4 else BehaviorStatus.FAILED

    def _on_nav2_result(self, status_code: int):
        self._result_status = status_code
