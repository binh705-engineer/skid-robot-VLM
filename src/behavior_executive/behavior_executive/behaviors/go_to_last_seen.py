#!/usr/bin/env python3
"""
go_to_last_seen.py

Navigate to ctx.last_target_xy_map - the last known map-frame position of
the target before it was lost (written by ApproachTarget / FollowTarget /
KeepDistance while they were running). Terminal: succeeds/fails based on
Nav2's result. Typically the first step in a tracking behavior's
on_failure recovery chain.
"""

from geometry_msgs.msg import PoseStamped

from behavior_executive.behavior_types import BehaviorContext, BehaviorTask, BehaviorStatus
from behavior_executive.behaviors.base import BaseBehavior
from behavior_executive.motion_primitives import send_nav2_goal


class GoToLastSeen(BaseBehavior):
    def on_enter(self, ctx: BehaviorContext, task: BehaviorTask) -> None:
        self._result_status = None

        if ctx.last_target_xy_map is None:
            ctx.node.get_logger().warn(
                "[GO_TO_LAST_SEEN] no last known position was recorded for this "
                "target. Failing immediately.")
            self._result_status = 6  # treat as ABORTED
            return

        map_frame = ctx.default_params.get("map_frame", "map")
        x, y = ctx.last_target_xy_map
        pose = PoseStamped()
        pose.header.frame_id = map_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0  # orientation doesn't matter much for this stop
        send_nav2_goal(ctx, pose, self._on_nav2_result)

    def tick(self, ctx: BehaviorContext, task: BehaviorTask) -> BehaviorStatus:
        if self._result_status is None:
            return BehaviorStatus.RUNNING
        return BehaviorStatus.SUCCEEDED if self._result_status == 4 else BehaviorStatus.FAILED

    def _on_nav2_result(self, status_code: int):
        self._result_status = status_code
