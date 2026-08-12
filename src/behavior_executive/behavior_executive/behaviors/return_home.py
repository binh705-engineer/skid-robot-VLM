#!/usr/bin/env python3
"""
return_home.py

Navigate to ctx.home_pose - the robot's pose captured once at the start
of the current task (see behavior_executor_node.target_command_callback(),
which calls capture_current_pose() before starting the task tree).
Typically used as an on_exhausted fallback, e.g.:
  "find the person in the red shirt, if not found return home"
   -> APPROACH_TARGET, on_failure=[GO_TO_LAST_SEEN, ROTATE_SCAN],
      on_exhausted=[RETURN_HOME]
"""

from behavior_executive.behavior_types import BehaviorContext, BehaviorTask, BehaviorStatus
from behavior_executive.behaviors.base import BaseBehavior
from behavior_executive.motion_primitives import send_nav2_goal


class ReturnHome(BaseBehavior):
    def on_enter(self, ctx: BehaviorContext, task: BehaviorTask) -> None:
        self._result_status = None

        if ctx.home_pose is None:
            ctx.node.get_logger().error(
                "[RETURN_HOME] home_pose was never captured for this task "
                "(TF lookup likely failed at task start). Failing.")
            self._result_status = 6
            return

        send_nav2_goal(ctx, ctx.home_pose, self._on_nav2_result)

    def tick(self, ctx: BehaviorContext, task: BehaviorTask) -> BehaviorStatus:
        if self._result_status is None:
            return BehaviorStatus.RUNNING
        return BehaviorStatus.SUCCEEDED if self._result_status == 4 else BehaviorStatus.FAILED

    def _on_nav2_result(self, status_code: int):
        self._result_status = status_code
