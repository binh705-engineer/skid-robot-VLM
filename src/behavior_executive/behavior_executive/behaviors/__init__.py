#!/usr/bin/env python3
"""
behaviors/__init__.py

BEHAVIOR_REGISTRY maps each BehaviorType to the class implementing it.
behavior_executor_node uses this to instantiate the right behavior for
whatever BehaviorTask it needs to run next - it never imports a specific
behavior class directly.
"""

from behavior_executive.behavior_types import BehaviorType

from behavior_executive.behaviors.approach_target import ApproachTarget
from behavior_executive.behaviors.follow_target import FollowTarget
from behavior_executive.behaviors.keep_distance import KeepDistance
from behavior_executive.behaviors.wait import Wait
from behavior_executive.behaviors.stop import Stop
from behavior_executive.behaviors.go_to_last_seen import GoToLastSeen
from behavior_executive.behaviors.rotate_scan import RotateScan
from behavior_executive.behaviors.search_target import SearchTarget
from behavior_executive.behaviors.replan import Replan
from behavior_executive.behaviors.abort_task import AbortTask
from behavior_executive.behaviors.return_home import ReturnHome

BEHAVIOR_REGISTRY = {
    BehaviorType.APPROACH_TARGET: ApproachTarget,
    BehaviorType.FOLLOW_TARGET: FollowTarget,
    BehaviorType.KEEP_DISTANCE: KeepDistance,
    BehaviorType.WAIT: Wait,
    BehaviorType.STOP: Stop,
    BehaviorType.GO_TO_LAST_SEEN: GoToLastSeen,
    BehaviorType.ROTATE_SCAN: RotateScan,
    BehaviorType.SEARCH_TARGET: SearchTarget,
    BehaviorType.REPLAN: Replan,
    BehaviorType.ABORT_TASK: AbortTask,
    BehaviorType.RETURN_HOME: ReturnHome,
}
