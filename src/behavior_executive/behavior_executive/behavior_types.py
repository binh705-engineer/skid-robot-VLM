#!/usr/bin/env python3
"""
behavior_types.py

Shared enums and dataclasses used by the whole behavior_executive package:
  - BehaviorType: the fixed vocabulary of navigation behaviors the VLM/Stage B
    layer is allowed to select from.
  - BehaviorStatus: the 3-state result every behavior's tick() must return.
  - BehaviorTask: one node in a task tree - a behavior to run, its params,
    and what to do if it fails (on_failure) or if even the recovery chain is
    exhausted (on_exhausted).
  - BehaviorContext: everything a behavior needs to do its job, gathered in
    one place so behavior classes never need direct access to the ROS node
    or to each other.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, List, Dict, Any

from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
import tf2_ros


class BehaviorType(str, Enum):
    APPROACH_TARGET = "APPROACH_TARGET"
    FOLLOW_TARGET = "FOLLOW_TARGET"
    KEEP_DISTANCE = "KEEP_DISTANCE"
    WAIT = "WAIT"
    STOP = "STOP"
    GO_TO_LAST_SEEN = "GO_TO_LAST_SEEN"
    ROTATE_SCAN = "ROTATE_SCAN"
    SEARCH_TARGET = "SEARCH_TARGET"
    REPLAN = "REPLAN"
    ABORT_TASK = "ABORT_TASK"
    RETURN_HOME = "RETURN_HOME"


class BehaviorStatus(str, Enum):
    RUNNING = "RUNNING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"


@dataclass
class BehaviorTask:
    """
    One node of the task tree the executor runs.

    JSON shape expected from /vlm/target_command (future Stage B output):
      {
        "behavior": "APPROACH_TARGET",
        "params": {"stop_distance_m": 2.0},
        "on_failure": ["GO_TO_LAST_SEEN", "ROTATE_SCAN"],
        "on_exhausted": [{"behavior": "RETURN_HOME"}]
      }
    Entries in on_failure/on_exhausted can be a bare behavior-name string
    (uses default params) or a full nested dict (own params/recovery).
    """
    behavior: BehaviorType
    params: Dict[str, Any] = field(default_factory=dict)
    on_failure: List["BehaviorTask"] = field(default_factory=list)
    on_exhausted: List["BehaviorTask"] = field(default_factory=list)

    @staticmethod
    def from_dict(data) -> "BehaviorTask":
        if isinstance(data, str):
            return BehaviorTask(behavior=BehaviorType(data))
        return BehaviorTask(
            behavior=BehaviorType(data["behavior"]),
            params=data.get("params", {}),
            on_failure=[BehaviorTask.from_dict(t) for t in data.get("on_failure", [])],
            on_exhausted=[BehaviorTask.from_dict(t) for t in data.get("on_exhausted", [])],
        )


@dataclass
class BehaviorContext:
    """
    Shared state + ROS handles passed into every behavior's on_enter/tick/on_exit.
    ONE instance is created by behavior_executor_node and reused across an
    entire task's lifetime (not recreated per behavior), so a behavior can
    read state left behind by a previous behavior in the same task - e.g.
    GO_TO_LAST_SEEN reads last_target_xy_map written by
    ApproachTarget/FollowTarget/KeepDistance while they were running.
    """
    node: Node
    tf_buffer: tf2_ros.Buffer
    nav2_client: ActionClient
    spin_client: ActionClient
    cmd_vel_pub: Any
    status_pub: Any

    # Global param defaults (declared on the node); a per-task params dict
    # in BehaviorTask can override any of these for that one task instance.
    default_params: Dict[str, Any]

    # Live tracking data, refreshed by behavior_executor_node's subscription callback
    latest_tracks: Any = None
    target_id: Optional[int] = None

    # State carried between behaviors within ONE task tree's lifetime
    last_target_xy_map: Optional[tuple] = None    # (x, y) in map frame - last known position of target_id
    home_pose: Optional[PoseStamped] = None        # captured at task start, used by RETURN_HOME

    # Nav2 goal bookkeeping, shared so behaviors never step on each other's in-flight goals
    current_goal_handle: Any = None

    def get_param(self, task: BehaviorTask, key: str, default=None):
        """Per-task override (task.params) wins over the node-wide default."""
        if key in task.params:
            return task.params[key]
        return self.default_params.get(key, default)
