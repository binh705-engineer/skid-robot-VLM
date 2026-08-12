#!/usr/bin/env python3
"""
Behavior Executor Node
-----------------------
Drop-in replacement for the old coordinate_mapper_node.py: SAME topic
interface (/vlm/target_command in, /kinematics/goal_status out), so
vlm_client_node.py needs NO changes today - just point the launch file at
this node instead of coordinate_mapper_node.

Difference from the old node: instead of hardcoding a single "approach and
stop" behavior, this node runs an explicit behavior task TREE:

  main_tasks:        [Task1, Task2, ...]  - run in order, ALL must succeed
                                             for the overall task to succeed
  Task.on_failure:   [..]                 - recovery chain tried if Task fails.
                                             If any step in this chain succeeds,
                                             Task itself is retried from scratch.
  Task.on_exhausted: [..]                 - fallback chain tried only if the
                                             WHOLE on_failure chain also fails.
                                             If this chain succeeds, the overall
                                             task still ends in "failed" (the
                                             original goal wasn't met), but the
                                             robot completed the fallback plan
                                             cleanly (e.g. RETURN_HOME).

Backward compatibility: if /vlm/target_command only contains
{"target_id": N} (today's vlm_client_node format, no "tasks" field), a
default single-task tree is built - see default_task_tree() below. A
future Stage B (language -> behavior planner) can instead publish the full
tree via a "tasks" field; see behavior_types.BehaviorTask.from_dict() for
the expected JSON shape.

Topics:
  Sub  /vlm/target_command     (std_msgs/String)  JSON {"target_id": int, "tasks": [...] (optional)}
  Sub  /tracked_persons_depth  (perception/msg/TrackedObject3DArray)
  Pub  /kinematics/goal_status (std_msgs/String)   "reached" / "failed"
  Pub  <cmd_vel_topic>         (geometry_msgs/Twist)
  Action clients: navigate_to_pose (nav2_msgs/action/NavigateToPose),
                  spin (nav2_msgs/action/Spin)
"""

import json
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose, Spin

import tf2_ros
import tf2_geometry_msgs  # noqa: F401 - required for Buffer.transform() to accept PoseStamped

from perception.msg import TrackedObject3DArray

from behavior_executive.behavior_types import (
    BehaviorContext, BehaviorTask, BehaviorType, BehaviorStatus,
)
from behavior_executive.behaviors import BEHAVIOR_REGISTRY
from behavior_executive.motion_primitives import capture_current_pose, publish_status


def default_task_tree() -> list:
    """
    Used when /vlm/target_command only carries {"target_id": N} (today's
    vlm_client_node format, no "tasks" field). Matches the OLD
    coordinate_mapper_node's behavior (approach + stop), plus the recovery
    chain discussed for the "camera loses the person while Nav2 avoids an
    obstacle" bug, plus a RETURN_HOME fallback if recovery is exhausted.
    """
    return [
        BehaviorTask(
            behavior=BehaviorType.APPROACH_TARGET,
            on_failure=[
                BehaviorTask(behavior=BehaviorType.GO_TO_LAST_SEEN),
                BehaviorTask(behavior=BehaviorType.ROTATE_SCAN),
            ],
            on_exhausted=[
                BehaviorTask(behavior=BehaviorType.RETURN_HOME),
            ],
        )
    ]


class BehaviorExecutorNode(Node):
    def __init__(self):
        super().__init__('behavior_executor_node')

        # ==== Params ====
        self.declare_parameter("target_command_topic", "/vlm/target_command")
        self.declare_parameter("tracked_persons_topic", "/tracked_persons_depth")
        self.declare_parameter("goal_status_topic", "/kinematics/goal_status")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("nav2_action_name", "navigate_to_pose")
        self.declare_parameter("spin_action_name", "spin")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("stop_distance_m", 2.5)
        self.declare_parameter("min_distance_m", 1.0)
        self.declare_parameter("max_distance_m", 3.0)
        self.declare_parameter("lost_grace_period_sec", 2.0)
        self.declare_parameter("goal_update_min_distance_m", 0.3)
        self.declare_parameter("check_period_sec", 0.5)
        self.declare_parameter("tf_timeout_sec", 0.3)
        self.declare_parameter("nav2_server_timeout_sec", 10.0)
        self.declare_parameter("scan_angle_deg", 360.0)

        default_params = {
            "map_frame": self.get_parameter("map_frame").value,
            "stop_distance_m": self.get_parameter("stop_distance_m").value,
            "min_distance_m": self.get_parameter("min_distance_m").value,
            "max_distance_m": self.get_parameter("max_distance_m").value,
            "lost_grace_period_sec": self.get_parameter("lost_grace_period_sec").value,
            "goal_update_min_distance_m": self.get_parameter("goal_update_min_distance_m").value,
            "tf_timeout_sec": self.get_parameter("tf_timeout_sec").value,
            "nav2_server_timeout_sec": self.get_parameter("nav2_server_timeout_sec").value,
            "scan_angle_deg": self.get_parameter("scan_angle_deg").value,
        }

        self.check_period_sec_ = self.get_parameter("check_period_sec").value

        # ==== TF ====
        tf_buffer = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(tf_buffer, self)

        cb_group = ReentrantCallbackGroup()

        # ==== Action clients ====
        nav2_client = ActionClient(
            self, NavigateToPose, self.get_parameter("nav2_action_name").value,
            callback_group=cb_group)
        spin_client = ActionClient(
            self, Spin, self.get_parameter("spin_action_name").value,
            callback_group=cb_group)

        # ==== Publishers ====
        status_pub = self.create_publisher(
            String, self.get_parameter("goal_status_topic").value, 10)
        cmd_vel_pub = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10)

        # ==== Shared context passed into every behavior ====
        self.ctx_ = BehaviorContext(
            node=self,
            tf_buffer=tf_buffer,
            nav2_client=nav2_client,
            spin_client=spin_client,
            cmd_vel_pub=cmd_vel_pub,
            status_pub=status_pub,
            default_params=default_params,
        )

        # ==== Task-tree runtime state ====
        self.state_lock_ = threading.Lock()
        self.running_ = False               # False = IDLE, no task tree active
        self.main_tasks_ = []
        self.main_index_ = 0
        self.recovery_chain_ = None          # currently active on_failure/on_exhausted chain
        self.recovery_index_ = 0
        self.recovery_kind_ = None           # "on_failure" | "on_exhausted"
        self.origin_main_task_ = None        # the main task whose failure triggered the current recovery chain
        self.current_task_ = None
        self.current_behavior_ = None        # instance implementing BaseBehavior

        # ==== Subscribers ====
        self.target_sub_ = self.create_subscription(
            String, self.get_parameter("target_command_topic").value,
            self.target_command_callback, 10, callback_group=cb_group)

        self.tracks_sub_ = self.create_subscription(
            TrackedObject3DArray, self.get_parameter("tracked_persons_topic").value,
            self.tracks_callback, 10, callback_group=cb_group)

        # ==== Timer driving the FSM ====
        self.timer_ = self.create_timer(
            self.check_period_sec_, self.tick_executor, callback_group=cb_group)

        self.get_logger().info(
            f"BehaviorExecutorNode ready. State=IDLE. Waiting for target on "
            f"'{self.get_parameter('target_command_topic').value}'.")

    # ---------------- Subscriptions ----------------

    def tracks_callback(self, msg: TrackedObject3DArray):
        self.ctx_.latest_tracks = msg

    def target_command_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            target_id = int(data["target_id"])
        except (json.JSONDecodeError, KeyError, ValueError, TypeError) as e:
            self.get_logger().error(f"Invalid target_command: '{msg.data}' ({e}). Ignoring.")
            return

        if data.get("tasks"):
            main_tasks = [BehaviorTask.from_dict(t) for t in data["tasks"]]
        else:
            main_tasks = default_task_tree()

        with self.state_lock_:
            if self.running_:
                self.get_logger().warn(
                    f"Received a new target_id={target_id} while a task tree is "
                    f"still running -- preempting the current task.")
                self._exit_current_behavior()

            self.ctx_.target_id = target_id
            self.ctx_.last_target_xy_map = None
            self.ctx_.home_pose = capture_current_pose(self.ctx_)
            if self.ctx_.home_pose is None:
                self.get_logger().warn(
                    "Could not capture home pose at task start -- RETURN_HOME "
                    "will not be available for this task if it's ever reached.")

            self.main_tasks_ = main_tasks
            self.main_index_ = 0
            self.recovery_chain_ = None
            self.recovery_index_ = 0
            self.recovery_kind_ = None
            self.origin_main_task_ = None
            self.running_ = True

            self._start_task(self.main_tasks_[0])

        self.get_logger().info(f"Starting task tree for target_id={target_id}.")

    # ---------------- FSM driver ----------------

    def tick_executor(self):
        with self.state_lock_:
            if not self.running_ or self.current_behavior_ is None:
                return

            status = self.current_behavior_.tick(self.ctx_, self.current_task_)
            if status == BehaviorStatus.RUNNING:
                return

            self._exit_current_behavior()

            if status == BehaviorStatus.SUCCEEDED:
                self._advance_after_success()
            else:
                self._advance_after_failure()

    def _start_task(self, task: BehaviorTask):
        """Call only while holding state_lock_."""
        behavior_cls = BEHAVIOR_REGISTRY[task.behavior]
        self.current_task_ = task
        self.current_behavior_ = behavior_cls()
        self.get_logger().info(f"-> starting behavior {task.behavior.value}")
        self.current_behavior_.on_enter(self.ctx_, task)

    def _exit_current_behavior(self):
        """Call only while holding state_lock_."""
        if self.current_behavior_ is not None:
            self.current_behavior_.on_exit(self.ctx_, self.current_task_)

    def _advance_after_success(self):
        """Call only while holding state_lock_, right after the current
        behavior returned SUCCEEDED."""
        if self.recovery_chain_ is not None:
            if self.recovery_kind_ == "on_failure":
                # Recovery worked (e.g. target reacquired) -> retry the
                # ORIGINAL main task from scratch.
                self.get_logger().info(
                    f"Recovery succeeded -- retrying {self.origin_main_task_.behavior.value}.")
                self.recovery_chain_ = None
                self.recovery_index_ = 0
                self.recovery_kind_ = None
                self._start_task(self.origin_main_task_)
            else:
                # on_exhausted chain succeeded (e.g. RETURN_HOME reached).
                # The original goal (e.g. finding the person) was NOT met,
                # but the fallback plan completed cleanly.
                self.get_logger().info(
                    "on_exhausted fallback completed. Task tree ends (original goal not met).")
                self._finish(overall_success=False)
            return

        # A main task succeeded normally -> advance to the next one, if any.
        self.main_index_ += 1
        if self.main_index_ >= len(self.main_tasks_):
            self._finish(overall_success=True)
        else:
            self._start_task(self.main_tasks_[self.main_index_])

    def _advance_after_failure(self):
        """Call only while holding state_lock_, right after the current
        behavior returned FAILED."""
        if self.recovery_chain_ is not None:
            self.recovery_index_ += 1
            if self.recovery_index_ < len(self.recovery_chain_):
                self._start_task(self.recovery_chain_[self.recovery_index_])
                return

            # Current chain (on_failure or on_exhausted) is exhausted.
            if self.recovery_kind_ == "on_failure":
                if self.origin_main_task_.on_exhausted:
                    self.get_logger().warn(
                        "on_failure recovery chain exhausted -- falling through to on_exhausted.")
                    self.recovery_chain_ = self.origin_main_task_.on_exhausted
                    self.recovery_index_ = 0
                    self.recovery_kind_ = "on_exhausted"
                    self._start_task(self.recovery_chain_[0])
                else:
                    self._finish(overall_success=False)
            else:
                # on_exhausted chain ALSO failed -> nothing left to try.
                self.get_logger().error("on_exhausted fallback chain also failed. Giving up.")
                self._finish(overall_success=False)
            return

        # A main task failed -> enter its on_failure chain (or on_exhausted
        # directly if on_failure wasn't defined, or give up if neither was).
        task = self.main_tasks_[self.main_index_]
        self.origin_main_task_ = task
        if task.on_failure:
            self.recovery_chain_ = task.on_failure
            self.recovery_index_ = 0
            self.recovery_kind_ = "on_failure"
            self._start_task(self.recovery_chain_[0])
        elif task.on_exhausted:
            self.recovery_chain_ = task.on_exhausted
            self.recovery_index_ = 0
            self.recovery_kind_ = "on_exhausted"
            self._start_task(self.recovery_chain_[0])
        else:
            self._finish(overall_success=False)

    def _finish(self, overall_success: bool):
        """Call only while holding state_lock_."""
        self.running_ = False
        self.current_task_ = None
        self.current_behavior_ = None
        self.ctx_.home_pose = None
        self.ctx_.last_target_xy_map = None
        publish_status(self.ctx_, "reached" if overall_success else "failed")
        self.get_logger().info(
            f"Task tree finished. overall_success={overall_success}. Back to IDLE.")


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorExecutorNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
