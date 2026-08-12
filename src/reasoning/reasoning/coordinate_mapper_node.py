#!/usr/bin/env python3
"""
Coordinate Mapper Node
----------------------
The missing link between the VLM (which only chooses "who") and the robot
(Nav2). This node:

1. Receives the locked target_id from /vlm/target_command
   (String JSON {"target_id": int}).
2. Looks up the REAL position of that track_id from /tracked_persons_depth
   (frame_id=base_link), CONTINUOUSLY (not just once) - because the person
   may keep moving while the robot is on its way, and because the robot
   itself is also moving, so the relative position (base_link) changes
   every frame.
3. Transforms the coordinate into the "map" frame (via TF, the robot
   already has map->odom->base_link), and computes a stop point at a safe
   distance (stop_distance_m, default 1.5m) from the person, along the
   direction from the robot to the person, facing the robot toward the
   person.
4. Sends that goal to Nav2 (NavigateToPose action). If the person has moved
   significantly, cancel the old goal and send a new one (don't spam a new
   goal every frame).
5. If the track_id DISAPPEARS from /tracked_persons_depth for longer than
   lost_grace_period_sec (default 2.0s - ByteTrack already has its own
   ~1s track_buffer for short occlusions; this is an extra safety layer on
   top of that), cancel the Nav2 goal and report "failed" -> vlm_client_node
   automatically returns to IDLE, and the user must issue a new command
   from scratch.
6. When Nav2 reports SUCCEEDED -> report "reached". When Nav2 reports
   failure/cancellation -> report "failed".

This node REPLACES "kinematics" from the original design - it combines
coordinate lookup + calling Nav2 + reporting status into a single node.

Topics:
  Sub  /vlm/target_command     (std_msgs/String)                      JSON {"target_id": <int>}
  Sub  /tracked_persons_depth  (perception/msg/TrackedObject3DArray)  CONTINUOUS position, frame_id=base_link
  Pub  /kinematics/goal_status (std_msgs/String)                      "reached" / "failed"
  Action client: navigate_to_pose (nav2_msgs/action/NavigateToPose)
"""

import functools
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs  # noqa: F401 - needed for Buffer.transform() to support PoseStamped

from perception.msg import TrackedObject3DArray

import json
from enum import Enum


def yaw_to_quaternion(yaw: float):
    """Return (x,y,z,w) for a quaternion that only rotates around the Z axis (yaw)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class MapperState(Enum):
    IDLE = 0
    TRACKING = 1   # received a target_id, sending/tracking a Nav2 goal


class CoordinateMapperNode(Node):
    def __init__(self):
        super().__init__('coordinate_mapper_node')

        # ==== Params ====
        self.declare_parameter("target_command_topic", "/vlm/target_command")
        self.declare_parameter("tracked_persons_topic", "/tracked_persons_depth")
        self.declare_parameter("goal_status_topic", "/kinematics/goal_status")
        self.declare_parameter("nav2_action_name", "navigate_to_pose")
        self.declare_parameter("map_frame", "map")

        # Stop this many meters away from the person before considering "arrived"
        self.declare_parameter("stop_distance_m", 2.5)

        # How long (seconds) the track_id can be missing while TRACKING before
        # it's considered truly lost
        self.declare_parameter("lost_grace_period_sec", 2.0)

        # How many meters the person must move before it counts as "moved
        # enough" to send a new goal (avoids spamming Nav2 with a new goal
        # every frame due to small tracker jitter)
        self.declare_parameter("goal_update_min_distance_m", 0.3)

        # Period (seconds) for re-checking position / lost status
        self.declare_parameter("check_period_sec", 0.5)

        # Timeout (seconds) for each TF lookup step
        self.declare_parameter("tf_timeout_sec", 0.3)

        # Timeout (seconds) for Nav2 wait_for_server
        self.declare_parameter("nav2_server_timeout_sec", 10.0)

        self.target_command_topic_ = self.get_parameter("target_command_topic").value
        self.tracked_persons_topic_ = self.get_parameter("tracked_persons_topic").value
        self.goal_status_topic_ = self.get_parameter("goal_status_topic").value
        self.nav2_action_name_ = self.get_parameter("nav2_action_name").value
        self.map_frame_ = self.get_parameter("map_frame").value
        self.stop_distance_m_ = self.get_parameter("stop_distance_m").value
        self.lost_grace_period_sec_ = self.get_parameter("lost_grace_period_sec").value
        self.goal_update_min_distance_m_ = self.get_parameter("goal_update_min_distance_m").value
        self.check_period_sec_ = self.get_parameter("check_period_sec").value
        self.tf_timeout_sec_ = self.get_parameter("tf_timeout_sec").value
        self.nav2_server_timeout_sec_ = self.get_parameter("nav2_server_timeout_sec").value

        # ==== TF ====
        self.tf_buffer_ = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_, self)

        # ==== State ====
        self.state_ = MapperState.IDLE
        self.state_lock_ = threading.Lock()
        self.current_target_id_ = None
        self.last_seen_time_ = None          # rclpy Time - last time the track_id was in-frame
        self.last_sent_goal_xy_map_ = None   # (x, y) in the map frame of the most recently sent goal
        self.current_goal_handle_ = None

        self.latest_tracks_ = None

        cb_group = ReentrantCallbackGroup()

        # ==== Subscribers ====
        self.target_sub_ = self.create_subscription(
            String, self.target_command_topic_,
            self.target_command_callback, 10,
            callback_group=cb_group)

        self.tracks_sub_ = self.create_subscription(
            TrackedObject3DArray, self.tracked_persons_topic_,
            self.tracks_callback, 10,
            callback_group=cb_group)

        # ==== Publisher ====
        self.status_pub_ = self.create_publisher(String, self.goal_status_topic_, 10)

        # ==== Nav2 action client ====
        self.nav2_client_ = ActionClient(self, NavigateToPose, self.nav2_action_name_)

        # ==== Timer: check for lost target + update goal ====
        self.check_timer_ = self.create_timer(
            self.check_period_sec_, self.check_tracking, callback_group=cb_group)

        self.get_logger().info(
            f"CoordinateMapperNode ready. State=IDLE. "
            f"Waiting for target on '{self.target_command_topic_}'. "
            f"stop_distance_m={self.stop_distance_m_}, "
            f"lost_grace_period_sec={self.lost_grace_period_sec_}."
        )

    # ---------------- Cache callback ----------------

    def tracks_callback(self, msg: TrackedObject3DArray):
        self.latest_tracks_ = msg

        # Update last_seen_time_ if the current target (if any) is still in this frame
        with self.state_lock_:
            if self.state_ == MapperState.TRACKING and self.current_target_id_ is not None:
                if any(obj.track_id == self.current_target_id_ for obj in msg.objects):
                    self.last_seen_time_ = self.get_clock().now()

    # ---------------- Receive a new target_id from vlm_client_node ----------------

    def target_command_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            target_id = int(data["target_id"])
        except (json.JSONDecodeError, KeyError, ValueError, TypeError) as e:
            self.get_logger().error(f"Invalid target_command: '{msg.data}' ({e}). Ignoring.")
            return

        with self.state_lock_:
            if self.state_ == MapperState.TRACKING:
                self.get_logger().warn(
                    f"Received a new target_id={target_id} while still TRACKING "
                    f"target_id={self.current_target_id_} -- cancelling the old goal, switching to the new target.")
                self._cancel_current_goal()

            self.current_target_id_ = target_id
            self.last_seen_time_ = self.get_clock().now()
            self.last_sent_goal_xy_map_ = None
            self.state_ = MapperState.TRACKING

        self.get_logger().info(f"Starting to track + move toward target_id={target_id}.")
        # Try sending the goal right away (don't wait for the check_tracking timer)
        self.try_send_goal_for_current_target()

    # ---------------- Timer: check for lost target + whether the goal needs updating ----------------

    def check_tracking(self):
        with self.state_lock_:
            if self.state_ != MapperState.TRACKING:
                return
            target_id = self.current_target_id_
            last_seen = self.last_seen_time_

        if target_id is None or last_seen is None:
            return

        elapsed = (self.get_clock().now() - last_seen).nanoseconds / 1e9
        if elapsed > self.lost_grace_period_sec_:
            self.get_logger().error(
                f"target_id={target_id} lost for {elapsed:.1f}s "
                f"(> {self.lost_grace_period_sec_}s). Cancelling goal, reporting failed.")
            with self.state_lock_:
                self._cancel_current_goal()
                self.state_ = MapperState.IDLE
                self.current_target_id_ = None
                self.last_sent_goal_xy_map_ = None
            self.publish_status("failed")
            return

        # Still visible -> check whether the position has changed enough to resend the goal
        self.try_send_goal_for_current_target()

    # ---------------- Compute + send the Nav2 goal ----------------

    def try_send_goal_for_current_target(self):
        with self.state_lock_:
            target_id = self.current_target_id_
            if target_id is None or self.state_ != MapperState.TRACKING:
                return
        if self.latest_tracks_ is None:
            return

        obj = next(
            (o for o in self.latest_tracks_.objects if o.track_id == target_id), None)
        if obj is None:
            return  # not seen in the latest frame yet, let check_tracking handle it if it times out

        goal_pose_map = self.compute_goal_pose(obj)
        if goal_pose_map is None:
            return  # TF error, will retry on the next check

        gx, gy = goal_pose_map.pose.position.x, goal_pose_map.pose.position.y

        if self.last_sent_goal_xy_map_ is not None:
            lx, ly = self.last_sent_goal_xy_map_
            moved = math.hypot(gx - lx, gy - ly)
            if moved < self.goal_update_min_distance_m_:
                return  # hasn't moved enough yet, don't spam a new goal

        self.send_nav2_goal(goal_pose_map)
        self.last_sent_goal_xy_map_ = (gx, gy)

    def compute_goal_pose(self, obj) -> "PoseStamped | None":
        """
        Compute the goal pose in the map frame: a point at a distance of
        stop_distance_m_ from obj.position, along the straight line from
        the robot (base_link origin) to obj, facing the robot toward obj.
        Computed first in base_link (robot = coordinate origin), then
        transformed into map - so it's correct no matter where the robot
        actually is in the map.
        """
        x, y = obj.position.x, obj.position.y
        distance = math.hypot(x, y)

        if distance < 1e-3:
            # obj is right at the robot's position (not physically plausible) - skip, don't compute a goal
            return None

        ratio = max(0.0, (distance - self.stop_distance_m_) / distance)
        goal_x_base = x * ratio
        goal_y_base = y * ratio
        yaw_base = math.atan2(y, x)  # face toward obj

        pose_base = PoseStamped()
        pose_base.header.frame_id = "base_link"
        pose_base.header.stamp = rclpy.time.Time().to_msg()  # use the "latest" available transform
        pose_base.pose.position.x = goal_x_base
        pose_base.pose.position.y = goal_y_base
        pose_base.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(yaw_base)
        pose_base.pose.orientation.x = qx
        pose_base.pose.orientation.y = qy
        pose_base.pose.orientation.z = qz
        pose_base.pose.orientation.w = qw

        try:
            pose_map = self.tf_buffer_.transform(
                pose_base, self.map_frame_,
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout_sec_))
            return pose_map
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF transform base_link -> {self.map_frame_} failed: {e}")
            return None

    def send_nav2_goal(self, pose_map: PoseStamped):
        if not self.nav2_client_.wait_for_server(timeout_sec=self.nav2_server_timeout_sec_):
            self.get_logger().error("Nav2 action server 'navigate_to_pose' is not available.")
            with self.state_lock_:
                self.state_ = MapperState.IDLE
                self.current_target_id_ = None
            self.publish_status("failed")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_map

        self.get_logger().info(
            f"Sending Nav2 goal: map=({pose_map.pose.position.x:.2f}, "
            f"{pose_map.pose.position.y:.2f})")

        # Cancel the currently running goal (if any) before sending a new one,
        # to avoid Nav2 receiving 2 overlapping goals
        self._cancel_current_goal()

        send_future = self.nav2_client_.send_goal_async(goal_msg)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 rejected the goal.")
            with self.state_lock_:
                self.state_ = MapperState.IDLE
                self.current_target_id_ = None
            self.publish_status("failed")
            return

        with self.state_lock_:
            self.current_goal_handle_ = goal_handle

        result_future = goal_handle.get_result_async()
        # IMPORTANT: bind THIS goal_handle to its own result callback (via
        # functools.partial), instead of using self._on_goal_result directly.
        # Reason: when a goal is cancelled because a new one replaces it (the
        # person moved), get_result_async() for the OLD goal still returns a
        # result (usually ABORTED/CANCELED). If we don't know which goal that
        # result belongs to, we might mistakenly treat it as the result of
        # the entire current tracking session and wrongly report "failed"
        # even though the NEW goal is still running fine.
        result_future.add_done_callback(
            functools.partial(self._on_goal_result, goal_handle))

    def _on_goal_result(self, sent_goal_handle, future):
        with self.state_lock_:
            # Only process the result if this is truly the currently ACTIVE
            # goal (not just checking "are we still TRACKING" - since state_
            # stays TRACKING even after the goal has been replaced by a newer
            # one for the same target_id).
            is_current = (
                self.state_ == MapperState.TRACKING
                and self.current_goal_handle_ is sent_goal_handle
            )
        if not is_current:
            # This result belongs to a goal that has already been
            # replaced/cancelled - ignore it, don't report status, to avoid
            # falsely reporting "failed" while a newer goal is still running
            # or has already succeeded.
            return

        result = future.result()
        status = result.status
        # GoalStatus (action_msgs/msg/GoalStatus):
        # 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
        if status == 4:
            self.get_logger().info("Nav2 reported reaching the goal (SUCCEEDED).")
            with self.state_lock_:
                self.state_ = MapperState.IDLE
                self.current_target_id_ = None
                self.last_sent_goal_xy_map_ = None
            self.publish_status("reached")
        else:
            self.get_logger().warn(f"Nav2 goal ended unsuccessfully (status={status}).")
            with self.state_lock_:
                self.state_ = MapperState.IDLE
                self.current_target_id_ = None
                self.last_sent_goal_xy_map_ = None
            self.publish_status("failed")

    def _cancel_current_goal(self):
        """Call while already holding state_lock_."""
        if self.current_goal_handle_ is not None:
            try:
                self.current_goal_handle_.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f"Error while cancelling the old Nav2 goal: {e}")
            self.current_goal_handle_ = None

    def publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CoordinateMapperNode()

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
