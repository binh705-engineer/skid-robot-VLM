#!/usr/bin/env python3
"""
motion_primitives.py

Low-level movement helpers shared by every behavior in behaviors/. This is
the direct descendant of the geometry/Nav2-goal code that used to live
entirely inside coordinate_mapper_node.py, extracted here so every behavior
calls the same tested implementation instead of duplicating it.
"""

import functools
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose, Spin
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from behavior_executive.behavior_types import BehaviorContext


def yaw_to_quaternion(yaw: float):
    """Return (x,y,z,w) for a quaternion that only rotates around the Z axis (yaw)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def find_track(ctx: BehaviorContext, track_id):
    """Return the TrackedObject3D for track_id in the latest cache, or None."""
    if ctx.latest_tracks is None or track_id is None:
        return None
    return next((o for o in ctx.latest_tracks.objects if o.track_id == track_id), None)


def _pose_from_base_ratio(ctx: BehaviorContext, x: float, y: float, ratio: float, face_away: bool = False):
    """
    Shared math for both compute_stop_pose_toward and compute_backoff_pose:
    build a PoseStamped in base_link at (x*ratio, y*ratio), then transform
    it into the map frame.
    """
    goal_x_base = x * ratio
    goal_y_base = y * ratio
    yaw_base = math.atan2(-y, -x) if face_away else math.atan2(y, x)

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

    map_frame = ctx.default_params.get("map_frame", "map")
    tf_timeout_sec = ctx.default_params.get("tf_timeout_sec", 0.3)
    try:
        return ctx.tf_buffer.transform(
            pose_base, map_frame,
            timeout=rclpy.duration.Duration(seconds=tf_timeout_sec))
    except (LookupException, ConnectivityException, ExtrapolationException) as e:
        ctx.node.get_logger().warn(f"TF transform base_link -> {map_frame} failed: {e}")
        return None


def compute_stop_pose_toward(ctx: BehaviorContext, obj, stop_distance_m: float):
    """
    Pose in the map frame: stop_distance_m away from obj (given in
    base_link), on the line robot->obj, facing obj. Used by
    APPROACH_TARGET / FOLLOW_TARGET, and by KEEP_DISTANCE when the target
    is farther than the allowed maximum.
    Returns None on TF failure or a degenerate distance (obj at the robot).
    """
    x, y = obj.position.x, obj.position.y
    distance = math.hypot(x, y)
    if distance < 1e-3:
        return None
    ratio = max(0.0, (distance - stop_distance_m) / distance)
    return _pose_from_base_ratio(ctx, x, y, ratio)


def compute_backoff_pose(ctx: BehaviorContext, obj, target_distance_m: float):
    """
    Pose in the map frame: target_distance_m away from obj, on the line
    robot<->obj but BEHIND the robot's current position (the robot backs
    away). Used by KEEP_DISTANCE when obj is closer than the allowed minimum.
    """
    x, y = obj.position.x, obj.position.y
    distance = math.hypot(x, y)
    if distance < 1e-3:
        return None
    ratio = (distance - target_distance_m) / distance  # negative -> behind the robot's current pose
    return _pose_from_base_ratio(ctx, x, y, ratio, face_away=True)


def capture_current_pose(ctx: BehaviorContext):
    """
    Look up the robot's CURRENT pose in the map frame, no offset applied.
    Used once at the start of a task to record home_pose (for RETURN_HOME).
    Returns None on TF failure.
    """
    map_frame = ctx.default_params.get("map_frame", "map")
    tf_timeout_sec = ctx.default_params.get("tf_timeout_sec", 0.3)
    try:
        tf_stamped = ctx.tf_buffer.lookup_transform(
            map_frame, "base_link", rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=tf_timeout_sec))
    except (LookupException, ConnectivityException, ExtrapolationException) as e:
        ctx.node.get_logger().warn(f"Could not capture current pose: {e}")
        return None

    pose = PoseStamped()
    pose.header.frame_id = map_frame
    pose.pose.position.x = tf_stamped.transform.translation.x
    pose.pose.position.y = tf_stamped.transform.translation.y
    pose.pose.position.z = 0.0
    pose.pose.orientation = tf_stamped.transform.rotation
    return pose


def cancel_current_goal(ctx: BehaviorContext):
    """Cancel whatever Nav2 goal is currently in flight, if any."""
    if ctx.current_goal_handle is not None:
        try:
            ctx.current_goal_handle.cancel_goal_async()
        except Exception as e:
            ctx.node.get_logger().warn(f"Error cancelling Nav2 goal: {e}")
        ctx.current_goal_handle = None


def send_nav2_goal(ctx: BehaviorContext, pose: PoseStamped, on_result):
    """
    Send a NavigateToPose goal. on_result(status_code) is called exactly
    once the goal finishes. status_code follows action_msgs/msg/GoalStatus:
    4=SUCCEEDED, 5=CANCELED, 6=ABORTED. Cancels any goal already in flight
    first, same "replace, don't stack" behavior as the old
    coordinate_mapper_node.send_nav2_goal().
    """
    nav2_server_timeout_sec = ctx.default_params.get("nav2_server_timeout_sec", 10.0)
    if not ctx.nav2_client.wait_for_server(timeout_sec=nav2_server_timeout_sec):
        ctx.node.get_logger().error("Nav2 action server 'navigate_to_pose' not available.")
        on_result(6)  # treat as ABORTED
        return

    cancel_current_goal(ctx)

    goal_msg = NavigateToPose.Goal()
    goal_msg.pose = pose

    send_future = ctx.nav2_client.send_goal_async(goal_msg)
    send_future.add_done_callback(functools.partial(_on_goal_response, ctx, on_result))


def send_spin_goal(ctx: BehaviorContext, spin_angle_rad: float, on_result):
    """
    Send a Nav2 Spin action goal (rotate in place by spin_angle_rad).
    on_result(status_code) called on completion, same status codes as
    send_nav2_goal.
    """
    spin_server_timeout_sec = ctx.default_params.get("nav2_server_timeout_sec", 10.0)
    if not ctx.spin_client.wait_for_server(timeout_sec=spin_server_timeout_sec):
        ctx.node.get_logger().error("Nav2 action server 'spin' not available.")
        on_result(6)
        return

    cancel_current_goal(ctx)

    goal_msg = Spin.Goal()
    goal_msg.target_yaw = spin_angle_rad

    send_future = ctx.spin_client.send_goal_async(goal_msg)
    send_future.add_done_callback(functools.partial(_on_goal_response, ctx, on_result))


def _on_goal_response(ctx: BehaviorContext, on_result, future):
    goal_handle = future.result()
    if not goal_handle.accepted:
        ctx.node.get_logger().error("Nav2/Spin rejected the goal.")
        on_result(6)
        return

    ctx.current_goal_handle = goal_handle
    result_future = goal_handle.get_result_async()
    # Bind THIS goal_handle to its own result callback (functools.partial),
    # same reasoning as the old coordinate_mapper_node: a stale result from
    # a goal that was replaced/cancelled must never be mistaken for the
    # result of a later, still-running goal.
    result_future.add_done_callback(
        functools.partial(_on_goal_result, ctx, goal_handle, on_result))


def _on_goal_result(ctx: BehaviorContext, sent_goal_handle, on_result, future):
    if ctx.current_goal_handle is not sent_goal_handle:
        return  # this result belongs to a goal that was already superseded - ignore it
    result = future.result()
    on_result(result.status)


def publish_cmd_vel_zero(ctx: BehaviorContext):
    """
    Directly publish a zero Twist. Bypasses Nav2 entirely - guarantees the
    robot stops even if the Nav2 action server is stuck or slow to react
    to a cancel request. Used by STOP and ABORT_TASK.
    """
    ctx.cmd_vel_pub.publish(Twist())


def publish_status(ctx: BehaviorContext, status: str):
    msg = String()
    msg.data = status
    ctx.status_pub.publish(msg)
