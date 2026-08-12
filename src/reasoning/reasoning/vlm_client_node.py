#!/usr/bin/env python3
"""
VLM Client Node
---------------
Receives one image with bbox+ID (~28-29Hz, /image_visualized), plus the
list of currently alive track_ids (/tracked_persons_depth). ONLY calls the
VLM when a new command trigger arrives.

NEW DESIGN: the VLM ONLY chooses "who" (target_id) - it no longer
receives/sends a BEV image, and no longer chooses a coordinate
(grid_n/grid_m). Resolving the real coordinate + continuously tracking it
while the robot moves is now the responsibility of coordinate_mapper_node.py
(which subscribes to /vlm/target_command + /tracked_persons_depth directly).

Shares the same schema/prompt/validator with reasoning/vlm_skill_vocabulary.py:
  - build_system_prompt(): generates the system prompt enforcing the 3-case
    JSON format.
  - safe_get_decision(): parse + validate + safe fallback (never raises).

State machine:
  IDLE      -> not calling the VLM, just caching the latest data (image + tracks)
  INFERRING -> received a trigger, waiting for fresh-enough data, calling the
               VLM (may retry if UNCERTAIN)
  EXECUTING -> already published a target_id to coordinate_mapper, waiting
               for goal_status or timeout

Handling the 3 cases from safe_get_decision():
  LOCKED     -> publish target_command {"target_id": <int>}, move to EXECUTING.
                (This also covers the fallback case "keep the old ID on
                parse error" - since LOCKED is now always a complete
                decision, there's no more coordinate field that needs to be
                checked for None.)
  UNCERTAIN  -> clear the old image/tracks cache, wait for TRULY NEW data,
                call the VLM again (up to max_uncertain_retries times).
  NOT_FOUND  -> return to IDLE immediately, treat the old ID (if any) as no
                longer valid.

previous_locked_id_ (the most recently locked ID, used as a fallback in
safe_get_decision()):
  - Kept throughout a single chain of UNCERTAIN retries for the SAME trigger.
  - Reset to None when: leaving EXECUTING (success/failure/timeout), when the
    VLM confirms NOT_FOUND, or when the UNCERTAIN retry budget runs out
    (giving up).

Topics:
  Sub  /image_visualized       (sensor_msgs/Image)                    bbox+ID image
  Sub  /tracked_persons_depth  (perception/msg/TrackedObject3DArray)  source of valid_ids
  Sub  /vlm/trigger            (std_msgs/String)                      command (= instruction)
  Sub  /kinematics/goal_status (std_msgs/String)                      "reached"/"failed"
  Pub  /vlm/target_command     (std_msgs/String)                      JSON {"target_id": <int>}
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from perception.msg import TrackedObject3DArray

from reasoning.vlm_skill_vocabulary import (
    build_system_prompt,
    safe_get_decision,
    VLMStatus,
)

import cv2
import base64
import json
import time
import threading
import requests
from enum import Enum
from time import perf_counter


class VlmState(Enum):
    IDLE = 0
    INFERRING = 1
    EXECUTING = 2


class VLMClientNode(Node):
    def __init__(self):
        super().__init__('vlm_client_node')

        # ==== Params ====
        self.declare_parameter("api_url", "http://localhost:8080/v1/chat/completions")
        self.declare_parameter("bbox_image_topic", "/image_visualized")
        self.declare_parameter("tracked_persons_topic", "/tracked_persons_depth")
        self.declare_parameter("trigger_topic", "/vlm/trigger")
        self.declare_parameter("goal_status_topic", "/kinematics/goal_status")
        self.declare_parameter("target_command_topic", "/vlm/target_command")
        self.declare_parameter("max_image_age_sec", 1.0)
        self.declare_parameter("image_wait_timeout_sec", 3.0)
        self.declare_parameter("vlm_request_timeout_sec", 60.0)
        self.declare_parameter("executing_timeout_sec", 60.0)
        self.declare_parameter("max_uncertain_retries", 2)

        self.api_url_ = self.get_parameter("api_url").value
        self.max_image_age_sec_ = self.get_parameter("max_image_age_sec").value
        self.image_wait_timeout_sec_ = self.get_parameter("image_wait_timeout_sec").value
        self.vlm_request_timeout_sec_ = self.get_parameter("vlm_request_timeout_sec").value
        self.executing_timeout_sec_ = self.get_parameter("executing_timeout_sec").value
        self.max_uncertain_retries_ = self.get_parameter("max_uncertain_retries").value

        self.bridge_ = CvBridge()

        # ==== State ====
        self.state_ = VlmState.IDLE
        self.state_lock_ = threading.Lock()
        self.executing_deadline_ = None
        self.uncertain_retry_count_ = 0
        self.previous_locked_id_ = None   # most recent still-"alive" locked ID (fallback)

        # Cache of the latest data - callbacks ONLY overwrite, no further processing
        self.latest_bbox_img_ = None
        self.latest_bbox_stamp_ = None

        self.latest_tracks_ = None
        self.latest_tracks_stamp_ = None

        cb_group = ReentrantCallbackGroup()

        # ==== Subscribers ====
        self.bbox_sub_ = self.create_subscription(
            Image, self.get_parameter("bbox_image_topic").value,
            self.bbox_image_callback, qos_profile_sensor_data,
            callback_group=cb_group)

        self.tracks_sub_ = self.create_subscription(
            TrackedObject3DArray, self.get_parameter("tracked_persons_topic").value,
            self.tracks_callback, 10,
            callback_group=cb_group)

        self.trigger_sub_ = self.create_subscription(
            String, self.get_parameter("trigger_topic").value,
            self.trigger_callback, 10,
            callback_group=cb_group)

        self.goal_status_sub_ = self.create_subscription(
            String, self.get_parameter("goal_status_topic").value,
            self.goal_status_callback, 10,
            callback_group=cb_group)

        # ==== Publisher ====
        self.target_pub_ = self.create_publisher(
            String, self.get_parameter("target_command_topic").value, 10)

        # Timer checking for EXECUTING timeout
        self.timeout_timer_ = self.create_timer(
            1.0, self.check_executing_timeout, callback_group=cb_group)

        self.get_logger().info(
            f"VLMClientNode ready. State=IDLE. "
            f"Waiting for trigger on '{self.get_parameter('trigger_topic').value}'. "
            f"(Manual test: ros2 topic pub --once {self.get_parameter('trigger_topic').value} "
            f"std_msgs/msg/String \"data: 'your command'\")"
        )

    # ---------------- Cache callbacks (DO NOT call the VLM here) ----------------

    def bbox_image_callback(self, msg: Image):
        self.latest_bbox_img_ = msg
        self.latest_bbox_stamp_ = self.get_clock().now()

    def tracks_callback(self, msg: TrackedObject3DArray):
        self.latest_tracks_ = msg
        self.latest_tracks_stamp_ = self.get_clock().now()

    # ---------------- Trigger: start a new inference cycle ----------------

    def trigger_callback(self, msg: String):
        with self.state_lock_:
            if self.state_ != VlmState.IDLE:
                self.get_logger().warn(
                    f"Ignoring new trigger since state is {self.state_.name} "
                    f"(previous command not finished yet, avoiding overlapping VLM calls).")
                return
            self.state_ = VlmState.INFERRING
            self.uncertain_retry_count_ = 0

        self.get_logger().info(f"Received trigger: '{msg.data}'. Starting VLM inference.")
        threading.Thread(target=self.run_inference, args=(msg.data,), daemon=True).start()

    # ---------------- Goal status from coordinate_mapper/kinematics node ----------------

    def goal_status_callback(self, msg: String):
        with self.state_lock_:
            if self.state_ != VlmState.EXECUTING:
                return  # reports completion but we're not waiting -> ignore, avoid mixing up with an old command

            status = msg.data.strip().lower()
            if status in ("reached", "success", "done"):
                self.get_logger().info("Reached the target. Returning to IDLE.")
            else:
                self.get_logger().warn(f"Reported failure ('{msg.data}'). Returning to IDLE.")

            self.state_ = VlmState.IDLE
            self.executing_deadline_ = None
            # The task has ended (success or not) -> the next command is a
            # completely new task, should not "stick" to the old ID.
            self.previous_locked_id_ = None

    def check_executing_timeout(self):
        with self.state_lock_:
            if self.state_ == VlmState.EXECUTING and self.executing_deadline_ is not None:
                if time.time() > self.executing_deadline_:
                    self.get_logger().error(
                        "EXECUTING timeout: no goal_status received. Forcing back to IDLE.")
                    self.state_ = VlmState.IDLE
                    self.executing_deadline_ = None
                    self.previous_locked_id_ = None

    # ---------------- Run the VLM (in a separate thread) ----------------

    def run_inference(self, instruction: str):
        t0 = perf_counter()

        bbox_img, valid_ids = self.wait_for_fresh_inputs()

        if bbox_img is None or valid_ids is None:
            self.get_logger().error(
                "Could not get sufficiently fresh data (bbox/tracks) within the allowed time. "
                "Cancelling the command, returning to IDLE.")
            with self.state_lock_:
                self.state_ = VlmState.IDLE
                self.uncertain_retry_count_ = 0
            return

        try:
            bbox_b64 = self.imgmsg_to_base64(bbox_img)
            t1 = perf_counter()

            system_prompt = build_system_prompt(
                instruction=instruction,
                valid_ids=valid_ids,
            )
            payload = self.build_payload(system_prompt, bbox_b64)
            t2 = perf_counter()

            self.get_logger().info("Calling the VLM server...")
            response = requests.post(
                self.api_url_, json=payload, timeout=self.vlm_request_timeout_sec_)
            t3 = perf_counter()

            self.get_logger().info(
                f"\n========== PROFILER ==========\n"
                f"encode      : {(t1 - t0):.3f}s\n"
                f"payload     : {(t2 - t1):.3f}s\n"
                f"HTTP+VLM    : {(t3 - t2):.3f}s\n"
                f"TOTAL       : {(t3 - t0):.3f}s\n"
                f"==============================="
            )

            if response.status_code != 200:
                self.get_logger().error(f"VLM server error: {response.status_code} - {response.text}")
                with self.state_lock_:
                    self.state_ = VlmState.IDLE
                    self.uncertain_retry_count_ = 0
                return

            result = response.json()
            content = result["choices"][0]["message"]["content"].strip()

            self.get_logger().info(
                f"\n========== RAW VLM ==========\n{content}\n============================="
            )

            decision = safe_get_decision(
                content,
                valid_ids=valid_ids,
                previous_locked_id=self.previous_locked_id_,
            )

            self.get_logger().info(
                f"PARSED -> status={decision.status.value}, id={decision.target_id}, "
                f"candidates={decision.candidate_ids}, reason={decision.reason}"
            )

            self.handle_decision(decision, instruction)

        except Exception as e:
            self.get_logger().error(f"Exception while calling the VLM: {str(e)}")
            with self.state_lock_:
                self.state_ = VlmState.IDLE
                self.uncertain_retry_count_ = 0

    def handle_decision(self, decision, instruction: str):
        """Handle a single VLMDecision returned by safe_get_decision()."""

        # --- LOCKED -> publish target_id, move to EXECUTING ---
        # (also covers the "keep old ID on parse error" fallback case - now a
        # COMPLETE decision, no coordinate field left that needs a None check.)
        if decision.status == VLMStatus.LOCKED:
            out_msg = String()
            out_msg.data = json.dumps({"target_id": decision.target_id})
            self.target_pub_.publish(out_msg)
            self.get_logger().info(
                f"VLM chose target_id={decision.target_id}. Published, moving to EXECUTING.")
            with self.state_lock_:
                self.state_ = VlmState.EXECUTING
                self.executing_deadline_ = time.time() + self.executing_timeout_sec_
                self.uncertain_retry_count_ = 0
                self.previous_locked_id_ = decision.target_id
            return

        # --- UNCERTAIN -> clear cache, wait for TRULY NEW image/tracks, bounded retry ---
        if decision.status == VLMStatus.UNCERTAIN:
            self.uncertain_retry_count_ += 1
            if self.uncertain_retry_count_ <= self.max_uncertain_retries_:
                self.get_logger().warn(
                    f"VLM UNCERTAIN (attempt {self.uncertain_retry_count_}/"
                    f"{self.max_uncertain_retries_}), candidates={decision.candidate_ids}. "
                    f"Waiting for fresh data before retrying.")
                # IMPORTANT: clear the old cache so wait_for_fresh_inputs() is
                # FORCED to wait for a truly new frame, instead of reusing the
                # same data that just caused the ambiguity.
                self.latest_bbox_img_ = None
                self.latest_bbox_stamp_ = None
                self.latest_tracks_ = None
                self.latest_tracks_stamp_ = None
                # Still in INFERRING, not returning to IDLE. Retry in a new
                # thread to avoid deep recursion if there are many retries.
                threading.Thread(
                    target=self.run_inference,
                    args=(instruction,),
                    daemon=True,
                ).start()
            else:
                self.get_logger().error(
                    f"VLM stayed UNCERTAIN for more than {self.max_uncertain_retries_} retries. "
                    f"Giving up, returning to IDLE.")
                with self.state_lock_:
                    self.state_ = VlmState.IDLE
                    self.uncertain_retry_count_ = 0
                    self.previous_locked_id_ = None
            return

        # --- NOT_FOUND -> return to IDLE immediately, old ID (if any) is no longer valid ---
        self.get_logger().warn(f"VLM returned {decision.status.value}. Returning to IDLE.")
        with self.state_lock_:
            self.state_ = VlmState.IDLE
            self.uncertain_retry_count_ = 0
            self.previous_locked_id_ = None

    def wait_for_fresh_inputs(self):
        """
        Wait until BOTH bbox_img and tracks are fresh enough
        (<= max_image_age_sec_), up to image_wait_timeout_sec_.
        Returns (bbox_img, valid_ids), or (None, None) on timeout.
        """
        deadline = time.time() + self.image_wait_timeout_sec_
        while time.time() < deadline:
            now = self.get_clock().now()

            bbox_fresh = (
                self.latest_bbox_img_ is not None and
                (now - self.latest_bbox_stamp_).nanoseconds / 1e9 <= self.max_image_age_sec_
            )
            tracks_fresh = (
                self.latest_tracks_ is not None and
                (now - self.latest_tracks_stamp_).nanoseconds / 1e9 <= self.max_image_age_sec_
            )

            if bbox_fresh and tracks_fresh:
                valid_ids = [obj.track_id for obj in self.latest_tracks_.objects]
                return self.latest_bbox_img_, valid_ids

            time.sleep(0.05)
        return None, None

    def imgmsg_to_base64(self, img_msg: Image) -> str:
        cv_image = self.bridge_.imgmsg_to_cv2(img_msg, "bgr8")
        _, buffer = cv2.imencode('.jpg', cv_image)
        return base64.b64encode(buffer).decode('utf-8')

    def build_payload(self, system_prompt: str, bbox_b64: str) -> dict:
        return {
            "messages": [
                {"role": "system", "content": system_prompt},
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "image_url",
                            "image_url": {"url": f"data:image/jpeg;base64,{bbox_b64}"},
                        },
                    ],
                },
            ],
            "max_tokens": 256,
            "top_p": 0.9,
            "temperature": 0.1,
            "cache_prompt": False,
        }


def main(args=None):
    rclpy.init(args=args)
    node = VLMClientNode()

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
