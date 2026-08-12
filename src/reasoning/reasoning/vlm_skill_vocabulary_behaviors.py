#!/usr/bin/env python3
"""
vlm_skill_vocabulary.py

Skill vocabulary for the VLM in the VLA pipeline (used by
reasoning/vlm_client_node.py).

DESIGN: a single VLM call now decides BOTH "who" (target_id) AND "what to
do" (behavior) from one camera image + one instruction. The two are kept
in the same JSON response (one prompt, one inference) per the project's
choice to minimize latency/cost, at the accepted tradeoff that a small
(~2B) model sharing its reasoning budget across two decisions is slightly
more failure-prone than two separate calls would be. To keep that risk
contained, the "behavior" field is intentionally a FLAT CHOICE among 5
fixed strings, not a nested plan - the VLM never has to produce a
correctly-nested JSON tree. Recovery chains (GO_TO_LAST_SEEN -> ROTATE_SCAN)
and the RETURN_HOME fallback are attached in code (see build_task_tree()
below), never chosen by the model itself.

Source of valid_ids: topic /tracked_persons_depth
(perception/msg/TrackedObject3DArray), field named `track_id`.

Rules:
  1. The VLM must only return JSON matching exactly one of the 3 predefined
     status forms (LOCKED / NOT_FOUND / UNCERTAIN).
  2. The VLM must not "invent" a track_id outside the real valid range.
  3. "behavior" must be one of SELECTABLE_BEHAVIORS; an invalid or missing
     value is NOT treated as fatal (unlike an invalid target_id) - it is
     silently defaulted to APPROACH_TARGET with a warning log, since a
     wrong movement-style choice is a much smaller safety concern than a
     wrong target_id, and discarding an otherwise-correct target_id over a
     minor formatting slip in a secondary field would waste a full
     inference cycle for no benefit.
  4. "wait_seconds" (for WAIT) and "keep_distance_m" (for KEEP_DISTANCE) are
     OPTIONAL numbers the VLM fills in only when the command states them
     (e.g. "wait 5 seconds", "keep 2 meters away"). Missing/invalid values
     fall back to behavior_executive's own config defaults - same lenient
     treatment as "behavior" above.
  5. Every output must go through parse_and_validate() before being trusted.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional, List, Dict, Any
import json
import logging

logger = logging.getLogger("vlm_skill_vocabulary")


# ============================================================
# 1. SKILL VOCABULARY
# ============================================================

class VLMStatus(str, Enum):
    LOCKED = "LOCKED"          # Found exactly one target matching the command
    NOT_FOUND = "NOT_FOUND"    # No track_id matches in the current frame
    UNCERTAIN = "UNCERTAIN"    # >=2 similar candidates, not confident enough to lock


# The only behaviors the VLM is allowed to pick directly. Recovery/fallback
# behaviors (GO_TO_LAST_SEEN, ROTATE_SCAN, SEARCH_TARGET, REPLAN, ABORT_TASK,
# RETURN_HOME) are attached automatically by build_task_tree() below, never
# selected by the model.
SELECTABLE_BEHAVIORS = [
    "APPROACH_TARGET",
    "FOLLOW_TARGET",
    "KEEP_DISTANCE",
    "WAIT",
    "STOP",
]

# Behaviors that involve continuously tracking a moving target - these are
# the only ones that get an automatic on_failure recovery chain attached
# (WAIT/STOP don't lose a target the way tracking behaviors do).
_TRACKING_BEHAVIORS = {"APPROACH_TARGET", "FOLLOW_TARGET", "KEEP_DISTANCE"}


# ============================================================
# 2. OUTPUT SCHEMA
# ============================================================

@dataclass
class VLMDecision:
    status: VLMStatus
    target_id: Optional[int] = None
    candidate_ids: Optional[List[int]] = None
    behavior: Optional[str] = None                 # only set when status == LOCKED
    return_home_if_not_found: bool = False          # only meaningful when status == LOCKED
    wait_seconds: Optional[float] = None            # only meaningful when behavior == WAIT
    keep_distance_m: Optional[float] = None         # only meaningful when behavior == KEEP_DISTANCE
    description: Optional[str] = None   # VLM's own reasoning for the choice - logging/debug only
    reason: Optional[str] = None        # internal error (parse_error, fallback...) - logging/debug only


# ============================================================
# 3. PROMPT — enforce JSON format, single camera image, who + what-to-do
# ============================================================

SYSTEM_PROMPT_TEMPLATE = """
You are the VLM perception + behavior-selection module of a mobile robot.

You receive one front-facing camera image and a user command.

- Each detected person has a bounding box and an ID.
- User command: "{instruction}"
- Valid IDs: {valid_ids}

Your task:

1. Internally analyze every person in the image.
   Consider:
   - ID
   - posture (standing, sitting, ...)
   - clothing
   - object being held
   - surrounding context

2. Internally compare every person with the user command.

3. If exactly one person matches, ALSO decide which behavior the robot
   should run, based on what the command asks for:

   - APPROACH_TARGET: walk up to the person and stop nearby (e.g. "go to",
     "come here", "walk toward", or no explicit movement style stated).
   - FOLLOW_TARGET: keep following the person as they move, do not stop
     when first reaching them (e.g. "follow", "go with", "escort", "come
     along with").
   - KEEP_DISTANCE: stay near the person but do not get too close (e.g.
     "keep an eye on", "watch from a distance", "don't get too close",
     "observe").
   - WAIT: stay in place / do not move toward anyone (e.g. "wait here",
     "stay", "don't move").
   - STOP: stop moving immediately (e.g. "stop", "halt").

   If the command is ambiguous about movement style, default to
   APPROACH_TARGET.

4. Also decide "return_home_if_not_found": true ONLY if the command
   explicitly says to return/go back in case the person isn't found (e.g.
   "... if you don't find them, come back" / "... otherwise return here").
   Otherwise false.

5. If behavior is WAIT and the command states a duration (e.g. "wait 5
   seconds", "stay for 10s"), also fill "wait_seconds" with that number.
   Otherwise omit it (or set it to null) - do NOT invent a number.

6. If behavior is KEEP_DISTANCE and the command states a distance (e.g.
   "keep 2 meters away", "stay 3m back"), also fill "keep_distance_m" with
   that number (in meters). Otherwise omit it (or set it to null) - do NOT
   invent a number.

7. Do NOT output your internal reasoning.

Return ONLY one JSON object.

If no person matches:

{{
    "description":"No matching person found.",
    "status":"NOT_FOUND"
}}

If multiple people match:

{{
    "description":"Briefly summarize why multiple candidates match (max 2 sentences).",
    "status":"UNCERTAIN",
    "candidate_ids":[id1,id2,...]
}}

If exactly one person matches:

{{
    "description":"Briefly summarize why the selected person matches (max 2 sentences).",
    "status":"LOCKED",
    "target_id":id,
    "behavior":"<one of: APPROACH_TARGET, FOLLOW_TARGET, KEEP_DISTANCE, WAIT, STOP>",
    "wait_seconds":number or null,
    "keep_distance_m":number or null,
    "return_home_if_not_found":true or false
}}

Rules:

- description MUST be concise.
- description MUST NOT exceed 2 sentences.
- Do NOT repeat information.
- Do NOT describe every person in detail.
- Do NOT output your reasoning process.
- target_id MUST belong to {valid_ids}.
- behavior MUST be exactly one of: APPROACH_TARGET, FOLLOW_TARGET, KEEP_DISTANCE, WAIT, STOP.
- wait_seconds and keep_distance_m MUST be null unless the command explicitly states that number.
- If uncertain about WHO matches, return UNCERTAIN instead of LOCKED.
- Return ONLY a single JSON object.
- No markdown.
- No code fences.
"""

def build_system_prompt(instruction: str, valid_ids: List[int]) -> str:
    return SYSTEM_PROMPT_TEMPLATE.format(
        instruction=instruction,
        valid_ids=valid_ids,
    )


# ============================================================
# 4. PARSER + VALIDATOR
# ============================================================

class VLMOutputError(Exception):
    """Raised when the VLM output cannot be parsed, or violates a real-world data constraint."""
    pass


def parse_and_validate(raw_text: str, valid_ids: List[int]) -> VLMDecision:
    text = raw_text.strip()

    start = text.find('{')
    end = text.rfind('}')
    if start == -1 or end == -1:
        raise VLMOutputError(f"No JSON found in output: '{text}'")

    try:
        data = json.loads(text[start:end + 1])
    except json.JSONDecodeError as e:
        raise VLMOutputError(f"Invalid JSON: '{text}' ({e})")

    status_raw = data.get("status")
    valid_status_values = {s.value for s in VLMStatus}
    if status_raw not in valid_status_values:
        raise VLMOutputError(f"'status' field missing or invalid: {data}")
    status = VLMStatus(status_raw)
    description = data.get("description")

    # --- LOCKED: requires target_id (strict), behavior + return_home_if_not_found (lenient) ---
    if status == VLMStatus.LOCKED:
        if "target_id" not in data:
            raise VLMOutputError(f"LOCKED is missing 'target_id' field: {data}")
        try:
            target_id = int(data["target_id"])
        except (ValueError, TypeError):
            raise VLMOutputError(f"'target_id' is not an integer: {data}")

        if target_id not in valid_ids:
            raise VLMOutputError(
                f"target_id={target_id} DOES NOT exist in valid_ids={valid_ids} "
                f"-- suspected hallucination."
            )

        # 'behavior' and 'return_home_if_not_found' are validated LENIENTLY -
        # an invalid/missing value here does not invalidate the whole
        # decision, it just falls back to a safe default (see module docstring).
        behavior = data.get("behavior")
        if behavior not in SELECTABLE_BEHAVIORS:
            logger.warning(
                f"[Invalid VLM 'behavior' field] got {behavior!r}, expected one of "
                f"{SELECTABLE_BEHAVIORS}. Defaulting to APPROACH_TARGET."
            )
            behavior = "APPROACH_TARGET"

        return_home_raw = data.get("return_home_if_not_found", False)
        return_home_if_not_found = return_home_raw if isinstance(return_home_raw, bool) else False

        # Numeric params are OPTIONAL and validated leniently, same spirit as
        # 'behavior' above: a missing/invalid number just falls back to
        # behavior_executive's own config default (see build_task_tree()),
        # it never invalidates an otherwise-correct target_id decision.
        wait_seconds_raw = data.get("wait_seconds")
        wait_seconds = (
            float(wait_seconds_raw)
            if isinstance(wait_seconds_raw, (int, float)) and wait_seconds_raw > 0
            else None
        )

        keep_distance_raw = data.get("keep_distance_m")
        keep_distance_m = (
            float(keep_distance_raw)
            if isinstance(keep_distance_raw, (int, float)) and keep_distance_raw > 0
            else None
        )

        return VLMDecision(
            status=status,
            target_id=target_id,
            behavior=behavior,
            return_home_if_not_found=return_home_if_not_found,
            wait_seconds=wait_seconds,
            keep_distance_m=keep_distance_m,
            description=description,
        )

    # --- UNCERTAIN: requires candidate_ids as a list of >=2 elements, all must be valid ---
    if status == VLMStatus.UNCERTAIN:
        candidates = data.get("candidate_ids")
        if not isinstance(candidates, list) or len(candidates) < 2:
            raise VLMOutputError(
                f"UNCERTAIN requires candidate_ids as a list of >=2 elements, got: {data}"
            )
        try:
            candidate_ids = [int(c) for c in candidates]
        except (ValueError, TypeError):
            raise VLMOutputError(f"candidate_ids contains a non-integer element: {data}")

        invalid = [c for c in candidate_ids if c not in valid_ids]
        if invalid:
            raise VLMOutputError(
                f"candidate_ids contains ID(s) that don't exist: {invalid} (valid={valid_ids})."
            )
        return VLMDecision(status=status, candidate_ids=candidate_ids, description=description)

    # --- NOT_FOUND: no extra fields needed ---
    return VLMDecision(status=status, description=description)


# ============================================================
# 5. SAFE FALLBACK
# ============================================================

def safe_get_decision(
    raw_text: str,
    valid_ids: List[int],
    previous_locked_id: Optional[int] = None,
) -> VLMDecision:
    """
    The only wrapper vlm_client_node.py needs to call. Never raises outward.

    Fallback on parse error:
      - If a target was previously locked (previous_locked_id) and that ID
        is still in valid_ids -> return LOCKED with that same target_id,
        defaulting behavior to APPROACH_TARGET and return_home_if_not_found
        to False (the conservative choice when we can't tell what the model
        actually intended).
      - Otherwise -> return UNCERTAIN, do not guess.
    """
    try:
        return parse_and_validate(raw_text, valid_ids)
    except VLMOutputError as e:
        logger.warning(f"[Invalid VLM output] {e} -- raw='{raw_text}'")

        if previous_locked_id is not None and previous_locked_id in valid_ids:
            return VLMDecision(
                status=VLMStatus.LOCKED,
                target_id=previous_locked_id,
                behavior="APPROACH_TARGET",
                return_home_if_not_found=False,
                reason=f"fallback_keep_previous: {e}",
            )
        return VLMDecision(status=VLMStatus.UNCERTAIN, reason=f"parse_error: {e}")


# ============================================================
# 6. TASK TREE BUILDER — turns a flat behavior choice into the JSON "tasks"
#    shape expected by behavior_executive/behavior_executor_node.py
# ============================================================

def build_task_tree(
    behavior: str,
    return_home_if_not_found: bool,
    wait_seconds: Optional[float] = None,
    keep_distance_m: Optional[float] = None,
    keep_distance_tolerance_m: float = 0.5,
) -> List[Dict[str, Any]]:
    """
    Turn a Stage-A/B flat behavior choice into the "tasks" list published on
    /vlm/target_command. Recovery chains and the RETURN_HOME fallback are
    attached HERE, in code - never chosen by the VLM itself (see module
    docstring for why). Matches the JSON shape expected by
    behavior_executive.behavior_types.BehaviorTask.from_dict().

    wait_seconds / keep_distance_m come straight from the VLM's own output
    (already lenient-validated by parse_and_validate() - None if the
    command didn't state a number). If None, the resulting task carries no
    override and behavior_executive falls back to its own config defaults
    (WAIT waits until preempted with no timeout; KEEP_DISTANCE uses
    min_distance_m/max_distance_m from behavior_executive_params.yaml).

    keep_distance_m is a single number from the command (e.g. "keep 2m
    away"), but the KEEP_DISTANCE behavior itself needs a [min, max] band -
    so a fixed tolerance is applied around it here in code, rather than
    asking the VLM to produce two numbers (min AND max), which would be a
    harder and more error-prone thing to ask a small model for.
    """
    task: Dict[str, Any] = {"behavior": behavior}

    params: Dict[str, Any] = {}
    if behavior == "WAIT" and wait_seconds is not None:
        params["timeout_sec"] = wait_seconds
    if behavior == "KEEP_DISTANCE" and keep_distance_m is not None:
        params["min_distance_m"] = max(0.0, keep_distance_m - keep_distance_tolerance_m)
        params["max_distance_m"] = keep_distance_m + keep_distance_tolerance_m
    if params:
        task["params"] = params

    if behavior in _TRACKING_BEHAVIORS:
        task["on_failure"] = [
            {"behavior": "GO_TO_LAST_SEEN"},
            {"behavior": "ROTATE_SCAN"},
        ]

    if return_home_if_not_found:
        task["on_exhausted"] = [{"behavior": "RETURN_HOME"}]

    return [task]
