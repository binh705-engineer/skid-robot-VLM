#!/usr/bin/env python3
"""
vlm_skill_vocabulary.py

Skill vocabulary for the VLM in the VLA pipeline (used by
reasoning/vlm_client_node.py).

NEW DESIGN: the VLM ONLY chooses "who" (target_id) from a single camera
image. It no longer chooses a coordinate (grid_n/grid_m/column). Resolving
the real-world coordinate of the chosen target_id (and continuously
tracking it while the robot moves) is now the responsibility of
reasoning/coordinate_mapper_node.py (which reads directly from
/tracked_persons_depth), and is no longer the VLM's job.

Reason for the change: the VLM (a small, 2B model) is prone to mistakes
when it has to reason about space (left/right, offsets...) from a single
static image. Separating "who to pick" (semantic — the VLM is good at
this) from "where they are" (geometric — code handles this accurately and
continuously, not just once) makes the system more stable.

Source of valid_ids: topic /tracked_persons_depth
(perception/msg/TrackedObject3DArray), field named `track_id`.

Rules:
  1. The VLM must only return JSON matching exactly one of the 3 predefined
     status forms (LOCKED / NOT_FOUND / UNCERTAIN — no more LOST, since the
     VLM no longer tracks continuously; detecting "lost" is now
     coordinate_mapper_node.py's job).
  2. The VLM must not "invent" a track_id outside the real valid range.
  3. Every output must go through parse_and_validate() before being trusted.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional, List
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


# ============================================================
# 2. OUTPUT SCHEMA
# ============================================================

@dataclass
class VLMDecision:
    status: VLMStatus
    target_id: Optional[int] = None
    candidate_ids: Optional[List[int]] = None
    description: Optional[str] = None   # VLM's own reasoning for the choice - logging/debug only
    reason: Optional[str] = None        # internal error (parse_error, fallback...) - logging/debug only


# ============================================================
# 3. PROMPT — enforce JSON format, single camera image, no coordinates
# ============================================================

SYSTEM_PROMPT_TEMPLATE = """
You are the VLM perception module of a mobile robot.

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

3. Do NOT output your internal reasoning.

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
    "target_id":id
}}

Rules:

- description MUST be concise.
- description MUST NOT exceed 2 sentences.
- Do NOT repeat information.
- Do NOT describe every person in detail.
- Do NOT output your reasoning process.
- target_id MUST belong to {valid_ids}.
- If uncertain, return UNCERTAIN instead of LOCKED.
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

    # --- LOCKED: requires target_id, must be valid against valid_ids ---
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
        return VLMDecision(status=status, target_id=target_id, description=description)

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
        is still in valid_ids -> return LOCKED with that same target_id
        (this is a COMPLETE decision, no coordinate field needs to be
        checked for None anymore like in the old design - since the VLM
        now only ever decides the target_id).
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
                reason=f"fallback_keep_previous: {e}",
            )
        return VLMDecision(status=VLMStatus.UNCERTAIN, reason=f"parse_error: {e}")
