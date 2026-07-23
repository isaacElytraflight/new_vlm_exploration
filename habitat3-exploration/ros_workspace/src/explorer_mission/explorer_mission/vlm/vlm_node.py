#!/usr/bin/env python3
"""VLM frontier openness rating action server."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor, as_completed
import os

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from explorer_msgs.action import RateFrontierOpenness
from explorer_msgs.msg import FrontierOpennessScores

from explorer_mission.vlm.backends import get_backend
from explorer_mission.vlm.backends.ollama import OllamaBackendError
from explorer_mission.vlm.gemini_auth import GeminiApiKeyError
from explorer_mission.vlm.parsing import parse_openness_result

OPENNESS_PROMPT = (
    "You are a small, ground-based exploration robot mapping the inside of a building. "
    "You cannot open doors, climb over objects, or use stairs.\n\n"
    "Analyze the provided image of a \"frontier\" (an unmapped direction) and rate how "
    "OPEN or EXPANSIVE the space ahead is on a scale from 0 to 5. Use the strict criteria below:\n\n"
    "### RATING SCALE:\n\n"
    "* **0: Blocked / Dead End**\n"
    "    * *Criteria:* The path is physically blocked, or it is a definitive dead end.\n"
    "    * *Visual Cues:* Closed doors, stairs, furniture/clutter blocking the floor, "
    "or a wall directly ahead with no turns.\n"
    "* **1: Shallow / Minor Space**\n"
    "    * *Criteria:* Accessible, but clearly ends very soon. Exploring it requires immediate backtracking.\n"
    "    * *Visual Cues:* The visible end of a short hallway a few feet away, or a very small closet/nook.\n"
    "* **2: Transitional Path**\n"
    "    * *Criteria:* A narrow path that leads deeper into the building or around a corner.\n"
    "    * *Visual Cues:* A long hallway, a doorway leading into a standard-sized room, "
    "or a hallway that turns out of sight.\n"
    "* **3: Large Finite Room**\n"
    "    * *Criteria:* A wide, open, but clearly bounded indoor area.\n"
    "    * *Visual Cues:* Large living rooms, laboratories, classrooms, foyers, or lobbies.\n"
    "* **4: Major Hub Area**\n"
    "    * *Criteria:* A massive open indoor space that itself branches off into multiple other rooms or hallways.\n"
    "    * *Visual Cues:* A central atrium, a main intersection of multiple hallways, "
    "or a large open-plan office grid.\n"
    "* **5: Unbounded / Outdoors**\n"
    "    * *Criteria:* The space is theoretically infinite or exits the building framework entirely.\n"
    "    * *Visual Cues:* An open exit leading outdoors, a loading dock opening to the outside, "
    "or a massive warehouse-scale environment.\n\n"
    "### OUTPUT FORMAT:\n"
    "Reply with a single JSON object. Put the integer score FIRST so short replies "
    "still parse if truncated. No markdown fences, no other text.\n\n"
    "{\n"
    "  \"score\": <integer 0-5>,\n"
    "  \"reasoning\": \"one short sentence\"\n"
    "}"
)


class VlmNode(Node):
    def __init__(self) -> None:
        super().__init__("vlm_server")
        self._backend = get_backend()
        self._backend.validate()
        self._scores_pub = self.create_publisher(
            FrontierOpennessScores, "exploration/vlm/scores", 10
        )
        self._server = ActionServer(
            self,
            RateFrontierOpenness,
            "vlm/rate_frontiers",
            execute_callback=self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info(
            f"VLM openness server ready (backend={self._backend.name}, model={self._backend.model_label})"
        )

    def goal_callback(self, _goal_request: RateFrontierOpenness.Goal) -> GoalResponse:
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _rate_one(self, frontier_id: int, image) -> tuple[int, int, str]:
        reasoning = ""
        try:
            response = self._backend.query(
                OPENNESS_PROMPT,
                [(f"Frontier {frontier_id}.", image)],
            )
            result = parse_openness_result(response)
            score = result.score
            reasoning = result.reasoning
        except Exception as exc:
            # Do NOT use score 0 — that marks the frontier fully_explored and
            # permanently drops it. Soft-fail to 1 so DFS can still try it.
            self.get_logger().error(
                f"Frontier {frontier_id} rating failed ({exc}); soft-fail score=1"
            )
            score = 1
            reasoning = f"rating failed: {exc}"
        self.get_logger().info(
            f"Frontier {frontier_id} openness score={score}"
            + (f" ({reasoning})" if reasoning else "")
        )
        return frontier_id, score, reasoning

    def execute(self, goal_handle):
        goal = goal_handle.request
        feedback = RateFrontierOpenness.Feedback()
        feedback.status = 0
        goal_handle.publish_feedback(feedback)

        if len(goal.images) != len(goal.frontier_ids):
            self.get_logger().error("RateFrontierOpenness goal size mismatch")
            goal_handle.abort()
            return RateFrontierOpenness.Result()

        feedback.status = 1
        goal_handle.publish_feedback(feedback)

        scores_by_id: dict[int, int] = {}
        reasonings_by_id: dict[int, str] = {}
        # Ollama on a laptop cannot sustain 8 concurrent VL requests — that caused
        # mass Read timed out → fake score 0 → frontiers permanently dropped.
        max_workers = max(
            1,
            min(len(goal.images), int(os.getenv("VLM_MAX_WORKERS", "2"))),
        )
        with ThreadPoolExecutor(max_workers=max_workers) as pool:
            futures = [
                pool.submit(self._rate_one, int(fid), image)
                for fid, image in zip(goal.frontier_ids, goal.images)
            ]
            for future in as_completed(futures):
                frontier_id, score, reasoning = future.result()
                scores_by_id[frontier_id] = score
                reasonings_by_id[frontier_id] = reasoning
                # Stream partial scores so explore can early-nav on score >= 3.
                partial = FrontierOpennessScores()
                partial.frontier_ids = [int(frontier_id)]
                partial.scores = [int(score)]
                partial.reasonings = [str(reasoning)]
                self._scores_pub.publish(partial)

        feedback.status = 2
        goal_handle.publish_feedback(feedback)

        result = RateFrontierOpenness.Result()
        for fid in goal.frontier_ids:
            result.frontier_ids.append(int(fid))
            result.scores.append(int(scores_by_id[int(fid)]))
            result.reasonings.append(str(reasonings_by_id.get(int(fid), "")))
        goal_handle.succeed()
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    logger = rclpy.logging.get_logger("vlm_server")
    try:
        node = VlmNode()
    except (GeminiApiKeyError, OllamaBackendError, ValueError) as exc:
        logger.fatal(str(exc))
        rclpy.shutdown()
        raise SystemExit(1) from exc
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
