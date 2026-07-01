#!/usr/bin/env python3
"""VLM frontier selection action server."""

from __future__ import annotations

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from explorer_msgs.action import FrontierViewsProcess

from explorer_mission.vlm.backends import get_backend
from explorer_mission.vlm.backends.ollama import OllamaBackendError
from explorer_mission.vlm.gemini_auth import GeminiApiKeyError
from explorer_mission.vlm.parsing import parse_leading_int, validate_frontier_choice

FRONTIER_PROMPT = (
    "You are an autonomous exploration robot. Your goal is to fully explore/cover "
    "the entire environment as quickly and efficiently as possible. You are given the "
    "current top-down occupancy map and a series of images. Each image corresponds to "
    "a numbered frontier on the map (labels 0, 1, 2, ...). Choose the single best "
    "frontier to navigate toward. Your answer must be only the label number shown on "
    "the map (e.g., \"0\", \"1\", etc.) with absolutely nothing else. Starting on the "
    "next line directly after this, please explain your reasoning."
)


class VlmNode(Node):
    def __init__(self) -> None:
        super().__init__("vlm_server")
        self._backend = get_backend()
        self._backend.validate()
        self._server = ActionServer(
            self,
            FrontierViewsProcess,
            "vlm/query",
            execute_callback=self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info(
            f"VLM server ready (backend={self._backend.name}, model={self._backend.model_label})"
        )

    def goal_callback(self, _goal_request: FrontierViewsProcess.Goal) -> GoalResponse:
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        goal = goal_handle.request
        feedback = FrontierViewsProcess.Feedback()
        feedback.status = 0
        goal_handle.publish_feedback(feedback)

        images = [("Current occupancy map.", goal.map)]
        for cim, frontier_id in zip(goal.images, goal.frontiers):
            images.append((f"Image for Frontier {int(frontier_id)}.", cim))

        feedback.status = 1
        goal_handle.publish_feedback(feedback)

        self.get_logger().info(
            f"Asking VLM to choose the best frontier (backend={self._backend.name})..."
        )
        response = self._backend.query(FRONTIER_PROMPT, images)
        self.get_logger().info(f"VLM response: {response}")
        chosen = parse_leading_int(response)
        chosen = validate_frontier_choice(chosen, [int(fid) for fid in goal.frontiers])

        feedback.status = 2
        goal_handle.publish_feedback(feedback)

        result = FrontierViewsProcess.Result()
        result.frontier = chosen
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
