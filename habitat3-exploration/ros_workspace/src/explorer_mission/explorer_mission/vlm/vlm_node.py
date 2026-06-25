#!/usr/bin/env python3
"""VLM frontier selection action server."""

from __future__ import annotations

import json
import os
import time
from typing import Any, List

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from explorer_msgs.action import FrontierViewsProcess

from explorer_mission.vlm.parsing import compressed_to_part, parse_leading_int

VISION_MODEL = "gemini-3.5-flash"
API_KEY = os.getenv("GEMINI_API_KEY", "")


def safe_send(parts: List[Any], max_retries: int = 3, wait_s: int = 5) -> str:
    """Send a request to the Gemini REST API with retries."""
    import requests

    if not API_KEY:
        raise RuntimeError("GEMINI_API_KEY environment variable is not set")

    api_url = (
        f"https://generativelanguage.googleapis.com/v1beta/models/"
        f"{VISION_MODEL}:generateContent?key={API_KEY}"
    )
    headers = {"Content-Type": "application/json"}
    payload = json.dumps({"contents": [{"parts": parts}]})

    for attempt in range(1, max_retries + 1):
        try:
            response = requests.post(api_url, headers=headers, data=payload, timeout=60)
            response.raise_for_status()
            json_response = response.json()
            candidates = json_response.get("candidates", [])
            if not candidates:
                raise RuntimeError("VLM returned no candidates.")
            content = candidates[0].get("content", {})
            content_parts = content.get("parts", [])
            if not content_parts or "text" not in content_parts[0]:
                raise RuntimeError(f"VLM returned malformed response: {json_response}")
            return content_parts[0]["text"]
        except Exception as exc:
            if attempt == max_retries:
                raise
            time.sleep(wait_s)
            last_exc = exc
    raise RuntimeError(str(last_exc))


class VlmNode(Node):
    def __init__(self) -> None:
        super().__init__("vlm_server")
        self._server = ActionServer(
            self,
            FrontierViewsProcess,
            "vlm/query",
            execute_callback=self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info(f"VLM server ready (model={VISION_MODEL})")

    def goal_callback(self, _goal_request: FrontierViewsProcess.Goal) -> GoalResponse:
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        goal = goal_handle.request
        feedback = FrontierViewsProcess.Feedback()
        feedback.status = 0
        goal_handle.publish_feedback(feedback)

        prompt = (
            "You are an autonomous exploration robot. Your goal is to fully explore/cover "
            "the entire environment as quickly and efficiently as possible. You are given the "
            "current top-down occupancy map and a series of images. Each image corresponds to "
            "a numbered frontier on the map. Choose the single best frontier to navigate toward. "
            "Your answer must be only the number of the chosen frontier (e.g., \"0\", \"1\", etc.) "
            "with absolutely nothing else. Starting on the next line directly after this, "
            "please explain your reasoning."
        )
        parts: List[Any] = [{"text": prompt}]
        map_part, map_caption = compressed_to_part(goal.map, "Current occupancy map.")
        parts.extend([map_part, map_caption])

        for i, (cim, frontier_id) in enumerate(zip(goal.images, goal.frontiers)):
            try:
                img_part, img_caption = compressed_to_part(
                    cim, f"Image for Frontier {int(frontier_id)}."
                )
                parts.extend([img_part, img_caption])
            except Exception as exc:
                self.get_logger().warn(f"Decode failed for image {i}: {exc}")

        feedback.status = 1
        goal_handle.publish_feedback(feedback)

        self.get_logger().info("Asking VLM to choose the best frontier...")
        response = safe_send(parts)
        self.get_logger().info(f"VLM response: {response}")
        chosen = parse_leading_int(response)

        feedback.status = 2
        goal_handle.publish_feedback(feedback)

        result = FrontierViewsProcess.Result()
        result.frontier = chosen
        goal_handle.succeed()
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VlmNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
