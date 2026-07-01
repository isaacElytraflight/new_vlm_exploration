"""Gemini REST VLM backend."""

from __future__ import annotations

import json
import time
from typing import Any, List, Sequence

import requests
from sensor_msgs.msg import CompressedImage

from explorer_mission.vlm.gemini_auth import (
    GeminiApiKeyError,
    VISION_MODEL,
    _api_error_detail,
    require_gemini_api_key,
    validate_gemini_api_key,
)
from explorer_mission.vlm.parsing import compressed_to_part


class GeminiBackend:
    name = "gemini"

    def __init__(self, model: str = VISION_MODEL, max_retries: int = 3, wait_s: int = 5) -> None:
        self._model = model
        self._max_retries = max_retries
        self._wait_s = wait_s

    @property
    def model_label(self) -> str:
        return self._model

    def validate(self) -> None:
        validate_gemini_api_key()

    def query(self, prompt: str, images: Sequence[tuple[str, CompressedImage]]) -> str:
        parts: List[Any] = [{"text": prompt}]
        for caption, msg in images:
            img_part, caption_part = compressed_to_part(msg, caption)
            parts.extend([img_part, caption_part])
        return self._safe_send(parts)

    def _safe_send(self, parts: List[Any]) -> str:
        api_key = require_gemini_api_key()
        api_url = (
            f"https://generativelanguage.googleapis.com/v1beta/models/"
            f"{self._model}:generateContent?key={api_key}"
        )
        headers = {"Content-Type": "application/json"}
        payload = json.dumps({"contents": [{"parts": parts}]})

        last_exc: Exception | None = None
        for attempt in range(1, self._max_retries + 1):
            try:
                response = requests.post(api_url, headers=headers, data=payload, timeout=60)
                if response.status_code in {400, 401, 403}:
                    raise GeminiApiKeyError(
                        "Gemini API rejected the request: "
                        f"{_api_error_detail(response)}"
                    )
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
            except GeminiApiKeyError:
                raise
            except Exception as exc:
                last_exc = exc
                if attempt == self._max_retries:
                    raise
                time.sleep(self._wait_s)
        if last_exc is not None:
            raise RuntimeError(str(last_exc)) from last_exc
        raise RuntimeError("VLM request failed without a captured error.")
