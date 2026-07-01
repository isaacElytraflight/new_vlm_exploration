"""Ollama local VLM backend (host-side inference)."""

from __future__ import annotations

import json
import logging
import os
import time
from typing import Sequence
from urllib.parse import urljoin

import requests
from sensor_msgs.msg import CompressedImage

from explorer_mission.vlm.parsing import compressed_to_base64_jpeg

LOG = logging.getLogger(__name__)

DEFAULT_OLLAMA_URL = "http://host.docker.internal:11434"
DEFAULT_MODEL = "qwen2.5vl:3b"
DEFAULT_MAX_EDGE = 512
DEFAULT_TIMEOUT_S = 30
DEFAULT_NUM_PREDICT = 64


class OllamaBackendError(RuntimeError):
    """Raised when Ollama is unreachable or misconfigured."""


class OllamaBackend:
    name = "local"

    def __init__(
        self,
        base_url: str | None = None,
        model: str | None = None,
        max_edge: int | None = None,
        timeout_s: float | None = None,
        num_predict: int | None = None,
        warmup: bool = True,
    ) -> None:
        self._base_url = (base_url or os.getenv("VLM_OLLAMA_URL", DEFAULT_OLLAMA_URL)).rstrip("/")
        self._model = model or os.getenv("VLM_LOCAL_MODEL", DEFAULT_MODEL)
        self._max_edge = int(max_edge or os.getenv("VLM_LOCAL_MAX_EDGE", DEFAULT_MAX_EDGE))
        self._timeout_s = float(timeout_s or os.getenv("VLM_LOCAL_TIMEOUT_S", DEFAULT_TIMEOUT_S))
        self._num_predict = int(num_predict or os.getenv("VLM_LOCAL_NUM_PREDICT", DEFAULT_NUM_PREDICT))
        self._warmup = warmup

    @property
    def model_label(self) -> str:
        return self._model

    def validate(self) -> None:
        try:
            response = requests.get(
                urljoin(self._base_url + "/", "api/tags"),
                timeout=min(self._timeout_s, 15.0),
            )
        except requests.RequestException as exc:
            raise OllamaBackendError(
                f"Cannot reach Ollama at {self._base_url}. Install Ollama on the host and "
                f"ensure it is running. Details: {exc}"
            ) from exc

        if not response.ok:
            raise OllamaBackendError(
                f"Ollama at {self._base_url} returned HTTP {response.status_code}."
            )

        try:
            payload = response.json()
        except json.JSONDecodeError as exc:
            raise OllamaBackendError("Ollama /api/tags returned invalid JSON.") from exc

        available = {
            str(entry.get("name", "")).split(":")[0]
            for entry in payload.get("models", [])
            if isinstance(entry, dict)
        }
        model_root = self._model.split(":")[0]
        names = {str(entry.get("name", "")) for entry in payload.get("models", []) if isinstance(entry, dict)}
        if names and self._model not in names and model_root not in available:
            raise OllamaBackendError(
                f"Ollama model {self._model!r} is not pulled. Run: ollama pull {self._model}"
            )

        if self._warmup:
            self._run_warmup()

    def _run_warmup(self) -> None:
        try:
            requests.post(
                urljoin(self._base_url + "/", "api/chat"),
                json={
                    "model": self._model,
                    "messages": [{"role": "user", "content": "Reply with 0."}],
                    "stream": False,
                    "think": False,
                    "options": {"num_predict": 8},
                },
                timeout=self._timeout_s,
            )
        except requests.RequestException as exc:
            LOG.warning("Ollama warmup request failed (continuing): %s", exc)

    def query(self, prompt: str, images: Sequence[tuple[str, CompressedImage]]) -> str:
        image_b64_list: list[str] = []
        caption_lines = [prompt, ""]
        for caption, msg in images:
            caption_lines.append(caption)
            image_b64_list.append(compressed_to_base64_jpeg(msg, max_edge=self._max_edge))

        content = "\n".join(caption_lines)
        started = time.perf_counter()
        try:
            response = requests.post(
                urljoin(self._base_url + "/", "api/chat"),
                json={
                    "model": self._model,
                    "messages": [{"role": "user", "content": content, "images": image_b64_list}],
                    "stream": False,
                    "think": False,
                    "options": {"num_predict": self._num_predict},
                },
                timeout=self._timeout_s,
            )
        except requests.RequestException as exc:
            raise OllamaBackendError(f"Ollama chat request failed: {exc}") from exc

        elapsed_s = time.perf_counter() - started
        if not response.ok:
            detail = response.text.strip()[:240] if response.text else response.reason
            raise OllamaBackendError(
                f"Ollama chat failed ({response.status_code}): {detail}"
            )

        try:
            payload = response.json()
        except json.JSONDecodeError as exc:
            raise OllamaBackendError("Ollama chat returned invalid JSON.") from exc

        message = payload.get("message", {})
        text = message.get("content", "") if isinstance(message, dict) else ""
        if not str(text).strip():
            raise RuntimeError(f"Ollama returned empty response: {payload}")

        LOG.info("Ollama response in %.2fs (model=%s)", elapsed_s, self._model)
        return str(text)
