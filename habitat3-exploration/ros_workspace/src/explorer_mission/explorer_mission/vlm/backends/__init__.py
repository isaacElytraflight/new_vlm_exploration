"""VLM backend factory."""

from __future__ import annotations

import os

from explorer_mission.vlm.backends.base import VlmBackend
from explorer_mission.vlm.backends.gemini import GeminiBackend
from explorer_mission.vlm.backends.ollama import OllamaBackend

DEFAULT_BACKEND = "local"


def get_backend() -> VlmBackend:
    backend = os.getenv("VLM_BACKEND", DEFAULT_BACKEND).strip().lower() or DEFAULT_BACKEND
    if backend in {"local", "ollama"}:
        return OllamaBackend()
    if backend == "gemini":
        return GeminiBackend()
    raise ValueError(
        f"Unknown VLM_BACKEND={backend!r}. Expected 'local' (default) or 'gemini'."
    )
