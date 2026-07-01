"""Gemini API key validation for the VLM frontier selector."""

from __future__ import annotations

import json
import os
from typing import Any

import requests

VISION_MODEL = "gemini-3.5-flash"
_MODELS_URL = "https://generativelanguage.googleapis.com/v1beta/models"


class GeminiApiKeyError(RuntimeError):
    """Raised when GEMINI_API_KEY is missing or rejected by the Gemini API."""


def require_gemini_api_key() -> str:
    key = os.getenv("GEMINI_API_KEY", "").strip()
    if not key:
        raise GeminiApiKeyError(
            "GEMINI_API_KEY is not set. Add it to habitat3-exploration/sim/.env "
            "(see sim/.env.example) and restart the sim container."
        )
    return key


def _api_error_detail(response: requests.Response) -> str:
    try:
        payload: dict[str, Any] = response.json()
    except (json.JSONDecodeError, ValueError):
        text = response.text.strip()
        return text[:240] if text else response.reason

    error = payload.get("error", {})
    message = error.get("message") if isinstance(error, dict) else None
    if isinstance(message, str) and message.strip():
        return message.strip()
    return response.reason or f"HTTP {response.status_code}"


def validate_gemini_api_key(api_key: str | None = None) -> None:
    """Verify the API key is present and accepted by the Gemini API."""
    key = (api_key if api_key is not None else require_gemini_api_key()).strip()
    if not key:
        raise GeminiApiKeyError(
            "GEMINI_API_KEY is empty. Set a non-empty key in habitat3-exploration/sim/.env."
        )

    try:
        response = requests.get(
            _MODELS_URL,
            params={"key": key},
            timeout=15,
        )
    except requests.RequestException as exc:
        raise GeminiApiKeyError(f"Could not reach Gemini API: {exc}") from exc

    if response.status_code in {400, 401, 403}:
        raise GeminiApiKeyError(
            "GEMINI_API_KEY is invalid or unauthorized: "
            f"{_api_error_detail(response)}"
        )
    if not response.ok:
        raise GeminiApiKeyError(
            "Gemini API key check failed "
            f"({response.status_code}): {_api_error_detail(response)}"
        )

    try:
        response.json()
    except (json.JSONDecodeError, ValueError, AttributeError) as exc:
        raise GeminiApiKeyError(
            "Gemini API returned an unexpected models response."
        ) from exc
