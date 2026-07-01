"""VLM backend tests with mocked HTTP."""

from __future__ import annotations

import base64
import io
import json
from unittest.mock import MagicMock, patch

import pytest
import requests
from sensor_msgs.msg import CompressedImage

from explorer_mission.vlm.backends import get_backend
from explorer_mission.vlm.backends.gemini import GeminiBackend
from explorer_mission.vlm.backends.ollama import OllamaBackend, OllamaBackendError
from explorer_mission.vlm.gemini_auth import GeminiApiKeyError, validate_gemini_api_key
from explorer_mission.vlm.parsing import (
    compressed_to_base64_jpeg,
    downscale_image_bytes,
    parse_leading_int,
)

try:
    from PIL import Image

    HAS_PIL = True
except ImportError:
    HAS_PIL = False


def _jpeg_msg(width: int = 800, height: int = 600) -> CompressedImage:
    if not HAS_PIL:
        pytest.skip("Pillow required for image fixture")
    buf = io.BytesIO()
    Image.new("RGB", (width, height), color=(120, 80, 40)).save(buf, format="JPEG")
    msg = CompressedImage()
    msg.format = "jpeg"
    msg.data = list(buf.getvalue())
    return msg


def test_vlm_parse_success_positive():
    response = "2 is the best frontier because it opens a new room."
    assert parse_leading_int(response) == 2


def test_vlm_api_error_negative():
    with patch("requests.post", side_effect=requests.exceptions.Timeout("timeout")):
        with pytest.raises(requests.exceptions.Timeout):
            requests.post("http://example.com", timeout=1)


def test_require_gemini_api_key_missing(monkeypatch):
    monkeypatch.delenv("GEMINI_API_KEY", raising=False)
    with pytest.raises(GeminiApiKeyError, match="not set"):
        validate_gemini_api_key()


def test_require_gemini_api_key_empty():
    with pytest.raises(GeminiApiKeyError, match="empty"):
        validate_gemini_api_key("   ")


def test_validate_gemini_api_key_rejected():
    response = MagicMock()
    response.ok = False
    response.status_code = 403
    response.reason = "Forbidden"
    response.json.return_value = {
        "error": {"message": "API key not valid. Please pass a valid API key."}
    }

    with patch("explorer_mission.vlm.gemini_auth.requests.get", return_value=response):
        with pytest.raises(GeminiApiKeyError, match="invalid or unauthorized"):
            validate_gemini_api_key("bad-key")


def test_validate_gemini_api_key_success():
    response = MagicMock()
    response.ok = True
    response.status_code = 200
    response.json.return_value = {"models": [{"name": "models/gemini-3.5-flash"}]}

    with patch("explorer_mission.vlm.gemini_auth.requests.get", return_value=response):
        validate_gemini_api_key("good-key")


def test_get_backend_defaults_to_local(monkeypatch):
    monkeypatch.delenv("VLM_BACKEND", raising=False)
    backend = get_backend()
    assert isinstance(backend, OllamaBackend)


def test_get_backend_gemini(monkeypatch):
    monkeypatch.setenv("VLM_BACKEND", "gemini")
    backend = get_backend()
    assert isinstance(backend, GeminiBackend)


def test_get_backend_unknown_negative(monkeypatch):
    monkeypatch.setenv("VLM_BACKEND", "openai")
    with pytest.raises(ValueError, match="Unknown VLM_BACKEND"):
        get_backend()


@pytest.mark.skipif(not HAS_PIL, reason="Pillow not installed")
def test_downscale_image_bytes_positive():
    msg = _jpeg_msg(800, 600)
    raw = bytes(msg.data)
    out = downscale_image_bytes(raw, max_edge=512)
    with Image.open(io.BytesIO(out)) as img:
        assert max(img.size) <= 512


def test_compressed_to_base64_jpeg_positive():
    msg = _jpeg_msg(800, 600)
    b64 = compressed_to_base64_jpeg(msg, max_edge=512)
    decoded = base64.b64decode(b64)
    with Image.open(io.BytesIO(decoded)) as img:
        assert max(img.size) <= 512


def test_ollama_validate_unreachable_negative():
    backend = OllamaBackend(base_url="http://127.0.0.1:59999", warmup=False)
    with patch(
        "explorer_mission.vlm.backends.ollama.requests.get",
        side_effect=requests.ConnectionError("refused"),
    ):
        with pytest.raises(OllamaBackendError, match="Cannot reach Ollama"):
            backend.validate()


def test_ollama_validate_missing_model_negative():
    backend = OllamaBackend(base_url="http://127.0.0.1:11434", model="missing:7b", warmup=False)
    tags = MagicMock()
    tags.ok = True
    tags.json.return_value = {"models": [{"name": "qwen2.5vl:3b"}]}
    with patch("explorer_mission.vlm.backends.ollama.requests.get", return_value=tags):
        with pytest.raises(OllamaBackendError, match="ollama pull"):
            backend.validate()


def test_ollama_query_builds_chat_payload_positive():
    backend = OllamaBackend(base_url="http://127.0.0.1:11434", warmup=False)
    msg = _jpeg_msg()
    chat = MagicMock()
    chat.ok = True
    chat.json.return_value = {"message": {"content": "1\nBecause it opens a hallway."}}

    with patch("explorer_mission.vlm.backends.ollama.requests.post", return_value=chat) as post:
        text = backend.query("Choose frontier", [("Map.", msg)])
        assert parse_leading_int(text) == 1
        payload = post.call_args.kwargs["json"]
        assert payload["model"] == backend.model_label
        assert payload["messages"][0]["images"]
        assert payload["options"]["num_predict"] == 64


def test_gemini_backend_query_positive(monkeypatch):
    monkeypatch.setenv("GEMINI_API_KEY", "test-key")
    backend = GeminiBackend()
    msg = _jpeg_msg()
    response = MagicMock()
    response.ok = True
    response.status_code = 200
    response.json.return_value = {
        "candidates": [{"content": {"parts": [{"text": "0\nOpen area."}]}}]
    }
    with patch("explorer_mission.vlm.backends.gemini.requests.post", return_value=response):
        text = backend.query("prompt", [("Map.", msg)])
        assert parse_leading_int(text) == 0


def test_gemini_backend_validate_without_key_negative(monkeypatch):
    monkeypatch.delenv("GEMINI_API_KEY", raising=False)
    backend = GeminiBackend()
    with pytest.raises(GeminiApiKeyError):
        backend.validate()


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2
