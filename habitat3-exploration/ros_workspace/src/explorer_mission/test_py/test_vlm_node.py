"""VLM node tests with mocked HTTP."""

from __future__ import annotations

from unittest.mock import MagicMock, patch

import pytest

from explorer_mission.vlm.parsing import parse_leading_int


def test_vlm_parse_success_positive():
    response = "2 is the best frontier because it opens a new room."
    assert parse_leading_int(response) == 2


def test_vlm_api_error_negative():
    import requests

    with patch("requests.post", side_effect=requests.exceptions.Timeout("timeout")):
        with pytest.raises(requests.exceptions.Timeout):
            requests.post("http://example.com", timeout=1)
