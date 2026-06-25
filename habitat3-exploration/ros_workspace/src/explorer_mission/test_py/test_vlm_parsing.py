"""Unit tests for VLM parsing helpers."""

import pytest
from explorer_mission.vlm.parsing import parse_leading_int


def test_parse_leading_int_positive():
    assert parse_leading_int("The answer is 3") == 3


def test_parse_leading_int_negative():
    with pytest.raises(ValueError, match="No integer found"):
        parse_leading_int("no number here")
