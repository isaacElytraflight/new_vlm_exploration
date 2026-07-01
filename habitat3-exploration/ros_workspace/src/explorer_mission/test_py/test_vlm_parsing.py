"""Unit tests for VLM parsing helpers."""

import pytest
from explorer_mission.vlm.parsing import parse_leading_int, validate_frontier_choice


def test_parse_leading_int_positive():
    assert parse_leading_int("The answer is 3") == 3


def test_parse_leading_int_negative():
    with pytest.raises(ValueError, match="No integer found"):
        parse_leading_int("no number here")


def test_validate_frontier_choice_accepts_label_id():
    assert validate_frontier_choice(1, [0, 1]) == 1


def test_validate_frontier_choice_rejects_out_of_range():
    with pytest.raises(ValueError, match="not in candidates"):
        validate_frontier_choice(4, [0, 1])
