"""Unit tests for VLM parsing helpers."""

import pytest
from explorer_mission.vlm.parsing import (
    parse_leading_int,
    parse_openness_score,
    validate_frontier_choice,
)


def test_parse_leading_int_positive():
    assert parse_leading_int("The answer is 3") == 3


def test_parse_leading_int_negative():
    with pytest.raises(ValueError, match="No integer found"):
        parse_leading_int("no number here")


def test_parse_openness_score_json_positive():
    raw = '{"reasoning": "short hallway", "score": 2}'
    assert parse_openness_score(raw) == 2


def test_parse_openness_score_blocked_zero_positive():
    raw = '{"reasoning": "dead end", "score": 0}'
    assert parse_openness_score(raw) == 0


def test_parse_openness_score_clamps_high_positive():
    raw = '{"reasoning": "huge", "score": 9}'
    assert parse_openness_score(raw) == 5


def test_parse_openness_score_fallback_int_positive():
    assert parse_openness_score("3") == 3


def test_parse_openness_score_malformed_negative():
    with pytest.raises(ValueError, match="No integer found"):
        parse_openness_score("no score here")


def test_validate_frontier_choice_accepts_label_id():
    assert validate_frontier_choice(1, [0, 1]) == 1


def test_validate_frontier_choice_rejects_out_of_range():
    with pytest.raises(ValueError, match="not in candidates"):
        validate_frontier_choice(4, [0, 1])


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2
