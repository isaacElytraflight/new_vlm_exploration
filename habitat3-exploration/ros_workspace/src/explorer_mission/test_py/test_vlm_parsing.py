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
    raw = '{"score": 2, "reasoning": "short hallway"}'
    assert parse_openness_score(raw) == 2


def test_parse_openness_score_blocked_zero_positive():
    raw = '{"score": 0, "reasoning": "dead end"}'
    assert parse_openness_score(raw) == 0


def test_parse_openness_score_clamps_high_positive():
    raw = '{"score": 9, "reasoning": "huge"}'
    assert parse_openness_score(raw) == 5


def test_parse_openness_score_fallback_int_positive():
    assert parse_openness_score("3") == 3


def test_parse_openness_score_truncated_json_with_score_first_positive():
    """Live failure mode: num_predict cuts mid-reasoning after score was emitted."""
    raw = '{\n  "score": 3,\n  "reasoning": "The image shows a large, open indoor space with a checkered flo'
    assert parse_openness_score(raw) == 3


def test_parse_openness_score_truncated_reasoning_first_still_finds_score_positive():
    raw = (
        '{\n  "reasoning": "The image shows a large, open indoor space with a checkered floor, '
        'ornate furniture, and framed portraits on the walls. The room is clearly bounded by walls and '
        'windows but offers significant horizontal and vertical depth. There are no visible '
        'obstacles or dead ends that would restrict movement; it",\n  "score": 4\n}'
    )
    # Complete enough to json-parse, or regex if slightly broken.
    assert parse_openness_score(raw) == 4


def test_parse_openness_score_truncated_no_score_negative():
    """Negative: truncated reasoning-only JSON with no digits must still fail."""
    raw = (
        '{\n  "reasoning": "The image shows a large, open indoor space with a checkered floor, '
        "ornate furniture, and framed portraits on the walls. The room is clearly bounded by walls and "
        "windows but offers significant horizontal and vertical depth. There are no visible "
        "obstacles or dead ends that would restrict movement; it"
    )
    with pytest.raises(ValueError):
        parse_openness_score(raw)


def test_parse_openness_score_malformed_negative():
    with pytest.raises(ValueError, match="No integer found"):
        parse_openness_score("no score here")


def test_parse_openness_result_includes_reasoning_positive():
    from explorer_mission.vlm.parsing import parse_openness_result

    raw = '{"score": 2, "reasoning": "short hallway"}'
    result = parse_openness_result(raw)
    assert result.score == 2
    assert result.reasoning == "short hallway"


def test_parse_openness_result_missing_reasoning_negative():
    from explorer_mission.vlm.parsing import parse_openness_result

    result = parse_openness_result('{"score": 1}')
    assert result.score == 1
    assert result.reasoning == ""


def test_validate_frontier_choice_accepts_label_id():
    assert validate_frontier_choice(1, [0, 1]) == 1


def test_validate_frontier_choice_rejects_out_of_range():
    with pytest.raises(ValueError, match="not in candidates"):
        validate_frontier_choice(4, [0, 1])


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2
