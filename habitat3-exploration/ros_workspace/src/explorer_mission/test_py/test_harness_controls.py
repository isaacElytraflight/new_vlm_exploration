"""Meta-tests: verify pytest harness is working (positive/negative controls)."""


def test_positive_control_passes():
    assert 1 + 1 == 2


def test_negative_control_fails_as_expected():
    import pytest

    with pytest.raises(AssertionError):
        assert False, "intentional failure for harness validation"
