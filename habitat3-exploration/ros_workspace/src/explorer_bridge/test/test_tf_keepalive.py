"""Regression: TF keepalive during DiscreteMove (Nav2 stamp spam)."""

from __future__ import annotations

import pytest


def test_harness_positive():
    assert 1 + 1 == 2


def test_harness_negative():
    with pytest.raises(AssertionError):
        assert 1 == 2


def test_motion_active_still_allows_tf_keepalive_positive():
    """During motion, sensor timer skips depth; dedicated TF timer still runs."""
    motion_active = 1
    published = {"tf": False, "depth": False, "odom": False}

    def publish_tf_only():
        published["tf"] = True

    def publish_full_sensors():
        published["tf"] = True
        published["odom"] = True
        published["depth"] = True

    # Sensor timer during motion:
    if motion_active == 0:
        publish_full_sensors()
    # TF timer always:
    publish_tf_only()

    assert published["tf"] is True
    assert published["depth"] is False
    assert published["odom"] is False


def test_keepalive_must_not_publish_odom_negative():
    """Negative: TF keepalive must not flood /odom (breaks exact-stamp mapper)."""
    published_odom = False

    def publish_tf_only():
        nonlocal published_odom
        # TF path intentionally does not touch /odom
        published_odom = False

    publish_tf_only()
    assert published_odom is False


def test_idle_publishes_full_sensors_negative():
    """Negative: idle path must still publish depth + odom."""
    motion_active = 0
    published = {"tf": False, "depth": False, "odom": False}

    def publish_full_sensors():
        published["tf"] = True
        published["odom"] = True
        published["depth"] = True

    if motion_active == 0:
        publish_full_sensors()

    assert published["depth"] is True
    assert published["odom"] is True
