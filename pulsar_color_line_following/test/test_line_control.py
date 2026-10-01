"""Unit tests for encoder distance calculations."""

import math

import pytest

from pulsar_color_line_following.line_control import (
    alignment_gate,
    calculate_wheel_distance,
)


def test_one_wheel_revolution_is_one_circumference():
    distance = calculate_wheel_distance(1100, 1100, 0.10, 1100)
    assert math.isclose(distance, 2.0 * math.pi * 0.10)


def test_negative_tick_direction_still_counts_distance():
    distance = calculate_wheel_distance(-550, -550, 0.10, 1100)
    assert math.isclose(distance, math.pi * 0.10)


def test_average_of_two_wheels_is_used():
    distance = calculate_wheel_distance(1100, 0, 0.10, 1100)
    assert math.isclose(distance, math.pi * 0.10)


def test_alignment_gate_enters_at_wide_threshold():
    assert alignment_gate(0.08, False, 0.08, 0.03)
    assert not alignment_gate(0.079, False, 0.08, 0.03)


def test_alignment_gate_stays_active_until_tight_threshold():
    assert alignment_gate(0.05, True, 0.08, 0.03)
    assert not alignment_gate(0.03, True, 0.08, 0.03)


def test_alignment_gate_rejects_invalid_hysteresis():
    with pytest.raises(ValueError):
        alignment_gate(0.1, False, 0.05, 0.05)
