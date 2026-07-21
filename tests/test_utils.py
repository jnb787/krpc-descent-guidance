"""
test_utils.py

Unit tests for utils.py's pure math helper functions.
"""

import sys
import os
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from guidance.utils import clamp, haversine_distance


def test_clamp_within_range():
    assert clamp(0.5, 0.0, 1.0) == 0.5


def test_clamp_below_min():
    assert clamp(-5, 0.0, 1.0) == 0.0


def test_clamp_above_max():
    assert clamp(5, 0.0, 1.0) == 1.0

def test_haversine_distance():
    """Equator to pole along a meridian is a quarter of the circumference."""
    kerbin_radius = 600_000  # meters
    d = haversine_distance(0.0, 0.0, 90.0, 0.0, kerbin_radius)
    expected = 2 * kerbin_radius * math.pi / 4
    assert abs(d - expected) < 1.0


def test_haversine_zero_distance():
    """Distance between a point and itself should be zero."""
    kerbin_radius = 600_000  # meters
    d = haversine_distance(0.0, 0.0, 0.0, 0.0, kerbin_radius)
    assert abs(d - 0.0) < 0.01
