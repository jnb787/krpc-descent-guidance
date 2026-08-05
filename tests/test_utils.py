"""
test_utils.py

Unit tests for utils.py's pure math helper functions.
"""

import sys
import os
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from guidance.utils import clamp, haversine_distance, surface_offset

KERBIN_RADIUS = 600_000  # meters


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


def test_surface_offset_on_target():
    """A position sitting on the target has no offset."""
    north, east = surface_offset(0.0, 0.0, 0.0, 0.0, KERBIN_RADIUS)
    assert abs(north) < 0.01
    assert abs(east) < 0.01


def test_surface_offset_north_is_positive():
    """One degree north of the target is +R*pi/180 meters north."""
    north, east = surface_offset(1.0, 0.0, 0.0, 0.0, KERBIN_RADIUS)
    assert abs(north - KERBIN_RADIUS * math.pi / 180) < 1.0
    assert abs(east) < 0.01


def test_surface_offset_south_is_negative():
    """Sign flips when the position is south of the target."""
    north, _ = surface_offset(-1.0, 0.0, 0.0, 0.0, KERBIN_RADIUS)
    assert north < 0.0


def test_surface_offset_east_shrinks_with_latitude():
    """A degree of longitude covers less ground away from the equator."""
    _, at_equator = surface_offset(0.0, 1.0, 0.0, 0.0, KERBIN_RADIUS)
    _, at_sixty = surface_offset(60.0, 1.0, 60.0, 0.0, KERBIN_RADIUS)
    # cos(60) == 0.5, so the same angular difference is half the distance.
    assert abs(at_sixty - at_equator * 0.5) < 1.0


def test_surface_offset_wraps_antimeridian():
    """A target just across +/-180 is a short hop away, not a lap round.

    The target sits 2 degrees east, so the position is 2 degrees west of
    it -- a negative east offset, not the 358 degrees the raw subtraction
    would give.
    """
    _, east = surface_offset(0.0, 179.0, 0.0, -179.0, KERBIN_RADIUS)
    assert abs(east + 2 * KERBIN_RADIUS * math.pi / 180) < 1.0


def test_surface_offset_magnitude_matches_haversine():
    """For small offsets the tangent-plane approximation tracks great-circle."""
    lat, lon = -0.05, -74.6      # a few km off the KSC pad
    north, east = surface_offset(lat, lon, -0.0972, -74.5577, KERBIN_RADIUS)
    approx = math.hypot(north, east)
    exact = haversine_distance(lat, lon, -0.0972, -74.5577, KERBIN_RADIUS)
    assert abs(approx - exact) < 1.0
