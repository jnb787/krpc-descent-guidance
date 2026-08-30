"""
test_utils.py

Unit tests for utils.py's pure math helper functions.
"""

import sys
import os
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from guidance.utils import (clamp, haversine_distance, surface_offset,
                           ballistic_fall_time, predicted_impact_offset)

KERBIN_RADIUS = 600_000  # meters
KERBIN_GRAVITY = 9.81    # m/s^2 at the surface


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


def test_ballistic_fall_time_free_fall():
    """Dropped from rest, fall time is the closed form sqrt(2h/g)."""
    t = ballistic_fall_time(1000.0, 0.0, KERBIN_GRAVITY)
    assert abs(t - math.sqrt(2 * 1000.0 / KERBIN_GRAVITY)) < 1e-9


def test_ballistic_fall_time_satisfies_kinematics():
    """Substituting t back into h = v*t + g*t^2/2 must recover the height.

    Stronger than a spot value: it checks the quadratic was solved for the
    right root, using numbers from a real flight (45 km, -692 m/s).
    """
    height, speed = 45000.0, -692.2
    t = ballistic_fall_time(height, speed, KERBIN_GRAVITY)
    recovered = abs(speed) * t + 0.5 * KERBIN_GRAVITY * t * t
    assert abs(recovered - height) < 1e-6


def test_ballistic_fall_time_at_ground_is_zero():
    """No height left means no time left, however fast you are going."""
    assert ballistic_fall_time(0.0, -500.0, KERBIN_GRAVITY) == 0.0


def test_ballistic_fall_time_below_ground_is_clamped():
    """A negative height must not put a negative under the square root."""
    assert ballistic_fall_time(-50.0, -100.0, KERBIN_GRAVITY) == 0.0


def test_ballistic_fall_time_shorter_when_already_falling():
    """Starting with downward speed gets you there sooner than from rest."""
    moving = ballistic_fall_time(1000.0, -100.0, KERBIN_GRAVITY)
    resting = ballistic_fall_time(1000.0, 0.0, KERBIN_GRAVITY)
    assert moving < resting


def test_ballistic_fall_time_ignores_sign_of_vertical_speed():
    """abs() means an ascending vessel is treated as if it were descending.

    Pinning the limitation rather than endorsing it. CORRECT only ever runs
    on a descending vessel today, but a profile that entered it while still
    climbing would get a badly short fall time and over-correct.
    """
    assert (ballistic_fall_time(1000.0, -100.0, KERBIN_GRAVITY)
            == ballistic_fall_time(1000.0, 100.0, KERBIN_GRAVITY))


def test_predicted_impact_offset_without_velocity():
    """With no horizontal motion, you land where you already are."""
    assert predicted_impact_offset(100.0, 200.0, 0.0, 0.0, 50.0) == (100.0, 200.0)


def test_predicted_impact_offset_carries_each_axis_independently():
    """Each axis advances by its own velocity times the fall time."""
    north, east = predicted_impact_offset(100.0, 200.0, 3.0, -4.0, 10.0)
    assert abs(north - 130.0) < 1e-9
    assert abs(east - 160.0) < 1e-9


def test_predicted_impact_offset_scales_with_fall_time():
    """Twice the fall time carries twice as far from the same start."""
    _, near = predicted_impact_offset(0.0, 0.0, 0.0, 5.0, 10.0)
    _, far = predicted_impact_offset(0.0, 0.0, 0.0, 5.0, 20.0)
    assert abs(far - 2.0 * near) < 1e-9


def test_predicted_impact_offset_sign_points_downrange():
    """Drifting east predicts an impact further east, not nearer."""
    _, east = predicted_impact_offset(0.0, 466.1, 0.0, 17.78, 48.6)
    assert east > 466.1
