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
    assert predicted_impact_offset(100.0, 200.0, -300.0, 0.0, 0.0, 50.0) == (100.0, 200.0)


def test_predicted_impact_offset_carries_each_axis_independently():
    """Each axis advances by its own velocity times the fall time."""
    north, east = predicted_impact_offset(100.0, 200.0, -300.0, 3.0, -4.0, 10.0)
    assert abs(north - 130.0) < 1e-9
    assert abs(east - 160.0) < 1e-9


def test_predicted_impact_offset_scales_with_fall_time():
    """Twice the fall time carries twice as far from the same start."""
    _, near = predicted_impact_offset(0.0, 0.0, -300.0, 0.0, 5.0, 10.0)
    _, far = predicted_impact_offset(0.0, 0.0, -300.0, 0.0, 5.0, 20.0)
    assert abs(far - 2.0 * near) < 1e-9


def test_predicted_impact_offset_sign_points_downrange():
    """Drifting east predicts an impact further east, not nearer."""
    _, east = predicted_impact_offset(0.0, 466.1, -300.0, 0.0, 17.78, 48.6)
    assert east > 466.1


# --- Coriolis deflection -----------------------------------------------------
#
# A vessel falling on a prograde-rotating body drifts east. The deflection is
# 2*omega*v_down, and v_down itself grows as g*t during the fall, so the
# displacement has both a T^2 and a T^3 term. Both default to off (omega and
# gravity default to 0.0), which is what the four tests above rely on.

KERBIN_OMEGA = 2.909e-4  # rad/s


def test_predicted_impact_offset_no_rotation_is_a_straight_line():
    """omega=0 must reproduce the pre-Coriolis behaviour exactly.

    The regression guard for every caller that does not opt in: no rotation,
    no deflection, however fast the vessel is falling.
    """
    _, east = predicted_impact_offset(0.0, 100.0, -744.0, 0.0, 5.0, 41.8,
                                      omega=0.0, gravity=KERBIN_GRAVITY)
    assert abs(east - (100.0 + 5.0 * 41.8)) < 1e-9


def test_predicted_impact_offset_gravity_isolates_the_cubic_term():
    """gravity=0 leaves only the T^2 term, so the two halves can be told apart.

    Without this a sign error in one term could hide inside the sum of both.
    """
    t = 41.8
    _, quadratic_only = predicted_impact_offset(0.0, 0.0, -744.0, 0.0, 0.0, t,
                                                omega=KERBIN_OMEGA, gravity=0.0)
    expected = 2 * KERBIN_OMEGA * 0.5 * 744.0 * t ** 2
    assert abs(quadratic_only - expected) < 1e-9


def test_predicted_impact_offset_descending_deflects_east():
    """A falling vessel lands east of where a straight line would put it."""
    _, drifting = predicted_impact_offset(0.0, 0.0, -744.0, 0.0, 0.0, 41.8,
                                          omega=KERBIN_OMEGA, gravity=KERBIN_GRAVITY)
    assert drifting > 0.0


def test_predicted_impact_offset_ascending_deflects_west():
    """The sign of the deflection follows the sign of v_up.

    Pins v0 = -v_up. A vessel still climbing is thrown the other way, and
    getting this backwards would send CORRECT's burn 180 degrees wrong.
    """
    _, climbing = predicted_impact_offset(0.0, 0.0, 744.0, 0.0, 0.0, 41.8,
                                          omega=KERBIN_OMEGA, gravity=KERBIN_GRAVITY)
    assert climbing < 0.0


def test_predicted_impact_offset_matches_numerical_integration():
    """Step the acceleration forward by hand and compare against the closed form.

    Stronger than any spot value: it independently integrates
    a_east(t) = 2*omega*(v0 + g*t) twice, so a dropped factor of 1/2, a T^2
    written where T^3 was meant, or a 1/6 that should be 1/3 all show up.
    Uses the flight state at CORRECT's exit on run 180024.
    """
    x0, v_east0, v_up, t_fall = 411.1, -8.72, -744.0, 41.8
    _, closed_form = predicted_impact_offset(0.0, x0, v_up, 0.0, v_east0, t_fall,
                                             omega=KERBIN_OMEGA, gravity=KERBIN_GRAVITY)

    steps = 200_000
    dt = t_fall / steps
    v0 = -v_up
    v_east, east = v_east0, x0
    for i in range(steps):
        v_east += 2 * KERBIN_OMEGA * (v0 + KERBIN_GRAVITY * i * dt) * dt
        east += v_east * dt

    assert abs(closed_form - east) < 0.01


def test_predicted_impact_offset_cubic_term_dominates_long_falls():
    """At high altitude the T^3 term is worth several times the T^2 term.

    This is the property the change was made for: correcting early was
    useless because the predictor was blind to most of the coming drift.
    At t_fall = 112 s the omitted term was over 3x the one that was there.
    """
    t = 112.4
    _, quadratic_only = predicted_impact_offset(0.0, 0.0, -110.0, 0.0, 0.0, t,
                                                omega=KERBIN_OMEGA, gravity=0.0)
    _, both_terms = predicted_impact_offset(0.0, 0.0, -110.0, 0.0, 0.0, t,
                                            omega=KERBIN_OMEGA, gravity=KERBIN_GRAVITY)
    cubic = both_terms - quadratic_only
    assert cubic > 3.0 * quadratic_only


def test_predicted_impact_offset_north_is_unaffected_by_rotation():
    """A radial fall deflects only east -- the north axis stays a straight line."""
    north, _ = predicted_impact_offset(50.0, 0.0, -744.0, 2.0, 0.0, 41.8,
                                       omega=KERBIN_OMEGA, gravity=KERBIN_GRAVITY)
    assert abs(north - (50.0 + 2.0 * 41.8)) < 1e-9
