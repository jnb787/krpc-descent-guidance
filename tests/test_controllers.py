"""
test_controllers.py

Unit tests for the pure-math parts of controllers.py. These don't
need a live KSP connection -- that's the point. Test your PID math
and suicide-burn calculation in isolation with known inputs/outputs.

Run with:
    pytest tests/
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from guidance.controllers import PIDController, suicide_burn_altitude


def test_pid_proportional_only():
    """With only kp set, output should be proportional to error."""
    pid = PIDController(kp=2.0, ki=0.0, kd=0.0, setpoint=10.0)
    output = pid.update(measurement=8.0, dt=1.0)
    # error = setpoint - measurement = 10 - 8 = 2
    # output should be kp * error = 2.0 * 2 = 4.0
    assert output == 4.0


def test_pid_zero_error_gives_zero_output():
    """When measurement equals setpoint, output should be zero."""
    pid = PIDController(kp=1.0, ki=1.0, kd=1.0, setpoint=5.0)
    output = pid.update(measurement=5.0, dt=1.0)
    assert output == 0.0


def test_suicide_burn_altitude_simple_case():
    """Sanity check the suicide-burn distance formula with round numbers.

    If falling at 100 m/s, max deceleration is 20 m/s^2, and gravity
    is 10 m/s^2, net deceleration during the burn is 10 m/s^2.
    Using v^2 = v0^2 + 2*a*d  ->  d = v0^2 / (2*a) = 100^2 / 20 = 500 m.
    """
    altitude = suicide_burn_altitude(
        velocity=100.0, max_deceleration=20.0, gravity=10.0
    )
    assert abs(altitude - 500.0) < 0.01


def test_suicide_burn_altitude_zero_velocity():
    """If you're not moving, you don't need any altitude margin."""
    altitude = suicide_burn_altitude(
        velocity=0.0, max_deceleration=20.0, gravity=10.0
    )
    assert altitude == 0.0
