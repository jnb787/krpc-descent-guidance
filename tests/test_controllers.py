"""
test_controllers.py

Unit tests for the pure-math parts of controllers.py. These don't
need a live KSP connection -- that's the point. Test your PID math
and suicide-burn calculation in isolation with known inputs/outputs.

Run with:
    pytest tests/
"""

import pytest
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


def test_pid_integral_only():
    """With only ki set, output should be integral of error over time."""
    pid = PIDController(kp=0.0, ki=2.0, kd=0.0, setpoint=10.0)
    # First update: error = 2, integral accumulates 2 * 1.0 = 2
    output1 = pid.update(measurement=8.0, dt=1.0)
    # output = ki * integral = 2.0 * 2 = 4.0
    assert output1 == 4.0

    # Second update: error = 2 again, integral accumulates another 2 * 1.0 = 4 total
    output2 = pid.update(measurement=8.0, dt=1.0)
    # output = ki * integral = 2.0 * 4 = 8.0
    assert output2 == 8.0


def test_pid_integral_with_zero_error():
    """Integral should not accumulate when error is zero."""
    pid = PIDController(kp=0.0, ki=1.0, kd=0.0, setpoint=5.0)
    output1 = pid.update(measurement=5.0, dt=1.0)
    assert output1 == 0.0

    output2 = pid.update(measurement=5.0, dt=1.0)
    assert output2 == 0.0


def test_pid_derivative_only():
    """With only kd set, output should be proportional to rate of error change."""
    pid = PIDController(kp=0.0, ki=0.0, kd=2.0, setpoint=10.0)
    # First update: no previous error, derivative should be zero
    output1 = pid.update(measurement=8.0, dt=1.0)
    assert output1 == 0.0

    # Second update: error changed from 2 to 1 (measurement 8 -> 9)
    # d_error = (1 - 2) / 1.0 = -1
    # output = kd * d_error = 2.0 * -1 = -2.0
    output2 = pid.update(measurement=9.0, dt=1.0)
    assert output2 == -2.0


def test_pid_derivative_constant_error():
    """Derivative should be zero when error is constant."""
    pid = PIDController(kp=0.0, ki=0.0, kd=1.0, setpoint=10.0)
    output1 = pid.update(measurement=5.0, dt=1.0)
    assert output1 == 0.0

    # Error stays at 5, so derivative = 0
    output2 = pid.update(measurement=5.0, dt=1.0)
    assert output2 == 0.0


def test_pid_full_pid():
    """Test P, I, and D terms working together."""
    pid = PIDController(kp=1.0, ki=1.0, kd=1.0, setpoint=10.0)

    # First update: measurement=8, error=2
    # P = 1.0 * 2 = 2.0
    # I = 1.0 * (2 * 1.0) = 2.0
    # D = 0 (no previous error)
    # Total = 2.0 + 2.0 + 0 = 4.0
    output1 = pid.update(measurement=8.0, dt=1.0)
    assert output1 == 4.0

    # Second update: measurement=7, error=3
    # P = 1.0 * 3 = 3.0
    # I = 1.0 * (2 + 3*1.0) = 5.0
    # D = 1.0 * (3 - 2) / 1.0 = 1.0
    # Total = 3.0 + 5.0 + 1.0 = 9.0
    output2 = pid.update(measurement=7.0, dt=1.0)
    assert output2 == 9.0


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

def test_pid_integral_accumulates_over_ticks():
    """Integral term should keep growing while error persists, and dt should scale it."""
    pid = PIDController(kp=0.0, ki=2.0, kd=0.0, setpoint=10.0)
    # Constant error of 2, but dt=0.5 so each tick adds 2 * 0.5 = 1.0 to the integral.
    # Tick 1: integral = 1.0  -> output = 2.0 * 1.0 = 2.0
    assert pid.update(measurement=8.0, dt=0.5) == pytest.approx(2.0)
    # Tick 2: integral = 2.0  -> output = 4.0
    assert pid.update(measurement=8.0, dt=0.5) == pytest.approx(4.0)
    # Tick 3: integral = 3.0  -> output = 6.0
    assert pid.update(measurement=8.0, dt=0.5) == pytest.approx(6.0)


def test_pid_derivative_tracks_changing_error():
    """Derivative should reflect how fast the error is changing, tick to tick."""
    pid = PIDController(kp=0.0, ki=0.0, kd=2.0, setpoint=10.0)
    # Measurements 8 -> 9 -> 9.5 give errors 2 -> 1 -> 0.5 (error shrinking, but slower each time).
    # Tick 1: no prev error, derivative = 0
    assert pid.update(measurement=8.0, dt=1.0) == pytest.approx(0.0)
    # Tick 2: d_error = (1 - 2)/1 = -1  -> output = 2.0 * -1 = -2.0
    assert pid.update(measurement=9.0, dt=1.0) == pytest.approx(-2.0)
    # Tick 3: d_error = (0.5 - 1)/1 = -0.5 -> output = 2.0 * -0.5 = -1.0
    # (smaller magnitude: error is still shrinking, but more slowly)
    assert pid.update(measurement=9.5, dt=1.0) == pytest.approx(-1.0)


def test_pid_full_multi_tick():
    """P, I, and D together across three ticks with a changing measurement."""
    pid = PIDController(kp=1.0, ki=1.0, kd=1.0, setpoint=10.0)
    # Tick 1: meas=8, error=2. P=2, integral=2 -> I=2, D=0 (first tick). Total=4.0
    assert pid.update(measurement=8.0, dt=1.0) == pytest.approx(4.0)
    # Tick 2: meas=7, error=3. P=3, integral=5 -> I=5, D=(3-2)/1=1. Total=9.0
    assert pid.update(measurement=7.0, dt=1.0) == pytest.approx(9.0)
    # Tick 3: meas=7, error=3 (unchanged). P=3, integral=8 -> I=8, D=(3-3)/1=0. Total=11.0
    assert pid.update(measurement=7.0, dt=1.0) == pytest.approx(11.0)