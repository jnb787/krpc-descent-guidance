"""
controllers.py

This is the heart of the project: the actual guidance and control
math. Two things live here:

1. PIDController - a general-purpose feedback controller, reusable
   for throttle control, attitude correction, horizontal position
   correction, etc.

2. suicide_burn_altitude() - the physics calculation for "how high
   do I need to be when I start my braking burn so that I reach
   zero velocity exactly at the surface." This is the core aerospace
   guidance concept for this project -- work through the derivation
   yourself (kinematics: v^2 = v0^2 + 2*a*d) before filling this in.
"""


class PIDController:
    """A standard PID (Proportional-Integral-Derivative) controller.

    Usage:
        pid = PIDController(kp=1.0, ki=0.0, kd=0.5, setpoint=0.0)
        output = pid.update(measurement, dt)

    TODO:
        - implement update(): compute error = setpoint - measurement,
          accumulate integral term, compute derivative from the
          change in error over dt, return kp*error + ki*integral + kd*derivative
        - add integral windup protection (clamp the accumulated integral)
        - consider output clamping (e.g. throttle must stay in [0, 1])
    """

    def __init__(self, kp: float, ki: float, kd: float, setpoint: float = 0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self._integral = 0.0
        self._prev_error = None

    def update(self, measurement: float, dt: float) -> float:
        """Compute the control output for one timestep.

        Args:
            measurement: current measured value (e.g. vertical speed)
            dt: time elapsed since the last update, in seconds

        Returns:
            The control signal (e.g. throttle delta).
        """
        raise NotImplementedError

    def reset(self) -> None:
        """Clear accumulated integral/derivative state."""
        self._integral = 0.0
        self._prev_error = None


def suicide_burn_altitude(
    velocity: float, max_deceleration: float, gravity: float
) -> float:
    """Compute the altitude at which to begin the braking burn.

    This answers: "given my current downward velocity and my vehicle's
    max deceleration capability, how high above the ground do I need
    to start burning full throttle to hit exactly zero velocity at
    the surface?"

    Derivation hint (fill in yourself, then implement):
        Use v^2 = v0^2 + 2*a*d, solving for d (the stopping distance),
        where the net deceleration during the burn is
        (max_deceleration - gravity), since gravity is still pulling
        you down while your engine pushes up.

    Args:
        velocity: current downward speed, m/s (positive = descending)
        max_deceleration: max thrust-based deceleration, m/s^2
        gravity: local gravitational acceleration, m/s^2 (e.g. 9.81
            at Kerbin sea level -- but note it's actually ~9.81 only
            near the surface; use the body's surface gravity constant)

    Returns:
        Altitude (meters) above the surface at which to begin the burn.
    """
    raise NotImplementedError
