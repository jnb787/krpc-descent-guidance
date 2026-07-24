"""
vehicle.py

Wraps vessel-specific properties needed for guidance calculations:
current mass, max thrust, specific impulse, throttle/attitude control.

Separating this from telemetry.py keeps a clean distinction:
    telemetry.py -> "where am I, how fast am I going"
    vehicle.py   -> "what can my vehicle actually do about it"
"""

from guidance.utils import clamp

class Vehicle:
    """Represents the controllable properties of the active vessel.

    TODO:
        - wrap conn.space_center.active_vessel
        - expose: mass (kg), available_thrust (N), max_deceleration (m/s^2)
        - expose control setters: set_throttle(value), point_retrograde(),
          point_at(direction)
        - remember thrust and mass change as fuel burns -- max_deceleration
          should be recomputed each guidance loop iteration, not cached
    """

    def __init__(self, conn, vessel):
        self.conn = conn
        self.vessel = vessel

    def current_mass(self) -> float:
        """Return current vessel mass in kg (changes as fuel burns)."""
        return self.vessel.mass

    def available_thrust(self) -> float:
        """Return max available thrust in Newtons at current throttle=1."""
        return self.vessel.max_thrust

    def max_deceleration(self) -> float:
        """Return max deceleration achievable right now (m/s^2).

        This is available_thrust() / current_mass(). It changes
        continuously through the burn as propellant mass decreases,
        which is exactly why the suicide-burn calculation has to be
        re-evaluated every guidance loop tick, not computed once.
        """
        return self.available_thrust() / self.current_mass()

    def set_throttle(self, value: float) -> None:
        """Set throttle, clamped to [0, 1]."""
        self.vessel.control.throttle = clamp(value, 0.0, 1.0)
