"""
vehicle.py

Wraps vessel-specific properties needed for guidance calculations:
current mass, max thrust, specific impulse, throttle/attitude control.

Separating this from telemetry.py keeps a clean distinction:
    telemetry.py -> "where am I, how fast am I going"
    vehicle.py   -> "what can my vehicle actually do about it"
"""

import time

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
        self.ap = vessel.auto_pilot
        
    def current_mass(self) -> float:
        """Return current vessel mass in kg (changes as fuel burns)."""
        return self.vessel.mass

    def available_thrust(self) -> float:
        """Return max available thrust in Newtons at current throttle=1."""
        return self.vessel.available_thrust

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

    def throttle(self) -> float:
        """Return the current throttle setting, [0, 1]."""
        return self.vessel.control.throttle

    def engage(self) -> None:
        """Take control with the kRPC autopilot (turn off stock SAS so they don't fight)."""
        self.vessel.control.sas = False
        self.ap.engage()

    def point(self, up: float, north: float, east: float) -> None:
        """Point along a direction in the surface frame: x=up, y=north, z=east.
        For descent: up large, north/east small (the horizontal PID sets those)."""
        self.ap.target_direction = (up, north, east)

    def point_retrograde(self) -> None:
        """Coast phase: cheap retrograde hold via stock SAS."""
        self.ap.disengage()               # don't let the autopilot fight SAS
        self.vessel.control.sas = True
        # SAS needs a physics tick to initialise before it will accept a mode
        # change -- setting sas_mode in the same tick is silently dropped and
        # you're left on plain stability assist.
        time.sleep(0.2)
        # SASMode.retrograde follows the navball's speed mode -- pin it to
        # surface, or we'd hold *orbital* retrograde on the way down.
        self.vessel.control.speed_mode = self.conn.space_center.SpeedMode.surface
        self.vessel.control.sas_mode = self.conn.space_center.SASMode.retrograde

        if self.vessel.control.sas_mode != self.conn.space_center.SASMode.retrograde:
            print("WARNING: SAS would not accept retrograde mode -- check that the "
                  "command pod/probe core supports it (needs SAS level 1+)")

    def deploy_legs(self) -> None:
        """Deploy legs for landing."""
        self.vessel.control.legs = True

    def apply_brakes(self) -> None:
        """Deploy airbrakes for landing."""
        self.vessel.control.brakes = True

    def enable_rcs(self) -> None:
        """Turn on RCS for extra attitude authority.

        Useful during descent, where the engine gimbal alone can be slow to
        settle the vessel. Costs monopropellant, so it's worth turning back
        off once you're down.
        """
        self.vessel.control.rcs = True

    def disable_rcs(self) -> None:
        """Turn off RCS."""
        self.vessel.control.rcs = False

    def legs_status(self) -> bool:
        """Return True if legs are deployed."""
        return self.vessel.control.legs

    def brakes_status(self) -> bool:
        """Return True if airbrakes are deployed."""
        return self.vessel.control.brakes

    def rcs_status(self) -> bool:
        """Return True if RCS is enabled."""
        return self.vessel.control.rcs

    def monopropellant(self) -> float:
        """Return remaining monopropellant, in units (0.0 if the craft has none)."""
        return self.vessel.resources.amount("MonoPropellant")
