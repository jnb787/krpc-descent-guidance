"""
telemetry.py

Connects to the kRPC server and exposes live vessel telemetry:
altitude, velocity (surface + orbital frames), fuel mass, orientation.

This module should be the ONLY place in the project that talks
directly to kRPC's raw API for reading state. Everything else
(controllers, mission logic) should consume clean Python data
from here, not touch `conn.space_center...` directly. That keeps
your guidance math testable without a live KSP connection.
"""

import krpc


def connect(name: str = "Descent Guidance"):
    """Open a connection to the kRPC server running in KSP.

    Returns:
        krpc.client.Client: an active connection object.
    """
    conn = krpc.connect(name=name)
    return conn


class Telemetry:
    """Wraps a vessel and reference frame to provide clean telemetry reads."""

    def __init__(self, conn, vessel):
        self.conn = conn
        self.vessel = vessel
        self.ref_frame = self.vessel.orbit.body.reference_frame
        self.flight = self.conn.add_stream(self.vessel.flight, self.ref_frame)
        self.fuel_stream_lf = self.conn.add_stream(self.vessel.resources.amount, 'LiquidFuel')
        self.fuel_stream_ox = self.conn.add_stream(self.vessel.resources.amount, 'Oxidizer')
        self.rot_stream = self.conn.add_stream(self.vessel.rotation, self.ref_frame)
        self.ut_stream = self.conn.add_stream(getattr, self.conn.space_center, 'ut')

    def ut(self) -> float:
        """Return universal (in-game) time in seconds.

        Unlike time.time(), this advances with the game clock, so it stays
        correct through time warp -- use it for anything that has to line
        up with the physics (control loop dt, phase durations).
        """
        return self.ut_stream()

    def altitude(self) -> float:
        """Return altitude in m above terrain surface."""
        return self.flight().surface_altitude

    def effective_altitude(self) -> float:
        """Return altitude in m factoring CoM height into the calculation."""
        return self.flight().surface_altitude - 8.75

    def vertical_speed(self) -> float:
        """Return vertical speed in m/s (negative = descending)."""
        return self.flight().vertical_speed

    def horizontal_speed(self) -> float:
        """Return horizontal speed in m/s."""
        return self.flight().horizontal_speed

    def fuel_mass(self) -> float:
        """Return current propellant mass in kg."""
        mass_of_fuel = 5*(self.fuel_stream_lf() + self.fuel_stream_ox())
        return mass_of_fuel

    def orientation(self) -> tuple:
        """Return (x, y, z, w) orientation quaternion."""
        return self.rot_stream()

    def is_landed(self) -> bool:
        """True once KSP considers the vessel landed or splashed down."""
        sit = self.conn.space_center.VesselSituation
        return self.vessel.situation in (sit.landed, sit.splashed)

    def g_force(self) -> float:
        """Return current g-force experienced by the vessel."""
        return self.flight().g_force

    def latitude(self) -> float:
        """Return current latitude in degrees."""
        return self.flight().latitude

    def longitude(self) -> float:
        """Return current longitude in degrees."""
        return self.flight().longitude

    def pitch(self) -> float:
        """Return pitch of the vessel's facing above the horizon, degrees.

        90 is straight up, so a commanded tilt of T degrees off vertical
        should settle at a pitch of (90 - T) if the autopilot is tracking.
        """
        return self.flight().pitch

    def heading(self) -> float:
        """Return compass heading of the vessel's facing, degrees (0 = north).

        Paired with pitch(), this is what the vessel actually did -- compare
        against the commanded north/east to tell a steering bug apart from
        the autopilot being overpowered by aerodynamic forces.
        """
        return self.flight().heading

    def dynamic_pressure(self) -> float:
        """Return dynamic pressure in Pascals (q = 0.5 * rho * v^2).

        Aerodynamic control authority scales with this, unlike thrust
        authority -- it is ~0 above 70 km and large low and fast.
        """
        return self.flight().dynamic_pressure

    def drag(self) -> tuple:
        """Return (x, y, z) aerodynamic drag force in Newtons."""
        return self.flight().drag