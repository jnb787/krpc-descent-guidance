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
    """Wraps a vessel and reference frame to provide clean telemetry reads.

    TODO:
        - store conn, vessel, and the reference frame(s) you need
          (surface frame for altitude/vertical speed, orbital frame
          for orbital velocity)
        - add methods like: altitude(), vertical_speed(), horizontal_speed(),
          fuel_mass(), position(), velocity()
        - consider using conn.add_stream(...) for values you read every
          loop iteration, it's much more efficient than polling
    """

    def __init__(self, conn, vessel):
        self.conn = conn
        self.vessel = vessel
        self.ref_frame = self.vessel.orbit.body.reference_frame
        self.flight = self.conn.add_stream(self.vessel.flight, self.ref_frame)
        self.fuel_stream_lf = self.conn.add_stream(self.vessel.resources.amount, 'LiquidFuel')
        self.fuel_stream_ox = self.conn.add_stream(self.vessel.resources.amount, 'Oxidizer')
        self.rot_stream = self.conn.add_stream(self.vessel.rotation, self.ref_frame)
    def altitude(self) -> float:
        """Return altitude in m above terrain surface."""
        return self.flight().surface_altitude

    def vertical_speed(self) -> float:
        """Return vertical speed in m/s (negative = descending)."""
        return self.flight().vertical_speed

    def horizontal_speed(self) -> float:
        """Return horizontal speed in m/s (negative = right)."""
        return self.flight().horizontal_speed

    def position(self) -> tuple:
        """Return (x, y, z) position in meters."""
        return self.vessel.position(self.ref_frame)

    def velocity(self) -> tuple:
        """Return (x, y, z) velocity in meters/sec."""
        return self.vessel.velocity(self.ref_frame)

    def fuel_mass(self) -> float:
        """Return current propellant mass in kg."""
        mass_of_fuel = 5*(self.fuel_stream_lf() + self.fuel_stream_ox())
        return mass_of_fuel

    def total_mass(self) -> float:
        """Return current total mass in kg."""
        return self.vessel.mass

    def orientation(self) -> tuple:
        """Return (x, y, z, w) orientation quaternion."""
        return self.rot_stream()