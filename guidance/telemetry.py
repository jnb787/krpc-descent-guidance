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
        ref_frame = self.vessel.orbit.body.reference_frame
        self.flight = self.conn.add_stream(self.vessel.flight, ref_frame)
        # TODO: set up reference frames and streams here

    def altitude(self) -> float:
        """Return altitude in m above terrian surface."""
        return self.flight.mean_altitude

    def vertical_speed(self) -> float:
        """Return vertical speed in m/s (negative = descending)."""
        return self.flight.vertical_speed

    def horizontal_speed(self) -> float:
        """Return horizontal speed in m/s (negative = right)."""
        return self.flight.horizontal_speed

    def position(self) -> tuple:
        """Return (x, y, z) position in meters."""
        return self.vessel.position

    def velocity(self) -> tuple:
        """Return (x, y, z) velocity in meters/sec."""
        return self.vessel.velocity

    def fuel_mass(self) -> float:
        """Return current propellant mass in kg."""
        return self.vessel.resources.propellant.amount
