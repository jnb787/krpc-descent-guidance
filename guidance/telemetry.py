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
        # TODO: set up reference frames and streams here

    def altitude(self) -> float:
        """Return surface altitude in meters."""
        raise NotImplementedError

    def vertical_speed(self) -> float:
        """Return vertical speed in m/s (negative = descending)."""
        raise NotImplementedError

    def fuel_mass(self) -> float:
        """Return current propellant mass in kg."""
        raise NotImplementedError
