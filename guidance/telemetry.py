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
        self.charge_stream = self.conn.add_stream(self.vessel.resources.amount, 'ElectricCharge')

        # A second Flight stream on the *surface* frame (x=up, y=north, z=east).
        # Attitude readings (pitch, heading, AoA, sideslip) are only meaningful
        # here -- in the body frame above, "the horizon" is the equatorial
        # plane, so a vertical vessel near the equator reads a pitch of ~0
        # rather than ~90.
        self.surface_ref_frame = self.vessel.surface_reference_frame
        self.surface_flight = self.conn.add_stream(self.vessel.flight, self.surface_ref_frame)

        # ...but NOT for velocity. surface_reference_frame has its origin at
        # the vessel's own centre of mass and travels with it, so the vessel's
        # velocity measured in it is identically zero. Take the axes from it
        # and the origin from the body, which gives surface-relative velocity
        # resolved into (up, north, east).
        self.ne_ref_frame = self.conn.space_center.ReferenceFrame.create_hybrid(
            position=self.vessel.orbit.body.reference_frame,
            rotation=self.vessel.surface_reference_frame,
            velocity=self.vessel.orbit.body.reference_frame)
        self.ne_flight = self.conn.add_stream(self.vessel.flight, self.ne_ref_frame)

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

    def mean_altitude(self) -> float:
        """Return altitude in m above sea level.

        Unlike altitude(), this is referenced to a fixed datum rather than
        to whatever happens to be underneath the vessel, so it does not step
        when you pass over a structure (the KSC pad deck reads several
        metres below the terrain datum) or drift as terrain height changes
        during a long descent. Guidance should measure its remaining
        distance against the landing site's own sea-level elevation.
        """
        return self.flight().mean_altitude

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
        return self.surface_flight().pitch

    def heading(self) -> float:
        """Return compass heading of the vessel's facing, degrees (0 = north).

        Paired with pitch(), this is what the vessel actually did -- compare
        against the commanded north/east to tell a steering bug apart from
        the autopilot being unable to hold the attitude it was given.
        """
        return self.surface_flight().heading

    def velocity_ne(self) -> tuple:
        """Return (north, east) components of surface velocity, m/s.

        horizontal_speed() gives only the magnitude; these carry the
        direction, which is what a velocity-nulling controller needs to
        know which way to lean.

        Sanity check: hypot(north, east) should equal horizontal_speed().
        If these read zero while horizontal_speed() does not, the frame
        is travelling with the vessel again.
        """
        _up, north, east = self.ne_flight().velocity
        return (north, east)

    def angle_of_attack(self) -> float:
        """Return pitch angle between the vessel's facing and its velocity, degrees.

        This is the achieved AoA. Compared against the commanded tilt it
        says whether the vehicle is actually holding the attitude asked of
        it, or weathercocking back to zero because the aerodynamic
        restoring moment beats the available control torque.
        """
        return self.surface_flight().angle_of_attack

    def sideslip_angle(self) -> float:
        """Return yaw angle between the vessel's facing and its velocity, degrees.

        The lateral counterpart to angle_of_attack() -- together they give
        the full achieved attitude relative to the airflow.
        """
        return self.surface_flight().sideslip_angle

    def dynamic_pressure(self) -> float:
        """Return dynamic pressure in Pascals (q = 0.5 * rho * v^2)."""
        return self.flight().dynamic_pressure

    def drag(self) -> tuple:
        """Return (x, y, z) aerodynamic drag force in Newtons."""
        return self.flight().drag

    def lift(self) -> tuple:
        """Return (x, y, z) aerodynamic lift force in Newtons."""
        return self.flight().lift

    def electric_charge(self) -> float:
        """Return remaining electric charge."""
        return self.charge_stream()