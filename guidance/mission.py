"""
mission.py

Top-level flight sequence: the state machine that ties telemetry,
vehicle, and controllers together into a full autonomous landing.

Phases (build and test these one at a time, in this order):
    1. DEORBIT   - execute a burn to drop the orbit's periapsis toward
                   the target landing site
    2. COAST     - wait for the vessel to descend, doing nothing but
                   monitoring altitude, until suicide_burn_altitude()
                   says it's time to start braking
    3. DESCENT   - the powered-descent / suicide burn itself: throttle
                   controlled by a PID loop targeting zero vertical
                   speed at zero altitude, attitude corrected to null
                   out horizontal drift toward the target
    4. LANDED    - cut throttle, log final results (landing error,
                   fuel remaining, max G, time to land)

TODO:
    - define a Phase enum or simple string states
    - implement run_mission(conn, target) as the main loop that
      transitions between phases and calls into controllers.py
    - log telemetry to CSV every loop iteration (see data/ folder)
"""

from enum import Enum, auto


class Phase(Enum):
    DEORBIT = auto()
    COAST = auto()
    DESCENT = auto()
    LANDED = auto()


def run_mission(conn, target_latitude: float, target_longitude: float) -> dict:
    """Run the full autonomous landing sequence.

    Args:
        conn: an active kRPC connection (see telemetry.connect())
        target_latitude: landing target latitude, degrees
        target_longitude: landing target longitude, degrees

    Returns:
        A results dict with keys like: landing_error_m, fuel_used_kg,
        max_g, time_to_land_s -- this is what you'll aggregate across
        20+ runs for your SMART goal's measurable success criteria.
    """
    raise NotImplementedError
