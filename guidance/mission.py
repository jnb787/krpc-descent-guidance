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

"""
import time
import os

from guidance import telemetry, vehicle
from guidance.flight_log import FlightLogger, write_summary
from guidance.controllers import suicide_burn_altitude, target_vertical_speed, PIDController
from guidance.utils import haversine_distance
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

    vessel = conn.space_center.active_vessel
    telem = telemetry.Telemetry(conn, vessel)
    vehic = vehicle.Vehicle(conn, vessel)
    throttle_controller = PIDController(kp=0.1, ki=0.01, kd=0.02, setpoint=0.0, integral_limit=50.0)

    body = vessel.orbit.body
    gravity = body.surface_gravity          
    body_radius = body.equatorial_radius

    start_fuel = telem.fuel_mass()
    start_time = time.time()
    max_g = 0.0

    log_fields = ["time", "phase", "altitude", "vertical_speed", "horizontal_speed",
              "target_vertical_speed", "throttle", "mass", "fuel_mass", "g_force"]
    data_dir = os.path.join(os.path.dirname(__file__), "..", "data")
    logger = FlightLogger(data_dir, prefix="landing", fields=log_fields)
    desired_speed = 0.0   # so the first ticks have something to log
    legs_deployed = False  # control.legs reads False mid-animation, so latch it here

    phase = Phase.DEORBIT          
    last_time = time.time()

    try:
        while True:
            now = time.time()
            dt = now - last_time
            last_time = now
            max_g = max(max_g, telem.g_force())

            logger.log({
                "time": now - start_time,
                "phase": phase.name,
                "altitude": telem.altitude(),
                "vertical_speed": telem.vertical_speed(),
                "horizontal_speed": telem.horizontal_speed(),
                "target_vertical_speed": desired_speed,
                "throttle": vehic.throttle(),
                "mass": vehic.current_mass(),
                "fuel_mass": telem.fuel_mass(),
                "g_force": telem.g_force(),
            })


            if phase == Phase.DEORBIT:
                print("DEORBIT")
                vehic.enable_rcs()
                vehic.point_retrograde()
                time.sleep(25)

                while telem.horizontal_speed() > 1000.0:
                    vehic.set_throttle(1.0)
                    time.sleep(0.2)

                vehic.set_throttle(0.0)

                phase = Phase.COAST
                print("COAST")
                
            elif phase == Phase.COAST:

                if telem.altitude() <= 25000.0 and not vehic.brakes_status():
                    print("BRAKES")
                    vehic.apply_brakes()

                if telem.altitude() <= suicide_burn_altitude(
                    velocity= telem.vertical_speed(), max_deceleration=vehic.max_deceleration(),
                    gravity=gravity, k=0.9):
                    vehic.set_throttle(0.0)
                    print("SUICIDE BURN")
                    phase = Phase.DESCENT
                    print("DESCENT")

                time.sleep(0.1)


            elif phase == Phase.DESCENT:

                if telem.is_landed():
                    vehic.set_throttle(0.0)
                    phase = Phase.LANDED
                    continue
                
                desired_speed = -target_vertical_speed(telem.altitude(), vehic.max_deceleration(), gravity, k=0.9, touchdown_speed=2.0)
                throttle_controller.setpoint = desired_speed

                if telem.altitude() <= 1000.0 and not legs_deployed:
                    print("LEGS")
                    vehic.deploy_legs()
                    legs_deployed = True

                max_decel = vehic.max_deceleration()
                if max_decel <= 0.0:
                    raise RuntimeError("No thrust available during descent -- out of fuel?")
                
                hover = gravity / max_decel  
                throttle_input = hover + throttle_controller.update(telem.vertical_speed(), dt)
                vehic.set_throttle(throttle_input)

                time.sleep(0.05)

            elif phase == Phase.LANDED:
                print("LANDED")
                results = {
                    "landing_error_m": haversine_distance(
                        telem.latitude(), telem.longitude(),
                        target_latitude, target_longitude, body_radius),
                    "fuel_used_kg": start_fuel - telem.fuel_mass(),
                    "max_g": max_g,
                    "time_to_land_s": time.time() - start_time,
                }

                summary_path = write_summary(data_dir, results)
                print(f"Summary appended to {summary_path}")
                return results

    finally:
        logger.close()