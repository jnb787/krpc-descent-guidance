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
import math
import time
import os

from guidance import telemetry, vehicle
from guidance.flight_log import FlightLogger, write_summary
from guidance.controllers import suicide_burn_altitude, target_vertical_speed, PIDController
from guidance.utils import haversine_distance, surface_offset, ballistic_fall_time, predicted_impact_offset
from enum import Enum, auto

class Phase(Enum):
    DEORBIT = auto()
    CORRECT = auto()
    COAST = auto()
    DESCENT = auto()
    LANDED = auto()


# Steepest tilt off vertical the descent will command, as tan(angle) -- the
# north/east components of the direction vector are a ratio against up=1.0.
MAX_TILT = math.tan(math.radians(15.0))

CORRECT_THROTTLE = 0.1
CORRECT_DV_TOLERANCE = 1.0  # m/s
CORRECT_FLOOR = 40000.0    # m
POINT_TOLERANCE = 10.0   # degrees

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
    throttle_controller = PIDController(kp=0.2, ki=0.02, kd=0.025, setpoint=0.0, integral_limit=10.0)
    north_controller = PIDController(kp=0.0006, ki=0.0, kd=0.013, setpoint=0.0, integral_limit=0.0)
    east_controller = PIDController(kp=0.0006, ki=0.0, kd=0.013, setpoint=0.0, integral_limit=0.0)

    body = vessel.orbit.body
    gravity = body.surface_gravity          
    body_radius = body.equatorial_radius

    start_fuel = telem.fuel_mass()
    start_time = time.time()
    max_g = 0.0

    log_fields = ["time", "ut", "phase", "altitude", "vertical_speed", "horizontal_speed",
              "target_vertical_speed", "throttle", "mass", "fuel_mass", "g_force",
              # horizontal guidance: where we are vs the target, what we asked
              # for, and what the vessel actually did about it
              "north_offset", "east_offset", "north_input", "east_input",
              "tilt_demand_deg", "tilt_cmd_deg", "pitch", "heading",
              # the impact predictor CORRECT steers on -- logged in every
              # phase so the ballistic guess can be scored against where the
              # vehicle actually came down. The dv CORRECT commands is just
              # -predicted / max(t_fall, 1), so it is not logged separately.
              "velocity_north", "velocity_east", "t_fall",
              "predicted_north", "predicted_east",
              # aero authority, which is what competes with the tilt command
              "dynamic_pressure", "drag"]
    data_dir = os.path.join(os.path.dirname(__file__), "..", "data")
    logger = FlightLogger(data_dir, prefix="landing", fields=log_fields)
    desired_speed = 0.0   # so the first ticks have something to log
    legs_deployed = False  # control.legs reads False mid-animation, so latch it here

    # Horizontal guidance state, logged every tick but only driven in DESCENT.
    north_input = east_input = 0.0
    tilt_demand_deg = tilt_cmd_deg = 0.0

    phase = Phase.DEORBIT       
    print("DEORBIT")
    vehic.enable_rcs()
    vehic.point_retrograde()
   
    last_time = time.time()

    try:
        while True:
            now = time.time()
            dt = now - last_time
            last_time = now
            max_g = max(max_g, telem.g_force())

            # Computed every tick, not just in DESCENT: watching the offset
            # evolve through COAST is what tells us whether cross-range can be
            # corrected up there, where there is far more time than the ~20 s
            # the burn actually lasts.
            north_offset, east_offset = surface_offset(
                telem.latitude(), telem.longitude(),
                target_latitude, target_longitude, body_radius)

            velocity_up, velocity_north, velocity_east = telem.velocity_une()

            t_fall = ballistic_fall_time(telem.altitude(), velocity_up, gravity)
            predicted_north, predicted_east = predicted_impact_offset(
                north_offset, east_offset, velocity_north, velocity_east, t_fall)
            
            drag_x, drag_y, drag_z = telem.drag()

            logger.log({
                "time": now - start_time,
                "ut": telem.ut(),
                "phase": phase.name,
                "altitude": telem.altitude(),
                "vertical_speed": telem.vertical_speed(),
                "horizontal_speed": telem.horizontal_speed(),
                "target_vertical_speed": desired_speed,
                "throttle": vehic.throttle(),
                "mass": vehic.current_mass(),
                "fuel_mass": telem.fuel_mass(),
                "g_force": telem.g_force(),
                "north_offset": north_offset,
                "east_offset": east_offset,
                "north_input": north_input,
                "east_input": east_input,
                "tilt_demand_deg": tilt_demand_deg,
                "tilt_cmd_deg": tilt_cmd_deg,
                "pitch": telem.pitch(),
                "heading": telem.heading(),
                "velocity_north": velocity_north,
                "velocity_east": velocity_east,
                "t_fall": t_fall,
                "predicted_north": predicted_north,
                "predicted_east": predicted_east,
                "dynamic_pressure": telem.dynamic_pressure(),
                "drag": math.sqrt(drag_x**2 + drag_y**2 + drag_z**2),
            })


            if phase == Phase.DEORBIT:
                if vehic.pointing_error() < POINT_TOLERANCE:

                    while telem.horizontal_speed() > 1000.0:
                        vehic.set_throttle(1.0)
                        time.sleep(0.2)

                    vehic.set_throttle(0.0)
                    vehic.engage()

                    phase = Phase.CORRECT
                    print("CORRECT")

            elif phase == Phase.CORRECT:
                dv_north = -predicted_north / max(t_fall, 1.0)
                dv_east = -predicted_east / max(t_fall, 1.0)
                dv_magnitude = math.hypot(dv_north, dv_east)

                if dv_magnitude <= CORRECT_DV_TOLERANCE or telem.altitude() <= CORRECT_FLOOR:
                    vehic.set_throttle(0.0)
                    vehic.point_retrograde()
                    phase = Phase.COAST
                    print("COAST")

                else:
                    vehic.point(north=dv_north, east=dv_east, up=0.0)

                    if vehic.pointing_error() < POINT_TOLERANCE:
                        vehic.set_throttle(CORRECT_THROTTLE)
                        
                    else:
                        vehic.set_throttle(0.0)

                    time.sleep(0.1)

            elif phase == Phase.COAST:

                if telem.altitude() <= 25000.0 and not vehic.brakes_status():
                    print("BRAKES")
                    vehic.apply_brakes()

                if telem.effective_altitude() <= suicide_burn_altitude(
                    velocity= telem.vertical_speed(), max_deceleration=vehic.max_deceleration(),
                    gravity=gravity, k=0.75):
                    vehic.set_throttle(0.0)
                    print("SUICIDE BURN")
                    phase = Phase.DESCENT
                    print("DESCENT")
                    vehic.engage()

                time.sleep(0.1)


            elif phase == Phase.DESCENT:

                if telem.is_landed():
                    vehic.set_throttle(0.0)
                    phase = Phase.LANDED
                    continue
                
                desired_speed = -target_vertical_speed(telem.effective_altitude(), vehic.max_deceleration(), gravity, k=0.75, touchdown_speed=2.0)
                throttle_controller.setpoint = desired_speed

                north_input = north_controller.update(north_offset, dt)
                east_input = east_controller.update(east_offset, dt)

                tilt = math.hypot(north_input, east_input)
                tilt_demand_deg = math.degrees(math.atan(tilt))
                if tilt > MAX_TILT:
                    north_input *= MAX_TILT / tilt
                    east_input  *= MAX_TILT / tilt
                tilt_cmd_deg = math.degrees(math.atan(math.hypot(north_input, east_input)))

                vehic.point(up=1.0, north=north_input, east=east_input)


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
                vehic.disable_rcs()
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
        vehic.set_throttle(0.0)
        logger.close()