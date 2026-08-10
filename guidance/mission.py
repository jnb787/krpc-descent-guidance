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
from guidance.utils import haversine_distance, surface_offset, limit_tilt
from enum import Enum, auto

class Phase(Enum):
    DEORBIT = auto()
    COAST = auto()
    DESCENT = auto()
    LANDED = auto()

max_tilt = math.tan(math.radians(15.0))
com_height = 8.75

def run_mission(conn, target_latitude: float, target_longitude: float,
                target_elevation: float = 0.0) -> dict:
    """Run the full autonomous landing sequence.

    Args:
        conn: an active kRPC connection (see telemetry.connect())
        target_latitude: landing target latitude, degrees
        target_longitude: landing target longitude, degrees
        target_elevation: sea-level elevation of the landing surface, m.
            Measure it once by parking the vessel on the site and reading
            mean_altitude() minus com_height. It matters because a built
            pad deck sits above the terrain datum that altitude() reports,
            so guidance trusting altitude() believes it has several metres
            more to fall than it does and arrives still descending.

    Returns:
        A results dict with keys like: landing_error_m, fuel_used_kg,
        max_g, time_to_land_s -- this is what you'll aggregate across
        20+ runs for your SMART goal's measurable success criteria.
    """

    vessel = conn.space_center.active_vessel
    telem = telemetry.Telemetry(conn, vessel)
    vehic = vehicle.Vehicle(conn, vessel)
    throttle_controller = PIDController(kp=0.2, ki=0.02, kd=0.025, setpoint=0.0, integral_limit=10.0)
    north_controller = PIDController(kp=3e-4, ki=0.0, kd=5e-3, setpoint=0.0, integral_limit=0.0)
    east_controller = PIDController(kp=3e-4, ki=0.0, kd=5e-3, setpoint=0.0, integral_limit=0.0)
    coast_north_controller = PIDController(kp=1e-4, ki=0.0, kd=0.0, setpoint=0.0, integral_limit=0.0)
    coast_east_controller = PIDController(kp=1e-4, ki=0.0, kd=0.0, setpoint=0.0, integral_limit=0.0)

    body = vessel.orbit.body
    gravity = body.surface_gravity          
    body_radius = body.equatorial_radius

    start_fuel = telem.fuel_mass()
    start_time = time.time()
    max_g = 0.0

    log_fields = ["time", "ut", "phase", "altitude", "height_above_target",
              "vertical_speed", "horizontal_speed",
              "target_vertical_speed", "throttle", "mass", "fuel_mass", "g_force",
              # horizontal guidance: where we are vs the target, and what we asked for
              "north_offset", "east_offset", "velocity_north", "velocity_east",
              "north_input", "east_input", "tilt_demand_deg", "tilt_cmd_deg",
              # what the vessel actually did about it -- pitch/heading are the
              # commanded attitude achieved, aoa/sideslip the attitude relative
              # to the airflow, which is what generates force
              "pitch", "heading", "angle_of_attack", "sideslip_angle",
              # aero authority, which is what competes with the tilt command,
              # and the charge that limits how long we can fight it
              "dynamic_pressure", "drag", "lift", "electric_charge"]
    data_dir = os.path.join(os.path.dirname(__file__), "..", "data")
    logger = FlightLogger(data_dir, prefix="landing", fields=log_fields)
    desired_speed = 0.0   # so the first ticks have something to log
    legs_deployed = False  # control.legs reads False mid-animation, so latch it here

    # Horizontal guidance state, logged every tick but only driven in DESCENT.
    north_input = east_input = 0.0
    tilt_demand_deg = tilt_cmd_deg = 0.0

    steering = False
    coast_max_tilt = math.tan(math.radians(5.0))
    time_lead = 60.0
    aero_sign = -1.0

    phase = Phase.DEORBIT          
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

            # Distance the feet still have to travel to reach the landing
            # surface. Referenced to sea level and the site's own elevation
            # rather than to altitude(), which measures to whatever terrain
            # is underneath right now -- that steps by the deck height as you
            # cross onto a built pad, and wanders as terrain changes during
            # the descent.
            height_above_target = telem.mean_altitude() - target_elevation - com_height

            velocity_north, velocity_east = telem.velocity_ne()
            drag_x, drag_y, drag_z = telem.drag()
            lift_x, lift_y, lift_z = telem.lift()

            logger.log({
                "time": now - start_time,
                "ut": telem.ut(),
                "phase": phase.name,
                "altitude": telem.altitude(),
                "height_above_target": height_above_target,
                "vertical_speed": telem.vertical_speed(),
                "horizontal_speed": telem.horizontal_speed(),
                "target_vertical_speed": desired_speed,
                "throttle": vehic.throttle(),
                "mass": vehic.current_mass(),
                "fuel_mass": telem.fuel_mass(),
                "g_force": telem.g_force(),
                "north_offset": north_offset,
                "east_offset": east_offset,
                "velocity_north": velocity_north,
                "velocity_east": velocity_east,
                "north_input": north_input,
                "east_input": east_input,
                "tilt_demand_deg": tilt_demand_deg,
                "tilt_cmd_deg": tilt_cmd_deg,
                "pitch": telem.pitch(),
                "heading": telem.heading(),
                "angle_of_attack": telem.angle_of_attack(),
                "sideslip_angle": telem.sideslip_angle(),
                "dynamic_pressure": telem.dynamic_pressure(),
                "drag": math.sqrt(drag_x**2 + drag_y**2 + drag_z**2),
                "lift": math.sqrt(lift_x**2 + lift_y**2 + lift_z**2),
                "electric_charge": telem.electric_charge(),
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

                correcting = telem.dynamic_pressure() >= 200.0

                if correcting and not steering:
                    vehic.engage()
                    steering = True

                if correcting and steering:
                    error_north = north_offset + time_lead * velocity_north
                    error_east  = east_offset  + time_lead * velocity_east

                    north_input = aero_sign * coast_north_controller.update(error_north, dt)
                    east_input  = aero_sign * coast_east_controller.update(error_east, dt)

                    north_input, east_input, tilt_demand_deg, tilt_cmd_deg = limit_tilt(
                        north_input, east_input, coast_max_tilt)

                    vehic.point(up=1.0, north=north_input, east=east_input)

                if height_above_target <= suicide_burn_altitude(
                    velocity= telem.vertical_speed(), max_deceleration=vehic.max_deceleration(),
                    gravity=gravity, k=0.75):
                    vehic.set_throttle(0.0)
                    print("SUICIDE BURN")
                    phase = Phase.DESCENT
                    print("DESCENT")

                if telem.altitude() <= 25000.0 and not vehic.brakes_status():
                    print("BRAKES")
                    vehic.apply_brakes()

                

                time.sleep(0.1)


            elif phase == Phase.DESCENT:

                if telem.is_landed():
                    vehic.set_throttle(0.0)
                    phase = Phase.LANDED
                    continue
                
                # Clamped at zero: height_above_target goes negative if the site
                # elevation is measured a little high, and target_vertical_speed
                # takes a sqrt that would raise a math domain error mid-descent.
                # At or below the deck the right target is just touchdown speed.
                desired_speed = -target_vertical_speed(max(height_above_target, 0.0), vehic.max_deceleration(), gravity, k=0.75, touchdown_speed=2.0)
                throttle_controller.setpoint = desired_speed

                north_input = north_controller.update(north_offset, dt)
                east_input = east_controller.update(east_offset, dt)

                north_input, east_input, tilt_demand_deg, tilt_cmd_deg = limit_tilt(
                    north_input, east_input, max_tilt)

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
        logger.close()