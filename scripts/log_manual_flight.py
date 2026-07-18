#!/usr/bin/env python3
"""
log_manual_flight.py

Week 1-2 exploration script. Fly the vessel MANUALLY in KSP while this
script connects via kRPC and prints/logs live telemetry. The goal here
isn't autonomy yet -- it's getting comfortable with the kRPC API and
understanding what real telemetry values look like during an actual
descent, before you try to control it programmatically.

Usage:
    python scripts/log_manual_flight.py
    (then fly manually in KSP and watch the printed telemetry)
"""

import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from guidance import telemetry
from guidance.flight_log import FlightLogger


def main():
    conn = telemetry.connect(name="Manual Flight Logger")
    vessel = conn.space_center.active_vessel
    print(f"Connected. Tracking vessel: {vessel.name}")
    print("Fly manually in KSP now. Press Ctrl+C here to stop logging.\n")


    telem = telemetry.Telemetry(conn, vessel)

    data_dir = os.path.join(os.path.dirname(__file__), "..", "data")
    logger = FlightLogger(data_dir, prefix="manual_flight",
                              fields=["time", "altitude", "vertical_speed"])

    start_time = time.time()

    try:
        while True:
            elapsed_time = time.time() - start_time
            x, y, z, w = telem.orientation()
            print(f"\n =============================\n\n"
                  f"ALTITUDE: {telem.altitude():.1f} m"
                  f"\nSPEED: vspeed={telem.vertical_speed():.1f} m/s | "
                  f"hspeed={telem.horizontal_speed():.1f} m/s"
                  f"\nMASS: total={telem.total_mass():.1f} kg | "
                  f"propellant={telem.fuel_mass():.1f} kg"
                  f"\nORIENTATION: x={x:.3f} | "
                  f"y={y:.3f} | "
                  f"z={z:.3f} | "
                  f"w={w:.3f}"
                  )

            logger.log({"time": elapsed_time, "altitude": telem.altitude(), "vertical_speed": telem.vertical_speed()})

            time.sleep(0.5)
    finally:
        logger.close()




if __name__ == "__main__":
    main()
