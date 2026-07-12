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


def main():
    conn = telemetry.connect(name="Manual Flight Logger")
    vessel = conn.space_center.active_vessel
    print(f"Connected. Tracking vessel: {vessel.name}")
    print("Fly manually in KSP now. Press Ctrl+C here to stop logging.\n")

    # TODO: replace this with real telemetry reads once telemetry.py
    # is implemented, e.g.:
    #
    # flight = vessel.flight(vessel.orbit.body.reference_frame)
    # while True:
    #     print(f"altitude={flight.surface_altitude:.1f} m  "
    #           f"vspeed={flight.vertical_speed:.1f} m/s")
    #     time.sleep(0.5)

    raise NotImplementedError("Fill in the telemetry loop above.")


if __name__ == "__main__":
    main()
