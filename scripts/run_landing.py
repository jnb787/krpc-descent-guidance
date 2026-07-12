#!/usr/bin/env python3
"""
run_landing.py

Main entry point. Connects to a running KSP instance via kRPC and
runs the full autonomous descent-and-landing sequence.

Usage:
    python scripts/run_landing.py --lat -0.09720 --lon -74.55767

(Those default coordinates are KSC's launchpad, just as a placeholder
target -- swap in wherever you actually want to land.)
"""

import argparse
import sys
import os

# allow running this script directly without installing the package
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from guidance import telemetry, mission


def main():
    parser = argparse.ArgumentParser(description="Autonomous KSP descent guidance")
    parser.add_argument("--lat", type=float, required=True, help="Target latitude")
    parser.add_argument("--lon", type=float, required=True, help="Target longitude")
    args = parser.parse_args()

    print("Connecting to kRPC server...")
    conn = telemetry.connect(name="Descent Guidance Run")
    print(f"Connected. Active vessel: {conn.space_center.active_vessel.name}")

    results = mission.run_mission(conn, args.lat, args.lon)
    print("Landing complete. Results:")
    for key, value in results.items():
        print(f"  {key}: {value}")


if __name__ == "__main__":
    main()
