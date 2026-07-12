"""
guidance
========

Core package for the autonomous powered-descent guidance system.

Modules:
    telemetry   - reads live vessel/flight data from the kRPC server
    vehicle     - wraps vessel state, engine/thrust/fuel info, staging
    controllers - PID controller and suicide-burn guidance math
    mission     - top-level flight sequence state machine
    utils       - math helpers, unit conversions, coordinate transforms
"""

__version__ = "0.1.0"
