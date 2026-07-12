"""
utils.py

Small, pure, easily-testable helper functions: unit conversions,
coordinate transforms, vector math helpers. Nothing in this file
should touch kRPC or need a live game connection -- that's what
makes it easy to unit test (see tests/test_utils.py).
"""

import math


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float,
                        body_radius: float) -> float:
    """Great-circle surface distance between two lat/lon points.

    Useful for computing landing error: distance between where you
    actually landed and your intended target coordinates.

    Args:
        lat1, lon1: first point, degrees
        lat2, lon2: second point, degrees
        body_radius: radius of the celestial body, meters
            (e.g. Kerbin ~600,000 m)

    Returns:
        Distance in meters.
    """
    raise NotImplementedError


def clamp(value: float, min_value: float, max_value: float) -> float:
    """Clamp value to the range [min_value, max_value]."""
    return max(min_value, min(value, max_value))
