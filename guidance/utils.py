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
    """Compute the great-circle distance between two points on the surface of a sphere.

    Args:
        lat1, lon1: latitude and longitude of the first point, in degrees
        lat2, lon2: latitude and longitude of the second point, in degrees
        body_radius: radius of the sphere, in meters

        returns: distance between the two points, in meters
    """
    # Convert to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    
    # Compute the difference in longitudes and latitudes
    delta_lat = lat2 - lat1
    delta_lon = lon2 - lon1

    # Compute the haversine formula
    a = math.sin(delta_lat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(delta_lon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return body_radius * c


def clamp(value: float, min_value: float, max_value: float) -> float:
    """Clamp value to the range [min_value, max_value]."""
    return max(min_value, min(value, max_value))
