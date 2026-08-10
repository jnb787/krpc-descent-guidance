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


def surface_offset(lat: float, lon: float, target_lat: float, target_lon: float,
                    body_radius: float) -> tuple:
    """Signed north/east offset of a position from a target, in meters.

    Converts an angular lat/lon difference into local tangent-plane
    distances, so a horizontal-position controller can work in meters
    instead of degrees (a degree of longitude shrinks by cos(latitude),
    so raw degree errors are not comparable between the two axes).

    Args:
        lat, lon: current position, in degrees
        target_lat, target_lon: target position, in degrees
        body_radius: radius of the body, in meters

    Returns:
        (north, east) offset in meters, positive when the current
        position is north/east *of the target*. Feed these straight to a
        PID with setpoint 0.0: the resulting error points back at the
        target.
    """
    delta_lat = math.radians(lat - target_lat)

    # Wrap into [-180, 180] so a target across the antimeridian gives the
    # short way round rather than a near-full lap of the body.
    delta_lon = math.radians((lon - target_lon + 180.0) % 360.0 - 180.0)

    # Equirectangular approximation: scale the east axis by the cosine of
    # the mean latitude. Good to well under a meter over the few km of
    # cross-range error a descent actually has to null out.
    mean_lat = math.radians((lat + target_lat) / 2.0)

    north = body_radius * delta_lat
    east = body_radius * math.cos(mean_lat) * delta_lon

    return (north, east)


def limit_tilt(north: float, east: float, max_tilt: float) -> tuple:
    """Scale a (north, east) tilt command down to a maximum magnitude.

    The components are ratios against an up-component of 1.0, so their
    magnitude is tan(tilt angle). Scaling both by the same factor keeps the
    commanded bearing intact -- clamping each axis on its own would bend the
    command toward the diagonal, and would also let the corner reach
    sqrt(2) * max_tilt.

    Args:
        north, east: requested tilt components
        max_tilt: largest allowed magnitude, as tan(angle)

    Returns:
        (north, east, demand_deg, cmd_deg) -- the limited components, the
        angle originally asked for, and the angle actually commanded. The
        gap between the last two is how far into saturation the controller
        is, which a limited command alone cannot tell you.
    """
    magnitude = math.hypot(north, east)
    demand_deg = math.degrees(math.atan(magnitude))

    if magnitude > max_tilt:
        scale = max_tilt / magnitude
        north *= scale
        east *= scale
        magnitude = max_tilt

    return (north, east, demand_deg, math.degrees(math.atan(magnitude)))


def clamp(value: float, min_value: float, max_value: float) -> float:
    """Clamp value to the range [min_value, max_value]."""
    return max(min_value, min(value, max_value))
