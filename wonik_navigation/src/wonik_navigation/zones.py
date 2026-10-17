"""Zones file helpers shared by zone_manager and route_supervisor.

A zones yaml has a frame_id and a list of zones: {name, type (avoid | stop), polygon: [[x, y], ...]}.
"""
import yaml


def load_zones(path):
    """(frame_id, zones) from a zones yaml."""
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return data.get("frame_id", "map"), data.get("zones", []) or []


def inside(x, y, polygon):
    """Ray casting point-in-polygon test."""
    result = False
    for (x0, y0), (x1, y1) in zip(polygon, polygon[1:] + polygon[:1]):
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            result = not result
    return result


def zone_at(x, y, zones):
    """First zone containing (x, y), or None."""
    for z in zones:
        if inside(x, y, z["polygon"]):
            return z
    return None
