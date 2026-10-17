"""Obstacle detection state and cmd_vel gate policy, shared by obstacle_detector.py and cmd_vel_gate.py
(ROS-free, so the tests exercise the same code as the nodes).

Sensor validity and obstacle policy are separate:
  validity  every required sensor must deliver valid data whose measurement time (header stamp) is
            recent; otherwise the detector reports sensors_ok = False and the gate stops the robot in
            every state, including a running detour.
  policy    what an obstacle means depends on the route supervisor's state: on a detour (AVOID) the
            avoid controller's own footprint checks handle obstacles; anywhere else the gate keeps a
            protective field in the direction of travel that grows with speed (the distance the base
            needs to stop), and stops in-place turns for an obstacle within the turn radius.

Directions: N_DIRECTIONS directions of travel, direction k at k * 45 deg counterclockwise from the front.
The distance in direction k is how far the body (grown by a side margin) can translate that way before it
touches the nearest obstacle point: points beside the body count too when the body sweeps into them
(e.g. moving diagonally). A translation between two directions is checked against both.
"""
import math

import numpy as np

REQUIRED, OPTIONAL = "required", "optional"
N_DIRECTIONS = 8
INF = float("inf")


def direction_name(k):
    return ("front", "front-left", "left", "rear-left", "rear", "rear-right", "right", "front-right")[k]


def contact_distances(x, y, ux, uy, half_x, half_y):
    """For points (x, y) and the rectangle +-half_x, +-half_y translating along the unit vector (ux, uy):
    the travel until each point touches the rectangle (inf if never; points inside are left out)."""
    lo = np.full(x.shape, -INF)
    hi = np.full(x.shape, INF)
    for p, u, h in ((x, ux, half_x), (y, uy, half_y)):
        if abs(u) < 1e-9:
            outside = np.abs(p) > h  # never within this slab
            lo[outside], hi[outside] = INF, -INF
        else:
            a, b = (p - h) / u, (p + h) / u
            lo = np.maximum(lo, np.minimum(a, b))
            hi = np.minimum(hi, np.maximum(a, b))
    hit = (lo <= hi) & (lo > 0)
    return np.where(hit, lo, INF)


def corridor_distances(x, y, half_length, half_width, side_margin, max_range, min_points):
    """How far the body (rectangle +-half_length x +-half_width, grown by side_margin) can translate in
    each of the N_DIRECTIONS directions before touching the min_points-th nearest obstacle point (inf for
    none within max_range). x, y: numpy arrays of the obstacle points in the body frame (already limited
    to the obstacle heights)."""
    dist = []
    for k in range(N_DIRECTIONS):
        th = 2 * math.pi * k / N_DIRECTIONS
        d = contact_distances(x, y, math.cos(th), math.sin(th), half_length + side_margin, half_width + side_margin)
        d = d[d <= max_range]
        dist.append(float(np.partition(d, min_points - 1)[min_points - 1]) if len(d) >= min_points else INF)
    return dist


class SensorTracker:
    """Latest obstacle data per sensor, judged by their measurement time.

    Per frame a sensor reports its corridor distances (N_DIRECTIONS, inf for none) and the obstacle points
    within the turn radius. A frame counts only if it is a valid measurement: the caller passes
    valid=False for a frame with too few usable points (e.g. an all-NaN depth image, which a real depth
    camera also returns for an object closer than its minimum range). Such a frame makes the sensor not
    current at once; it clears nothing.
    required sensors: missing, stale or invalid data -> sensors_ok False, obstacles assumed everywhere
        (fail-safe).
    optional sensors: used while current. When one goes stale (or invalid), its last valid obstacles are
        kept (a stale camera cannot clear what only it saw) until it delivers valid data again.
    A sensor that is switched off on purpose (e.g. depth_camera:=false) is left out of both lists.
    """

    def __init__(self, required, optional=(), timeout=0.5, future_tolerance=0.05, min_points=5):
        self.kind = {t: REQUIRED for t in required}
        self.kind.update({t: OPTIONAL for t in optional if t not in self.kind})
        self.timeout = timeout
        self.future_tolerance = future_tolerance
        self.min_points = min_points
        self.reset()

    def reset(self):
        """Forget all data (e.g. the clock jumped back)."""
        self.latest = {}  # topic -> {stamp, dist, near, valid}; obstacles of the last valid frame
        self.rejected = 0

    def update(self, topic, stamp, now, dist, near_count, valid=True):
        """Store a sensor result. stamp: measurement time, now: current time [s], dist: corridor distances.
        Returns False if rejected."""
        if topic not in self.kind or not (math.isfinite(stamp) and stamp > 0):
            self.rejected += 1
            return False
        if stamp - now > self.future_tolerance:
            self.rejected += 1  # from the future: clock offset, not usable
            return False
        last = self.latest.get(topic)
        if last is not None and stamp < last["stamp"]:
            self.rejected += 1  # older than what we have
            return False
        if valid:
            self.latest[topic] = {"stamp": stamp, "dist": [float(d) for d in dist], "near": int(near_count),
                                  "valid": True}
        else:
            entry = dict(last) if last is not None else {"dist": [INF] * N_DIRECTIONS, "near": 0}
            entry.update(stamp=stamp, valid=False)
            self.latest[topic] = entry
        return True

    def fresh(self, topic, now):
        entry = self.latest.get(topic)
        return (entry is not None and entry["valid"] and
                -self.future_tolerance <= now - entry["stamp"] <= self.timeout)

    def evaluate(self, now):
        """(dist, near, sensors_ok, problems): nearest obstacle per direction, an obstacle within the turn
        radius, all required sensors current, and human-readable problems."""
        dist = [INF] * N_DIRECTIONS
        near = False
        sensors_ok = True
        problems = []

        def take(entry):
            nonlocal near
            for k in range(N_DIRECTIONS):
                dist[k] = min(dist[k], entry["dist"][k])
            near |= entry["near"] >= self.min_points

        for topic, kind in sorted(self.kind.items()):
            entry = self.latest.get(topic)
            if self.fresh(topic, now):
                take(entry)
                continue
            if entry is None:
                age = "no data"
            elif not entry["valid"]:
                age = "no valid measurement in its last frame"
            else:
                age = "%.2f s old" % (now - entry["stamp"])
            if kind == REQUIRED:
                sensors_ok = False
                dist = [0.0] * N_DIRECTIONS
                near = True
                problems.append("required %s: %s" % (topic, age))
            elif entry is not None:
                take(entry)
                problems.append("optional %s: %s, keeping its last obstacles" % (topic, age))
        return dist, near, sensors_ok, problems


def stopping_distance(speed, dec, latency, margin):
    """Distance the base needs to stop from speed [m/s]: latency [s] at speed, braking at dec, plus margin."""
    speed = abs(speed)
    return speed * latency + speed * speed / (2 * max(dec, 1e-3)) + margin


def travel_directions(vx, vy):
    """Corridors a translation (vx, vy) moves into: the one it points along, or the two bracketing it."""
    a = math.atan2(vy, vx) % (2 * math.pi) / (2 * math.pi) * N_DIRECTIONS
    k = int(math.floor(a + 1e-9)) % N_DIRECTIONS
    return [k] if abs(a - round(a)) < 1e-6 else [k, (k + 1) % N_DIRECTIONS]


def gate_command(cmd, detector_fresh, sensors_ok, dist, near, avoid_active, speed, dec=0.5, latency=0.3, margin=0.3):
    """cmd_vel gate policy. cmd: (vx, vy, wz); dist: corridor distances from the body; speed: the robot's
    current speed (the faster of it and the command counts). Returns ((vx, vy, wz), reason); reason is
    None when the command passes.

    - non-finite command                         -> stop
    - detector silent or a required sensor stale -> stop (in every state)
    - supervisor on a detour (AVOID, fresh)      -> pass: the avoid controller checks its own motion
    - otherwise: a translation stops when an obstacle in the corridor(s) it moves into is closer than the
      stopping distance; a turn in place stops for an obstacle within the turn radius (the corners
      sweep a circle).
    """
    vx, vy, wz = cmd
    if not all(math.isfinite(v) for v in cmd):
        return (0.0, 0.0, 0.0), "invalid command"
    if not detector_fresh:
        return (0.0, 0.0, 0.0), "obstacle detector silent"
    if not sensors_ok:
        return (0.0, 0.0, 0.0), "required sensor data missing"
    if avoid_active:
        return (vx, vy, wz), None
    v = math.hypot(vx, vy)
    if v >= 1e-3:
        need = stopping_distance(max(v, abs(speed)), dec, latency, margin)
        for k in travel_directions(vx, vy):
            if dist[k] < need:
                return (0.0, 0.0, 0.0), "obstacle %s at %.2f m (needs %.2f m)" % (direction_name(k), dist[k], need)
        return (vx, vy, wz), None
    if near and abs(wz) > 1e-6:
        return (0.0, 0.0, 0.0), "obstacle within the turn radius"
    return (0.0, 0.0, wz), None
