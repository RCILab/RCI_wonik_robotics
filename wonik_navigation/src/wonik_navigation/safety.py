"""Obstacle detection state and cmd_vel gate policy, shared by obstacle_detector.py and cmd_vel_gate.py
(ROS-free, so the tests exercise the same code as the nodes).

Sensor validity and obstacle policy are separate:
  validity  every required sensor must deliver valid data whose measurement time (header stamp) is
            recent; otherwise the detector reports sensors_ok = False and the gate stops the robot in
            every state, including a running detour.
  policy    what an obstacle means depends on the route supervisor's state: on a detour (AVOID) the
            avoid controller's own footprint checks handle obstacles; anywhere else the gate keeps a
            protective field in the direction of travel that grows with speed (the distance the base
            needs to stop), and stops an in-place turn when the footprint would touch an obstacle within
            the angle the base needs to stop turning.

The robot is described in the base frame (base_footprint), whose origin is the point the base turns
about in place. The body need not be centred on it (e.g. a single-drive pallet truck turns about its
fork roller axle, with the body ahead of it).

Directions: N_DIRECTIONS directions of travel, direction k at k * 45 deg counterclockwise from the front.
The distance in direction k is how far the body's bounding box (grown by a side margin) can translate
that way before it touches the nearest obstacle point: points beside the body count too when the body
sweeps into them (e.g. moving diagonally). A translation between two directions is checked against both.

Turns: for each turning direction (counterclockwise, clockwise), the angle the footprint polygon can
turn in place before it touches the nearest obstacle point (TurnClearance).
"""
import math

import numpy as np

REQUIRED, OPTIONAL = "required", "optional"
N_DIRECTIONS = 8
INF = float("inf")
FREE_TURN = [INF, INF]  # turn clearance with no obstacle: [counterclockwise, clockwise]


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


def bounding_box(polygon):
    """(x_min, x_max, y_min, y_max) of a polygon [[x, y], ...]."""
    p = np.asarray(polygon, dtype=float)
    return float(p[:, 0].min()), float(p[:, 0].max()), float(p[:, 1].min()), float(p[:, 1].max())


def corridor_distances(x, y, body, side_margin, max_range, min_points):
    """How far the body box (x_min, x_max, y_min, y_max in the base frame, grown by side_margin) can
    translate in each of the N_DIRECTIONS directions before touching the min_points-th nearest obstacle
    point (inf for none within max_range). x, y: numpy arrays of the obstacle points in the base frame
    (already limited to the obstacle heights)."""
    x_min, x_max, y_min, y_max = body
    cx, cy = (x_min + x_max) / 2, (y_min + y_max) / 2
    half_x, half_y = (x_max - x_min) / 2 + side_margin, (y_max - y_min) / 2 + side_margin
    dist = []
    for k in range(N_DIRECTIONS):
        th = 2 * math.pi * k / N_DIRECTIONS
        d = contact_distances(x - cx, y - cy, math.cos(th), math.sin(th), half_x, half_y)
        d = d[d <= max_range]
        dist.append(float(np.partition(d, min_points - 1)[min_points - 1]) if len(d) >= min_points else INF)
    return dist


def inside_polygon(x, y, polygon):
    """Boolean array: points (x, y) inside the polygon [[x, y], ...] (any simple polygon, convex or not)."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    inside = np.zeros(x.shape, dtype=bool)
    p = np.asarray(polygon, dtype=float)
    for (x0, y0), (x1, y1) in zip(p, np.roll(p, -1, axis=0)):
        crosses = (y0 > y) != (y1 > y)
        with np.errstate(divide="ignore", invalid="ignore"):
            x_cross = x0 + (y - y0) * (x1 - x0) / (y1 - y0)
        inside ^= crosses & (x < x_cross)
    return inside


class TurnClearance:
    """How far the footprint polygon (base frame) can turn in place, about the base frame origin, before
    it touches an obstacle point.

    A point at radius r is touched by the footprint points at the same radius. For radius bins of r_step
    and angle bins of a_step, the angles the footprint covers are precomputed, and from them, for every
    (radius bin, angle bin) of an obstacle point, the turn that first brings a covered angle onto it in
    each direction. The bins are conservative: the angles returned are never larger than the exact ones,
    and at most about 3 * a_step smaller. Points farther than the footprint's farthest point are never
    touched; points inside the footprint (the robot itself) are left out."""

    def __init__(self, polygon, r_step=0.01, a_step=math.radians(1.0)):
        self.polygon = np.asarray(polygon, dtype=float)
        self.r_step = r_step
        self.n_a = int(round(2 * math.pi / a_step))
        self.a_step = 2 * math.pi / self.n_a
        self.r_max = float(np.hypot(self.polygon[:, 0], self.polygon[:, 1]).max())
        n_r = int(math.ceil(self.r_max / r_step)) + 1

        # covered[i, j]: some footprint point with radius in bin i and angle in bin j (sampled finely)
        sub = 4
        radii = (np.arange(n_r * sub) + 0.5) * (r_step / sub)
        angles = (np.arange(self.n_a * sub) + 0.5) * (self.a_step / sub)
        rr, aa = np.meshgrid(radii, angles, indexing="ij")
        hit = inside_polygon(rr * np.cos(aa), rr * np.sin(aa), self.polygon)
        covered = hit.reshape(n_r, sub, self.n_a, sub).any(axis=(1, 3))
        # a footprint edge crossing a bin between the samples: grow the coverage by one angle bin
        covered = covered | np.roll(covered, 1, axis=1) | np.roll(covered, -1, axis=1)

        # ccw[i, j]: bins from the nearest covered bin clockwise of j up to j (the footprint turns
        # counterclockwise onto the point); cw[i, j] the same counterclockwise of j
        self.ccw = np.full((n_r, self.n_a), INF)
        self.cw = np.full((n_r, self.n_a), INF)
        for i in range(n_r):
            cov = np.flatnonzero(covered[i])
            if len(cov) == 0:
                continue
            j = np.arange(self.n_a)
            back = (j[:, None] - cov[None, :]) % self.n_a  # bins from a covered bin forward to j
            self.ccw[i] = np.maximum(back.min(axis=1) - 1, 0) * self.a_step
            fwd = (cov[None, :] - j[:, None]) % self.n_a
            self.cw[i] = np.maximum(fwd.min(axis=1) - 1, 0) * self.a_step

    def clearance(self, x, y, min_points=1):
        """[counterclockwise, clockwise] turn [rad] until the footprint touches the min_points-th nearest
        of the obstacle points (x, y) (inf if fewer of them are ever touched)."""
        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        r = np.hypot(x, y)
        near = r < self.r_max
        x, y, r = x[near], y[near], r[near]
        outside = ~inside_polygon(x, y, self.polygon)
        x, y, r = x[outside], y[outside], r[outside]
        i = np.minimum((r / self.r_step).astype(int), self.ccw.shape[0] - 1)
        j = ((np.arctan2(y, x) % (2 * math.pi)) / self.a_step).astype(int) % self.n_a
        out = []
        for table in (self.ccw, self.cw):
            a = table[i, j]
            a = a[np.isfinite(a)]
            out.append(float(np.partition(a, min_points - 1)[min_points - 1]) if len(a) >= min_points else INF)
        return out


class SensorTracker:
    """Latest obstacle data per sensor, judged by their measurement time.

    Per frame a sensor reports its corridor distances (N_DIRECTIONS, inf for none) and its turn clearance
    ([counterclockwise, clockwise], inf for none). A frame counts only if it is a valid measurement: the
    caller passes valid=False for a frame with too few usable points (e.g. an all-NaN depth image, which a
    real depth camera also returns for an object closer than its minimum range). Such a frame makes the
    sensor not current at once; it clears nothing.
    required sensors: missing, stale or invalid data -> sensors_ok False, obstacles assumed everywhere
        (fail-safe).
    optional sensors: used while current. When one goes stale (or invalid), its last valid obstacles are
        kept (a stale camera cannot clear what only it saw) until it delivers valid data again.
    A sensor that is switched off on purpose (e.g. depth_camera:=false) is left out of both lists.
    """

    def __init__(self, required, optional=(), timeout=0.5, future_tolerance=0.05):
        self.kind = {t: REQUIRED for t in required}
        self.kind.update({t: OPTIONAL for t in optional if t not in self.kind})
        self.timeout = timeout
        self.future_tolerance = future_tolerance
        self.reset()

    def reset(self):
        """Forget all data (e.g. the clock jumped back)."""
        self.latest = {}  # topic -> {stamp, dist, turn, valid}; obstacles of the last valid frame
        self.rejected = 0

    def update(self, topic, stamp, now, dist, turn, valid=True):
        """Store a sensor result. stamp: measurement time, now: current time [s], dist: corridor distances,
        turn: turn clearance. Returns False if rejected."""
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
            self.latest[topic] = {"stamp": stamp, "dist": [float(d) for d in dist],
                                  "turn": [float(a) for a in turn], "valid": True}
        else:
            entry = dict(last) if last is not None else {"dist": [INF] * N_DIRECTIONS, "turn": list(FREE_TURN)}
            entry.update(stamp=stamp, valid=False)
            self.latest[topic] = entry
        return True

    def fresh(self, topic, now):
        entry = self.latest.get(topic)
        return (entry is not None and entry["valid"] and
                -self.future_tolerance <= now - entry["stamp"] <= self.timeout)

    def evaluate(self, now):
        """(dist, turn, sensors_ok, problems): nearest obstacle per direction, turn clearance, all required
        sensors current, and human-readable problems."""
        dist = [INF] * N_DIRECTIONS
        turn = list(FREE_TURN)
        sensors_ok = True
        problems = []

        def take(entry):
            for k in range(N_DIRECTIONS):
                dist[k] = min(dist[k], entry["dist"][k])
            for k in range(2):
                turn[k] = min(turn[k], entry["turn"][k])

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
                turn = [0.0, 0.0]
                problems.append("required %s: %s" % (topic, age))
            elif entry is not None:
                take(entry)
                problems.append("optional %s: %s, keeping its last obstacles" % (topic, age))
        return dist, turn, sensors_ok, problems


def stopping_distance(speed, dec, latency, margin):
    """Distance the base needs to stop from speed [m/s]: latency [s] at speed, braking at dec, plus margin.
    The same formula gives the angle to stop a turn from a yaw rate [rad/s] with dec [rad/s^2]."""
    speed = abs(speed)
    return speed * latency + speed * speed / (2 * max(dec, 1e-3)) + margin


def travel_directions(vx, vy):
    """Corridors a translation (vx, vy) moves into: the one it points along, or the two bracketing it."""
    a = math.atan2(vy, vx) % (2 * math.pi) / (2 * math.pi) * N_DIRECTIONS
    k = int(math.floor(a + 1e-9)) % N_DIRECTIONS
    return [k] if abs(a - round(a)) < 1e-6 else [k, (k + 1) % N_DIRECTIONS]


def gate_command(cmd, detector_fresh, sensors_ok, dist, turn, avoid_active, speed, yaw_rate=0.0, dec=0.5,
                 latency=0.3, margin=0.3, dec_theta=0.5, margin_theta=0.15):
    """cmd_vel gate policy. cmd: (vx, vy, wz); dist: corridor distances from the body; turn: turn
    clearance [counterclockwise, clockwise]; speed, yaw_rate: the robot's current motion (the faster of it
    and the command counts). Returns ((vx, vy, wz), reason); reason is None when the command passes.

    - non-finite command                         -> stop
    - detector silent or a required sensor stale -> stop (in every state)
    - supervisor on a detour (AVOID, fresh)      -> pass: the avoid controller checks its own motion
    - otherwise: a translation stops when an obstacle in the corridor(s) it moves into is closer than the
      stopping distance; a turn in place stops when the footprint would touch an obstacle within the angle
      the base needs to stop turning (latency, braking at dec_theta, plus margin_theta).
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
    if abs(wz) > 1e-6:
        need = stopping_distance(max(abs(wz), abs(yaw_rate)), dec_theta, latency, margin_theta)
        k = 0 if wz > 0 else 1
        if turn[k] < need:
            return (0.0, 0.0, 0.0), "obstacle %.0f deg into the %s turn (needs %.0f deg)" % (
                math.degrees(turn[k]), ("counterclockwise", "clockwise")[k], math.degrees(need))
    return (0.0, 0.0, wz), None
