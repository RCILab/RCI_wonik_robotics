"""Obstacle detection state and cmd_vel gate policy, shared by obstacle_detector.py and cmd_vel_gate.py
(ROS-free, so the tests exercise the same code as the nodes).

Sensor validity and obstacle policy are separate:
  validity  every required sensor must deliver data whose measurement time (header stamp) is recent;
            otherwise the detector reports sensors_ok = False and the gate stops the robot in every
            state, including a running detour.
  policy    what an obstacle means depends on the route supervisor's state: on a detour (AVOID) the
            avoid controller's own footprint checks handle obstacles, anywhere else the gate stops
            translation for an obstacle in the front box and in-place turns for one within the turn
            radius.
"""
import math

REQUIRED, OPTIONAL = "required", "optional"


class SensorTracker:
    """Latest obstacle counts per sensor, judged by their measurement time.

    A frame counts only if it is a valid measurement: the caller passes valid=False for a frame with too
    few usable points (e.g. an all-NaN depth image, which a real depth camera also returns for an object
    closer than its minimum range). Such a frame makes the sensor not current at once; it does not
    clear anything.
    required sensors: missing, stale or invalid data -> sensors_ok False and obstacles assumed
        (fail-safe).
    optional sensors: used while current. When one goes stale (or invalid) after its last valid data
        showed an obstacle, that obstacle is kept (a stale camera cannot clear what only it saw) until the
        sensor delivers fresh data again; when it goes stale with nothing seen it is just reported.
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
        self.latest = {}  # topic -> {stamp, front, near, valid}; counts are from the last valid frame
        self.rejected = 0

    def update(self, topic, stamp, now, front_count, near_count, valid=True):
        """Store a sensor result. stamp: measurement time, now: current time [s]. False if rejected."""
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
            self.latest[topic] = {"stamp": stamp, "front": int(front_count), "near": int(near_count), "valid": True}
        else:
            entry = dict(last) if last is not None else {"front": 0, "near": 0}
            entry.update(stamp=stamp, valid=False)
            self.latest[topic] = entry
        return True

    def fresh(self, topic, now):
        entry = self.latest.get(topic)
        return (entry is not None and entry["valid"] and
                -self.future_tolerance <= now - entry["stamp"] <= self.timeout)

    def evaluate(self, now):
        """(detected, near, sensors_ok, problems): obstacle in the front box, obstacle within the turn
        radius, all required sensors fresh, and a list of human-readable problems."""
        detected = near = False
        sensors_ok = True
        problems = []
        for topic, kind in sorted(self.kind.items()):
            entry = self.latest.get(topic)
            if self.fresh(topic, now):
                detected |= entry["front"] >= self.min_points
                near |= entry["near"] >= self.min_points
                continue
            if entry is None:
                age = "no data"
            elif not entry["valid"]:
                age = "no valid measurement in its last frame"
            else:
                age = "%.2f s old" % (now - entry["stamp"])
            if kind == REQUIRED:
                sensors_ok = False
                detected = near = True
                problems.append("required %s: %s" % (topic, age))
            elif entry is not None:
                held = entry["front"] >= self.min_points or entry["near"] >= self.min_points
                detected |= entry["front"] >= self.min_points
                near |= entry["near"] >= self.min_points
                problems.append("optional %s: %s%s" % (topic, age, ", keeping its last obstacle" if held else ""))
        return detected, near, sensors_ok, problems


def gate_command(cmd, detector_fresh, sensors_ok, detected, near, avoid_active):
    """cmd_vel gate policy. cmd: (vx, vy, wz). Returns ((vx, vy, wz), reason); reason is None when passed.

    - non-finite command                         -> stop
    - detector silent or a required sensor stale -> stop (in every state)
    - supervisor on a detour (AVOID, fresh)      -> pass: the avoid controller checks its own motion
    - otherwise: translation stops for an obstacle in the front box; turning in place stops for an
      obstacle within the turn radius (the corners sweep a circle), and passes otherwise.
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
    translating = abs(vx) >= 0.01 or abs(vy) >= 0.01
    if translating:
        if detected:
            return (0.0, 0.0, 0.0), "obstacle ahead"
        return (vx, vy, wz), None
    if near and abs(wz) > 1e-6:
        return (0.0, 0.0, 0.0), "obstacle within the turn radius"
    if detected:
        return (0.0, 0.0, wz), None  # turning in place only, without the creeping remainder
    return (vx, vy, wz), None
