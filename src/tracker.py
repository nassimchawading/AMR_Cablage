r"""Tracking of the unexpected obstacles seen by the lidar (plan B1 + B2).

    tracker = Tracker()
    tracker.update(points, now)        # every step: world points NOT explained by the map (lidar only)
    for t in tracker.confirmed():      # objects seen several times
        t.pos, t.vel, t.speed, t.status, t.predict(1.5)

B1 - clusters: the lidar gives single points; points closer than CLUSTER_GAP to each other belong to
     the same object (single-linkage). A walking person is seen as one or two legs, CLUSTER_GAP joins
     them. Each cluster gives a centre (mean of its points) and a size.
B2 - tracks: one Kalman filter per object (OpenCV cv2.KalmanFilter), constant-velocity model:
     state (x, y, vx, vy), measurement (x, y) = the cluster centre. Every step each track is
     predicted forward, then the clusters are associated with the tracks by nearest neighbour
     (closest pairs first, within GATE of the prediction). A cluster nobody claims starts a new track;
     a track not seen for LOST_AFTER s is dropped (in between it keeps being predicted, so a person
     hidden for a moment, or leaving the lidar field, is not forgotten at once); two tracks closer
     than DUPLICATE are one object: the younger is dropped.
     Status: "new" until CONFIRM_HITS detections; then "moving" while its speed is above STILL_SPEED,
     "static" once it stayed below STILL_SPEED for STATIC_TIME s ("new" in between).
The sonars are NOT used here: a sonar gives a distance inside a 30 deg cone, not a position.
"""

import math

import cv2
import numpy as np

CLUSTER_GAP = 0.6       # m: points closer than this = same object. The lidar (13 cm high) sees LEGS:
                        # mid-stride the feet are ~0.5 m apart; with 0.35 each leg became an object of its
                        # own (one standing still, the other at twice the walking speed)
MIN_POINTS = 2          # smaller clusters are noise
GATE = 0.8              # m: a cluster farther than this from a track's prediction cannot be that track
DUPLICATE = 0.5         # m: two tracks this close are the same object: the younger one is dropped
LOST_AFTER = 1.5        # s without detection -> the track is dropped (same as the navigator's MEMORY)
CONFIRM_HITS = 4        # detections before a track is trusted
ACCEL_SIGMA = 1.0       # m/s2: how fast a person can change speed (Kalman process noise)
MEAS_SIGMA = 0.12       # m: jitter of the cluster centre (swinging legs, partial view)
STILL_SPEED = 0.15      # m/s: slower than this = not moving
STATIC_TIME = 2.0       # s not moving -> "static"
MOVED_DIST = 0.6        # m from where it was confirmed: the track "moved" (a pausing person, not a box;
                        # a box's visible centre shifts by at most ~0.3 m as other faces come into view)


def cluster(points):
    """Single-linkage clusters of N x 2 points: list of (centre, size, n_points)."""
    n = len(points)
    if n == 0:
        return []
    d = np.hypot(points[:, None, 0] - points[None, :, 0], points[:, None, 1] - points[None, :, 1])
    near = d < CLUSTER_GAP
    labels = np.arange(n)
    while True:                         # propagate the smallest label through the neighbours
        new = np.min(np.where(near, labels[None, :], n), axis=1)
        if np.array_equal(new, labels):
            break
        labels = new
    out = []
    for lab in np.unique(labels):
        p = points[labels == lab]
        if len(p) >= MIN_POINTS:
            size = float(np.max(np.hypot(*(p - p.mean(axis=0)).T))) * 2
            out.append((p.mean(axis=0), size, len(p)))
    return out


class Track:
    _next_id = 1

    def __init__(self, centre, size, now):
        self.id = Track._next_id
        Track._next_id += 1
        kf = cv2.KalmanFilter(4, 2)
        kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * MEAS_SIGMA ** 2
        kf.statePost = np.array([[centre[0]], [centre[1]], [0.0], [0.0]], np.float32)
        kf.errorCovPost = np.diag([MEAS_SIGMA ** 2, MEAS_SIGMA ** 2, 1.0, 1.0]).astype(np.float32)
        self.kf = kf
        self.size = size
        self.hits = 1
        self.created = self.last_seen = self.t = now
        self.still_since = now
        self.moved = False                  # ever seen walking (see MOVED_DIST)
        self.first_pos = None               # where it was when confirmed
        self.history = [tuple(centre)]

    def predict_step(self, now):
        dt = max(now - self.t, 1e-3)
        self.t = now
        kf = self.kf
        kf.transitionMatrix = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
        # white-acceleration model: the speed may change by ~ACCEL_SIGMA * dt per step
        g = np.array([[dt * dt / 2, 0], [0, dt * dt / 2], [dt, 0], [0, dt]], np.float32)
        kf.processNoiseCov = (g @ g.T) * ACCEL_SIGMA ** 2
        kf.predict()
        # without a detection the prediction becomes the state (correct() is not called)
        kf.statePost = kf.statePre.copy()
        kf.errorCovPost = kf.errorCovPre.copy()

    def correct(self, centre, size, now):
        self.kf.correct(np.array([[centre[0]], [centre[1]]], np.float32))
        self.size = 0.8 * self.size + 0.2 * size
        self.hits += 1
        self.last_seen = now
        # (velocity only trusted once confirmed: the first estimates are still settling, and they kept
        # a box that never moved from becoming "static")
        if self.hits >= CONFIRM_HITS and self.speed > STILL_SPEED:
            self.still_since = now
        # a person who stops will walk on; a box never moved. Judged on the DISTANCE really travelled
        # since the track was confirmed, not on speed: seen from the moving robot (first from ~5 m, half
        # hidden), a box shows other faces and its centre seems to drift, which looked like a speed and
        # cost the stray boxes their quick re-plan
        if self.hits == CONFIRM_HITS:
            self.first_pos = self.pos
        if self.first_pos is not None and math.dist(self.pos, self.first_pos) > MOVED_DIST:
            self.moved = True
        self.history.append(tuple(self.pos))
        del self.history[:-60]

    @property
    def pos(self):
        s = self.kf.statePost
        return np.array([float(s[0, 0]), float(s[1, 0])])

    @property
    def vel(self):
        s = self.kf.statePost
        return np.array([float(s[2, 0]), float(s[3, 0])])

    @property
    def speed(self):
        return float(np.hypot(*self.vel))

    @property
    def status(self):
        # "moving" as soon as the track is confirmed and moves: a worker stepping out from behind a
        # machine 1 m from the robot's route reaches it in ~1.4 s - waiting STATIC_TIME first meant no
        # prediction at all for him. Only "static" needs STATIC_TIME of evidence.
        if self.hits < CONFIRM_HITS:
            return "new"
        if self.t - self.still_since >= STATIC_TIME:
            return "static"
        return "moving" if self.speed > STILL_SPEED else "new"

    def predict(self, seconds):
        """Where the object will be in `seconds` if it keeps its velocity (static: where it is)."""
        return self.pos if self.status == "static" else self.pos + self.vel * seconds


class Tracker:
    def __init__(self):
        self.tracks = []
        self.clusters = []

    def update(self, points, now):
        """points: N x 2 world points of unexpected lidar detections at time `now`."""
        for t in self.tracks:
            t.predict_step(now)
        self.clusters = cluster(np.asarray(points, float).reshape(-1, 2))
        pairs = sorted((math.dist(t.pos, c[0]), i, j) for i, t in enumerate(self.tracks)
                       for j, c in enumerate(self.clusters))
        used_t, used_c = set(), set()
        for d, i, j in pairs:                       # nearest neighbour, closest pairs first
            if d > GATE:
                break
            if i in used_t or j in used_c:
                continue
            used_t.add(i)
            used_c.add(j)
            self.tracks[i].correct(self.clusters[j][0], self.clusters[j][1], now)
        for j, (centre, size, _) in enumerate(self.clusters):
            if j not in used_c:
                self.tracks.append(Track(centre, size, now))
        keep = []
        for t in sorted(self.tracks, key=lambda t: -t.hits):     # oldest (most detections) first
            if now - t.last_seen <= LOST_AFTER and all(math.dist(t.pos, k.pos) > DUPLICATE for k in keep):
                keep.append(t)
        self.tracks = keep

    def confirmed(self):
        return [t for t in self.tracks if t.hits >= CONFIRM_HITS]


def draw_tracks(img, tracks, to_px, ahead=1.5, px_per_m=40):
    """Draw tracks on a map image: to_px(x, y) -> (u, v) pixel. Moving = orange with a velocity
    arrow and its predicted position `ahead` s later; static = grey; new = thin outline."""
    for t in tracks:
        c = to_px(*t.pos)
        st = t.status
        col = {"moving": (0, 140, 255), "static": (110, 110, 110), "new": (190, 190, 190)}[st]
        r = max(4, int(px_per_m * max(t.size, 0.3) / 2))
        if len(t.history) > 1:
            cv2.polylines(img, [np.array([to_px(*p) for p in t.history], np.int32)], False, col, 1, cv2.LINE_AA)
        cv2.circle(img, c, r, col, 2 if st != "new" else 1, cv2.LINE_AA)
        if st == "moving":
            p = to_px(*t.predict(ahead))
            cv2.arrowedLine(img, c, p, col, 2, cv2.LINE_AA, tipLength=0.25)
            cv2.circle(img, p, r, col, 1, cv2.LINE_AA)
        if st != "new":
            cv2.putText(img, f"{t.id} {t.speed:.1f}m/s" if st == "moving" else f"{t.id} fixe",
                        (c[0] + r + 2, c[1] - r), cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1, cv2.LINE_AA)
