"""Read the youBot's sensors from Python (they are mounted by equip_youbot.py).

    s = Sensors(sim)
    pts = s.lidar_points()      # N x 2 numpy array, in metres, robot frame: [:, 0] forward, [:, 1] left
    d = s.sonar_distances()     # {"avant": 0.65, "gauche": None, ...} (None = nothing within range)
    img = s.camera_image()      # H x W x 3 numpy array, BGR (OpenCV order), top row first

All positions are in the ROBOT frame (forward / left), centred on youBot_ref.
"""

import numpy as np

LIDAR_MAX = 5.0                  # m, range of the library fast Hokuyo
SELF_BOX = (0.36, 0.25)          # |forward| < 0.36 and |left| < 0.25: points on the robot itself


class Sensors:
    def __init__(self, sim, robot_path="/youBot"):
        self.sim = sim
        self.ref = sim.getObject(robot_path + "/youBot_ref")
        self.lidar = [sim.getObject(robot_path + "/lidar/sensor1"), sim.getObject(robot_path + "/lidar/sensor2")]
        robot = sim.getObject(robot_path)
        self.sonars = {}
        for h in sim.getObjectsInTree(robot, sim.sceneobject_proximitysensor, 0):
            alias = sim.getObjectAlias(h)
            if alias.startswith("sonar_"):
                self.sonars[alias[len("sonar_"):]] = h
        self.camera = sim.getObject(robot_path + "/camera")
        self.geometry = self._read_geometry()

    def _in_robot(self, h):
        """Mount position (forward, left) and view direction (rad, 0 = forward, + = left) of a sensor."""
        m = self.sim.getObjectMatrix(h, self.ref)
        return (m[7], -m[3]), float(np.arctan2(-m[2], m[6]))        # sensors look along their +z axis

    def _read_geometry(self):
        """Fields of view READ FROM THE SCENE (not guessed), for draw_scan():
        lidar = mount position, range and one (direction, half angle) per depth sensor;
        sonars = per sonar: mount position, direction, half angle, near / far limit of its cone."""
        sim = self.sim
        fields = []
        for s in self.lidar:
            pos, d = self._in_robot(s)
            fields.append((d, sim.getObjectFloatParam(s, sim.visionfloatparam_perspective_angle) / 2))
        lidar = dict(pos=pos, range=sim.getObjectFloatParam(self.lidar[0], sim.visionfloatparam_far_clipping),
                     fields=fields)
        sonars = {}
        for name, h in self.sonars.items():
            pos, d = self._in_robot(h)
            off, rng = sim.getFloatProperty(h, "volume_offset"), sim.getFloatProperty(h, "volume_range")
            # volume_angle[0] = FULL opening of the cone (its volume_edges reach half of it off-axis)
            sonars[name] = dict(pos=pos, dir=d, half=sim.getFloatArrayProperty(h, "volume_angle")[0] / 2,
                                near=off, far=off + rng)
        return dict(lidar=lidar, sonars=sonars)

    def lidar_points(self):
        """Obstacle points seen by the lidar, in the robot frame (forward, left)."""
        out = []
        for s in self.lidar:
            res = self.sim.readVisionSensor(s)
            if not isinstance(res, (list, tuple)) or len(res) < 3 or not res[2]:
                continue                                  # no data this step (first step; the API can also
                                                          # return a bare -1): the obstacle memory covers it
            u = np.asarray(res[2], dtype=float)
            w, h = int(u[0]), int(u[1])
            data = u[2:2 + 4 * w * h].reshape(-1, 4)     # x, y, z (sensor frame), distance
            data = data[data[:, 3] < LIDAR_MAX * 0.999]  # drop "nothing hit" rays
            if not len(data):
                continue
            m = np.asarray(self.sim.getObjectMatrix(s, self.ref)).reshape(3, 4)
            p = data[:, :3] @ m[:, :3].T + m[:, 3]        # into youBot_ref's frame
            out.append(np.column_stack((p[:, 1], -p[:, 0])))   # robot frame: forward = +y, left = -x
        if not out:
            return np.zeros((0, 2))
        pts = np.vstack(out)
        on_robot = (np.abs(pts[:, 0]) < SELF_BOX[0]) & (np.abs(pts[:, 1]) < SELF_BOX[1])
        return pts[~on_robot]

    def sonar_distances(self):
        """Distance measured by each ultrasonic sensor (m), or None if nothing is in range."""
        out = {}
        for name, h in self.sonars.items():
            res = self.sim.readProximitySensor(h)
            out[name] = round(res[1], 3) if res and res[0] > 0 else None
        return out

    def camera_image(self):
        """Current camera image as a BGR numpy array (ready for OpenCV)."""
        img, (w, h) = self.sim.getVisionSensorImg(self.camera)
        img = np.frombuffer(img, dtype=np.uint8).reshape(h, w, 3)
        return np.ascontiguousarray(img[::-1, :, ::-1])     # CoppeliaSim: bottom row first, RGB


def _sector(pt, pos, d, half, r0, r1):
    """Polygon (image px) of the ring sector between r0 and r1 around the robot-frame point pos,
    centred on direction d (rad), half opening `half`."""
    a = np.linspace(d - half, d + half, 24)
    out = [pt(pos[0] + r1 * np.cos(t), pos[1] + r1 * np.sin(t)) for t in a]
    out += [pt(pos[0] + r0 * np.cos(t), pos[1] + r0 * np.sin(t)) for t in a[::-1]]
    return np.array(out, np.int32)


def draw_scan(points, sonars=None, size=500, scale=100, geometry=None):
    """Top view centred on the robot (forward = up), 1 px = 1/scale m. Returns a BGR image.
    With `geometry` (Sensors.geometry): the lidar field is shaded and each sonar is drawn as its real
    cone, grey when empty, orange with an arc at the measured distance when it hears an echo; what is
    left white is outside every sensor (blind zones)."""
    import cv2
    img = np.full((size, size, 3), 255, np.uint8)
    c = size // 2

    def pt(f, l):
        return int(round(c - l * scale)), int(round(c - f * scale))

    if geometry:
        li = geometry["lidar"]
        shade = img.copy()
        for d, half in li["fields"]:
            cv2.fillPoly(shade, [_sector(pt, li["pos"], d, half, 0.0, li["range"])], (250, 225, 200))
        cv2.addWeighted(shade, 0.6, img, 0.4, 0, img)
        cones = img.copy()
        for name, g in geometry["sonars"].items():
            d = (sonars or {}).get(name)
            col = (120, 200, 255) if d is not None else (205, 205, 205)
            cv2.fillPoly(cones, [_sector(pt, g["pos"], g["dir"], g["half"], g["near"], g["far"])], col)
        cv2.addWeighted(cones, 0.55, img, 0.45, 0, img)
        for name, g in geometry["sonars"].items():
            d = (sonars or {}).get(name)
            poly = _sector(pt, g["pos"], g["dir"], g["half"], g["near"], g["far"])
            cv2.polylines(img, [poly], True, (0, 140, 255) if d is not None else (160, 160, 160), 1, cv2.LINE_AA)
            if d is not None:                                      # echo: somewhere on this arc
                arc = _sector(pt, g["pos"], g["dir"], g["half"], d, d)[:24]
                cv2.polylines(img, [arc], False, (0, 110, 255), 3, cv2.LINE_AA)
    for r in range(1, int(size / scale / 2) + 1):                 # 1 m circles
        cv2.circle(img, (c, c), r * scale, (225, 225, 225), 1)
    cv2.rectangle(img, (c - int(0.2 * scale), c - int(0.29 * scale)), (c + int(0.2 * scale), c + int(0.29 * scale)), (120, 120, 120), 2)
    cv2.line(img, (c, c), (c, c - int(0.4 * scale)), (120, 120, 120), 2)                # forward arrow
    for f, l in points:
        u, v = pt(f, l)
        if 0 <= u < size and 0 <= v < size:
            cv2.circle(img, (u, v), 2, (40, 40, 220), -1)
    if geometry:
        fov = np.degrees(sum(2 * h for _, h in geometry["lidar"]["fields"]))
        g = next(iter(geometry["sonars"].values()))
        cv2.putText(img, f"lidar {fov:.0f} deg (bleu), points (rouge), cercles 1 m", (8, size - 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (60, 60, 60), 1)
        cv2.putText(img, f"sonars {np.degrees(2 * g['half']):.0f} deg / {g['far']:.1f} m (gris, orange = echo)",
                    (8, size - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (60, 60, 60), 1)
        return img
    if sonars:
        dirs = {"avant": 0, "avant_gauche": 45, "gauche": 90, "arriere_gauche": 135, "arriere": 180,
                "arriere_droite": -135, "droite": -90, "avant_droite": -45}
        for name, d in sonars.items():
            if d is None or name not in dirs:
                continue
            a = np.radians(dirs[name])
            r = 0.3 + d                                            # sensors sit ~0.3 m from the centre
            cv2.circle(img, (int(c - np.sin(a) * r * scale), int(c - np.cos(a) * r * scale)), 7, (0, 160, 255), 2)
    cv2.putText(img, "lidar (red)  sonar (orange)  1 m circles", (8, size - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (60, 60, 60), 1)
    return img
