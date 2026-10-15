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


def draw_scan(points, sonars=None, size=500, scale=100):
    """Top view centred on the robot (forward = up), 1 px = 1/scale m. Returns a BGR image."""
    import cv2
    img = np.full((size, size, 3), 255, np.uint8)
    c = size // 2
    for r in range(1, int(size / scale / 2) + 1):                 # 1 m circles
        cv2.circle(img, (c, c), r * scale, (225, 225, 225), 1)
    cv2.rectangle(img, (c - int(0.2 * scale), c - int(0.29 * scale)), (c + int(0.2 * scale), c + int(0.29 * scale)), (120, 120, 120), 2)
    cv2.line(img, (c, c), (c, c - int(0.4 * scale)), (120, 120, 120), 2)                # forward arrow
    for f, l in points:
        u, v = int(c - l * scale), int(c - f * scale)
        if 0 <= u < size and 0 <= v < size:
            cv2.circle(img, (u, v), 2, (40, 40, 220), -1)
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
