r"""Precise docking at a station's transfer table: camera (ArUco) first, then lidar.

    dock = Docker(sim, robot, sensors)
    result = dock.dock(marker_id)        # robot ends square, centred, DOCK_DIST from the table face
    dock.undock()                        # back off to the navigation pose

Why two sensors: after navigation the robot is only known to ~2 cm / ~1 deg (UWB + gyro) and the
path follower stops within a few cm. The arm needs better than that.
  1. ALIGN (camera): the station's ArUco marker gives its position and orientation relative to the
     robot (src\markers.py). The mecanum base drives sideways / forwards and turns at the same time
     to a point ALIGN_DIST in front of the marker, square to it. This also CHECKS the station: the
     marker id must be the expected one.
  2. APPROACH (lidar): at the end the camera is too close to see the whole marker, so the last
     ~20 cm use the lidar: the table's front face is a straight line in the scan; fitting a line
     gives the distance (and the angle) to the face. Drive straight in until DOCK_DIST.
Obstacle avoidance is off during docking (the robot moves slowly towards a known table).
"""

import math

import numpy as np

from equip_youbot import CAMERA_POS
from markers import MarkerDetector

ALIGN_DIST = CAMERA_POS[0] + 0.30   # robot centre -> marker at the end of ALIGN (camera 0.30 m away)
DOCK_DIST = 0.40                    # robot centre -> table face when docked (front of the lidar ~4 cm away)
LIDAR_ROI = 0.25                    # m: lidar points within +-LIDAR_ROI to the side = the table face
V_ALIGN, V_APPROACH = 0.12, 0.06    # m/s
K_POS, K_YAW = 1.5, 1.5
TOL_POS, TOL_YAW = 0.004, math.radians(0.4)
SMOOTH = 0.4                        # exponential smoothing of the measurements (0..1, 1 = no smoothing)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Docker:
    def __init__(self, sim, robot, sensors, log=print):
        self.sim, self.robot, self.sensors, self.log = sim, robot, sensors, log
        self.markers = MarkerDetector(sim)

    # ------------------------------------------------------------ measurements
    def see_marker(self, marker_id):
        ms = [m for m in self.markers.detect(self.sensors.camera_image()) if m.id == marker_id]
        return ms[0] if ms else None

    def face(self):
        """Distance (at the robot's centre line) and yaw error to the flat face ahead, from the lidar.
        Line fit fwd = a + b * left over the points straight ahead. yaw > 0: turn left to be square."""
        pts = self.sensors.lidar_points()
        sel = pts[(np.abs(pts[:, 1]) < LIDAR_ROI) & (pts[:, 0] < 1.2)]
        if len(sel) < 10:
            return None, None
        b, a = np.polyfit(sel[:, 1], sel[:, 0], 1)
        return float(a), float(-math.atan(b))

    # ------------------------------------------------------------ motion
    def _clip(self, v, vmax):
        return max(-vmax, min(vmax, v))

    def dock(self, marker_id, timeout=40.0):
        sim, robot = self.sim, self.robot
        t0 = sim.getSimulationTime()
        res = dict(ok=False, marker=None, time=0.0, phase="ALIGN")
        f = l = y = None
        settled = 0
        # ---- 1. ALIGN with the camera
        lost_since = None
        while sim.getSimulationTime() - t0 < timeout:
            m = self.see_marker(marker_id)
            if m is None:
                robot.stop()
                lost_since = lost_since or sim.getSimulationTime()
                if sim.getSimulationTime() - lost_since > 2.0:
                    self.log(f"  docking: marker {marker_id} not seen -> abort")
                    res["phase"] = "NO MARKER"
                    return self._done(res, t0)
                sim.step()
                continue
            lost_since = None
            res["marker"] = m.id
            # target = point ALIGN_DIST in front of the marker, along the marker's normal
            tf = m.fwd - ALIGN_DIST * math.cos(m.yaw)
            tl = m.left - ALIGN_DIST * math.sin(m.yaw)
            if f is None:
                f, l, y = tf, tl, m.yaw
            else:
                f, l, y = (f + SMOOTH * (tf - f), l + SMOOTH * (tl - l), y + SMOOTH * (m.yaw - y))
            if math.hypot(f, l) < TOL_POS and abs(y) < TOL_YAW:
                settled += 1
                if settled >= 5:
                    break
            else:
                settled = 0
            d = math.hypot(f, l)
            v = min(V_ALIGN, K_POS * d)
            s = v / d if d > 1e-6 else 0.0
            robot.move(f * s, l * s, self._clip(K_YAW * y, 0.4))
            sim.step()
        else:
            robot.stop()
            res["phase"] = "ALIGN TIMEOUT"
            return self._done(res, t0)
        res["align_time"] = round(sim.getSimulationTime() - t0, 1)

        # ---- 2. APPROACH with the lidar (camera too close to see the marker at the end)
        res["phase"] = "APPROACH"
        settled = 0
        while sim.getSimulationTime() - t0 < timeout:
            a, yaw = self.face()
            if a is None:
                robot.stop()
                res["phase"] = "NO FACE"
                return self._done(res, t0)
            e = a - DOCK_DIST
            if abs(e) < TOL_POS and abs(yaw) < TOL_YAW:
                settled += 1
                if settled >= 5:
                    res.update(ok=True, phase="DOCKED", face_dist=round(a, 4), face_yaw_deg=round(math.degrees(yaw), 2))
                    return self._done(res, t0)
            else:
                settled = 0
            robot.move(self._clip(K_POS * e, V_APPROACH), 0.0, self._clip(K_YAW * yaw, 0.3))
            sim.step()
        res["phase"] = "APPROACH TIMEOUT"
        return self._done(res, t0)

    def undock(self, dist=ALIGN_DIST + 0.25, timeout=15.0):
        """Back away from the table (straight, with the lidar) until the face is `dist` away."""
        t0 = self.sim.getSimulationTime()
        while self.sim.getSimulationTime() - t0 < timeout:
            a, _ = self.face()
            if a is None or a >= dist:
                break
            self.robot.move(-V_ALIGN, 0.0, 0.0)
            self.sim.step()
        self.robot.stop()

    def _done(self, res, t0):
        self.robot.stop()
        self.sim.step()
        res["time"] = round(self.sim.getSimulationTime() - t0, 1)
        return res
