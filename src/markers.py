r"""ArUco markers seen by the robot camera: which station, and where it is relative to the robot.

    det = MarkerDetector(sim)
    for m in det.detect(sensors.camera_image()):
        m.id, m.fwd, m.left, m.yaw, m.dist

Each station's transfer table carries a real ArUco marker (dictionary 4X4_50, 15 cm) on its face
towards the aisle. OpenCV finds the 4 corners of each marker in the image; knowing the marker size
and the camera's focal length, solvePnP gives the marker's position and orientation in the camera
frame. We then express it in the ROBOT frame (forward, left), using the camera's pose on the robot
read from the scene (so the camera's mounting tilt is taken into account automatically).

Results per marker:
  fwd, left   position of the marker centre (m), robot frame
  yaw         angle (rad) the robot must turn to face the marker squarely (0 = already square)
  dist        distance from the camera (m)
"""

import math
from dataclasses import dataclass

import cv2
import numpy as np

import layout
from equip_youbot import CAMERA_ANGLE, CAMERA_RES

W, H = CAMERA_RES
F = (W / 2) / math.tan(CAMERA_ANGLE / 2)    # focal length (px): CoppeliaSim's view angle spans the larger side
K = np.array([[F, 0, W / 2], [0, F, H / 2], [0, 0, 1]], float)
S = layout.ARUCO_SIZE
OBJ = np.array([[-S / 2, S / 2, 0], [S / 2, S / 2, 0], [S / 2, -S / 2, 0], [-S / 2, -S / 2, 0]], float)
# OpenCV camera frame (x right, y down, z forward) -> CoppeliaSim vision sensor frame (x left, y up, z forward)
CV_TO_SENSOR = np.diag([-1.0, -1.0, 1.0])


@dataclass
class Marker:
    id: int
    fwd: float
    left: float
    up: float
    yaw: float
    dist: float
    corners: np.ndarray       # 4 x 2 pixels


class MarkerDetector:
    def __init__(self, sim, robot_path="/youBot"):
        ref = sim.getObject(robot_path + "/youBot_ref")
        cam = sim.getObject(robot_path + "/camera")
        m = np.asarray(sim.getObjectMatrix(cam, ref)).reshape(3, 4)      # camera pose in youBot_ref's frame
        # youBot_ref frame -> robot frame (forward = +y, left = -x, up = +z)
        to_robot = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
        self.R = to_robot @ m[:, :3] @ CV_TO_SENSOR       # OpenCV camera frame -> robot frame
        self.t = to_robot @ m[:, 3]
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, "DICT_" + layout.ARUCO_DICT))
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(dictionary, params)

    def detect(self, img):
        corners, ids, _ = self.detector.detectMarkers(img)
        out = []
        if ids is None:
            return out
        for c, i in zip(corners, ids.ravel()):
            c = c.reshape(4, 2)
            ok, rvec, tvec = cv2.solvePnP(OBJ, c, K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            p = self.R @ tvec.ravel() + self.t                     # marker centre, robot frame
            rot, _ = cv2.Rodrigues(rvec)
            n = self.R @ rot[:, 2]                                 # marker normal (points out of its face)
            # facing the marker squarely = looking along -n: yaw error in the horizontal plane
            yaw = math.atan2(-n[1], -n[0])
            out.append(Marker(int(i), float(p[0]), float(p[1]), float(p[2]), yaw, float(np.linalg.norm(tvec)), c))
        return out

    @staticmethod
    def draw(img, markers):
        """Overlay for the live view: outline, id, distance."""
        for m in markers:
            pts = m.corners.astype(int)
            cv2.polylines(img, [pts], True, (0, 255, 0), 2)
            u, v = pts[0]
            cv2.putText(img, f"id {m.id}  {m.dist:.2f} m  {math.degrees(m.yaw):+.1f} deg", (u, v - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        return img
