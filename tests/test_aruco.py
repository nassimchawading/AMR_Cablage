r"""Part 7 test: the robot camera finds each station's ArUco marker and measures where it is.

For every station: the robot is placed in front of the transfer table at 3 distances (camera
0.75 / 0.45 / 0.25 m from the marker) and slightly off-centre / turned, like at the end of a
navigation. The marker position measured by the camera is compared with the true one.
Saves media\aruco_<station>.png (camera view with the detection overlay).

Needs scenes\atelier_cablage.ttt open. Run (PowerShell, from the project folder):
  .\.venv\Scripts\python.exe tests\test_aruco.py
"""

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))
import cv2
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import layout
from equip_youbot import CAMERA_POS, place_youbot
from markers import MarkerDetector
from sensors import Sensors

# (camera distance to the marker face, sideways offset, heading error)
CASES = [(0.75, 0.10, math.radians(8)), (0.45, -0.05, math.radians(-5)), (0.25, 0.0, 0.0)]


def main():
    sim = RemoteAPIClient().require("sim")
    robot = sim.getObject("/youBot")
    sensors = Sensors(sim)
    sim.setStepping(True)
    sim.startSimulation()
    worst_pos = worst_yaw = 0.0
    found = total = 0
    try:
        det = MarkerDetector(sim)
        for name, st in layout.STATIONS.items():
            mid = st["marker_id"]
            marker = sim.getObject(f"/Atelier/aruco_{mid}")
            mx, my, mz = sim.getObjectPosition(marker, sim.handle_world)
            x, y, h = st["pose"]
            fx, fy = math.cos(h), math.sin(h)            # station heading = robot looks at the marker
            for k, (d, lat, dh) in enumerate(CASES):
                back = d + CAMERA_POS[0]                 # robot centre this far from the marker face
                rx = mx - fx * back - fy * lat
                ry = my - fy * back + fx * lat
                place_youbot(sim, robot, rx, ry, h + dh)
                for _ in range(3):
                    sim.step()
                img = sensors.camera_image()
                ms = [m for m in det.detect(img) if m.id == mid]
                total += 1
                # true marker position in the robot frame, and true yaw error (= -dh)
                c, s = math.cos(h + dh), math.sin(h + dh)
                tf = (mx - rx) * c + (my - ry) * s
                tl = -(mx - rx) * s + (my - ry) * c
                if not ms:
                    print(f"{name:11s} d={d:.2f}  NOT FOUND")
                    continue
                found += 1
                m = ms[0]
                ep = math.hypot(m.fwd - tf, m.left - tl)
                ey = abs(math.degrees(m.yaw + dh))
                worst_pos, worst_yaw = max(worst_pos, ep), max(worst_yaw, ey)
                print(f"{name:11s} d={d:.2f}  id {m.id}  fwd {m.fwd:.3f} (true {tf:.3f})  left {m.left:+.3f} "
                      f"(true {tl:+.3f})  up {m.up:.3f} (true {mz:.3f})  yaw {math.degrees(m.yaw):+5.1f} deg "
                      f"(true {math.degrees(-dh):+5.1f})  err {ep * 100:.1f} cm / {ey:.1f} deg")
                if k == 1:
                    cv2.imwrite(str(ROOT / "media" / f"aruco_{name}.png"), det.draw(img.copy(), ms))
    finally:
        sim.stopSimulation()
    print(f"found {found}/{total}, worst position error {worst_pos * 100:.1f} cm, worst yaw error {worst_yaw:.1f} deg")
    ok = found == total and worst_pos < 0.02 and worst_yaw < 3.0
    print("RESULT:", "PASSED" if ok else "FAILED")


if __name__ == "__main__":
    main()
