r"""Part 3 test: lidar, sonar ring and camera of the youBot.

Check mode (default): reads all sensors once and compares with the 3 known test obstacles
(box 1.2 m ahead, post 0.7 m to the left, wall 1.0 m behind), then saves
media\camera_robot.png and media\lidar_scan.png.

Live mode:  .\.venv\Scripts\python.exe tests\test_sensors.py --live
  opens 2 windows (camera + lidar/sonar view) while the robot turns slowly on the spot.
  Press q in a window (or Ctrl+C in PowerShell) to stop.

Needs scenes\test_youbot.ttt open.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))
import cv2
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from sensors import Sensors, draw_scan
from youbot import YouBot


def nearest(points, sector):
    """Closest lidar point (distance from the robot centre) in an angular sector (deg)."""
    a = np.degrees(np.arctan2(points[:, 1], points[:, 0]))
    sel = points[(a >= sector[0]) & (a <= sector[1])]
    return float(np.min(np.hypot(sel[:, 0], sel[:, 1]))) if len(sel) else None


def check(sim, sensors):
    for _ in range(5):
        sim.step()
    pts = sensors.lidar_points()
    son = sensors.sonar_distances()
    print(f"lidar: {len(pts)} points")
    print(f"  ahead (box front face at 0.95 m from centre): {nearest(pts, (-10, 10))}")
    print(f"  left  (post surface at 0.64 m from centre):    {nearest(pts, (80, 100))}")
    print("sonars:", son)
    print("  expected: avant ~0.65 (box), gauche ~0.42 (post), arriere ~0.65 (wall), others None")
    img = sensors.camera_image()
    print("camera:", img.shape)
    cv2.imwrite(str(ROOT / "media" / "camera_robot.png"), img)
    cv2.imwrite(str(ROOT / "media" / "lidar_scan.png"), draw_scan(pts, son))
    print("Saved media\\camera_robot.png and media\\lidar_scan.png")


def live(sim, sensors, robot):
    robot.move(turn=0.3)
    while True:
        sim.step()
        cv2.imshow("camera youBot", sensors.camera_image())
        cv2.imshow("lidar + sonars", draw_scan(sensors.lidar_points(), sensors.sonar_distances()))
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break


def main():
    sim = RemoteAPIClient().require("sim")
    sensors = Sensors(sim)
    robot = YouBot(sim)
    sim.setStepping(True)
    sim.startSimulation()
    try:
        if "--live" in sys.argv:
            live(sim, sensors, robot)
        else:
            check(sim, sensors)
    except KeyboardInterrupt:
        pass
    finally:
        robot.stop()
        sim.stopSimulation()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
