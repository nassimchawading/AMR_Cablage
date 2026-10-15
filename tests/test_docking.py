r"""Part 7 test: precise docking at every station (camera ArUco alignment + lidar approach).

The robot is placed where navigation would leave it (station pose + random error: up to 10 cm
sideways, 10 cm along, 8 deg), then docks. The final TRUE pose is compared with the ideal one
(centred on the marker, square, DOCK_DIST from the table face).

Needs scenes\atelier_cablage.ttt open. Run (PowerShell, from the project folder):
  .\.venv\Scripts\python.exe tests\test_docking.py
"""

import math
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import layout
from docking import DOCK_DIST, Docker
from equip_youbot import place_youbot
from sensors import Sensors
from youbot import YouBot

MARKER_OUT = 0.0045        # the marker sits 4.5 mm in front of the face (on its white plate)


def main():
    random.seed(7)
    sim = RemoteAPIClient().require("sim")
    robot, sensors = YouBot(sim), Sensors(sim)
    sim.setStepping(True)
    sim.startSimulation()
    worst = [0.0, 0.0, 0.0]
    ok_all = True
    try:
        docker = Docker(sim, robot, sensors)
        for name, st in layout.STATIONS.items():
            if name == "dock":
                continue            # the charging dock is a different docking (Part 8)
            mid = st["marker_id"]
            mx, my, _ = sim.getObjectPosition(sim.getObject(f"/Atelier/aruco_{mid}"), sim.handle_world)
            x, y, h = st["pose"]
            c, s = math.cos(h), math.sin(h)
            along, side, dh = random.uniform(-0.1, 0.1), random.uniform(-0.1, 0.1), math.radians(random.uniform(-8, 8))
            place_youbot(sim, robot.model, x + c * along - s * side, y + s * along + c * side, h + dh)
            for _ in range(3):
                sim.step()
            r = docker.dock(mid)
            # ideal: centred on the marker, square, DOCK_DIST from the face
            fx, fy = mx - c * MARKER_OUT, my - s * MARKER_OUT
            tx, ty, th = robot.true_pose()
            e_dist = (fx - tx) * c + (fy - ty) * s - DOCK_DIST
            e_lat = -(fx - tx) * s + (fy - ty) * c
            e_yaw = math.degrees(math.atan2(math.sin(th - h), math.cos(th - h)))
            worst = [max(worst[0], abs(e_dist)), max(worst[1], abs(e_lat)), max(worst[2], abs(e_yaw))]
            good = r["ok"] and abs(e_dist) < 0.01 and abs(e_lat) < 0.01 and abs(e_yaw) < 1.0
            ok_all &= good
            print(f"{name:11s} start error {along * 100:+5.1f} / {side * 100:+5.1f} cm, {math.degrees(dh):+4.1f} deg -> "
                  f"{r['phase']} in {r['time']} s (align {r.get('align_time')} s)  final error: distance "
                  f"{e_dist * 100:+.2f} cm, sideways {e_lat * 100:+.2f} cm, angle {e_yaw:+.2f} deg  {'OK' if good else 'BAD'}")
            docker.undock()
    finally:
        robot.stop()
        sim.stopSimulation()
    print(f"worst: distance {worst[0] * 100:.2f} cm, sideways {worst[1] * 100:.2f} cm, angle {worst[2]:.2f} deg")
    print("RESULT:", "PASSED" if ok_all else "FAILED")


if __name__ == "__main__":
    main()
