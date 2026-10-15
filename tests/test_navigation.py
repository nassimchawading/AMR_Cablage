r"""Part 6 test: the youBot drives A* routes through the workshop while 3 workers walk around.

Mission: dock -> sertissage -> rebuts -> dock. For each leg: time, distance, stops, sideways
dodges, re-plans, emergency stops, minimum distance to a worker, physical collisions.
Draws the robot's true path (blue) and the workers' paths (orange) on the map:
media\navigation_test.png. The current route is also drawn live in CoppeliaSim (blue line).

Needs scenes\atelier_cablage.ttt open (robot at the dock).
Run (PowerShell, from the project folder):
  .\.venv\Scripts\python.exe tests\test_navigation.py
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))
import cv2
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import layout
from navigator import Navigator
from occupancy import OccupancyGrid
from sensors import Sensors
from youbot import YouBot

MISSION = ["sertissage", "rebuts", "dock"]


def draw(grid, nav, path):
    img = grid.image()
    for trail in nav.walker_trails.values():
        for p in trail[::4]:
            cv2.circle(img, grid.world_to_image(*p), 2, (40, 150, 245), -1)
    for p, q in zip(nav.trail, nav.trail[1:]):
        cv2.line(img, grid.world_to_image(*p), grid.world_to_image(*q), (200, 80, 30), 3)
    for name in ["dock"] + MISSION:
        u, v = grid.world_to_image(*layout.STATIONS[name]["pose"][:2])
        cv2.drawMarker(img, (u, v), (0, 0, 0), cv2.MARKER_CROSS, 16, 2)
        cv2.putText(img, name, (u + 6, v - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    cv2.putText(img, "robot (blue)   workers (orange dots)", (10, img.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
    cv2.imwrite(str(path), img)


def main():
    sim = RemoteAPIClient().require("sim")
    robot, sensors = YouBot(sim), Sensors(sim)
    grid = OccupancyGrid.from_layout()
    walkers = {w.name: sim.getObject(f"/Atelier/operateurs_mobiles/{w.name}/Bill") for w in layout.WALKERS}
    nav = Navigator(sim, robot, sensors, grid, walkers=walkers)
    sim.setStepping(True)
    sim.startSimulation()
    results = []
    try:
        for _ in range(5):
            sim.step()
        for name in MISSION:
            print(f"--> going to {name}")
            r = nav.go_to(layout.STATIONS[name]["pose"])
            results.append((name, r))
            print(f"    {'REACHED' if r['reached'] else 'NOT reached'} in {r['time']} s (sim), {r['wall_time']} s real, "
                  f"{r['distance']} m, stops {r['stops']}, escapes {r['escapes']}, re-plans {r['replans']}, "
                  f"emergency stops {r['emergency_stops']}, min dist to a worker {r['min_dist_worker']} m, "
                  f"contacts: robot moving {r['contacts_moving']} / robot stopped {r['contacts_stopped']} steps")
    finally:
        robot.stop()
        sim.stopSimulation()
        draw(grid, nav, ROOT / "media" / "navigation_test.png")
        print("Saved media\\navigation_test.png")
    ok = all(r["reached"] and r["contacts_moving"] == 0 and r["contacts_stopped"] == 0 for _, r in results)
    print("RESULT:", "PASSED" if ok and len(results) == len(MISSION) else "FAILED")


if __name__ == "__main__":
    main()
