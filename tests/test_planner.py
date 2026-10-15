r"""Part 5 test: occupancy grid + A* planner (no CoppeliaSim needed).

1. builds the map from src\layout.py and checks every station pose is reachable (not forbidden);
2. plans a route between every pair of stations, reports length and computing time;
3. draws the map with the routes from the dock to every station: media\carte_chemins.png

Optional:  --scene   also draws the dock -> sertissage route in the open workshop scene
           in CoppeliaSim (blue line on the floor, visible in the CoppeliaSim window).

Run (PowerShell, from the project folder):
  .\.venv\Scripts\python.exe tests\test_planner.py [--scene]
"""

import itertools
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))
import cv2

import layout
from occupancy import OccupancyGrid
from planner import Planner, path_length

COLORS = [(40, 40, 220), (40, 160, 40), (220, 120, 30), (180, 40, 180), (30, 160, 220), (120, 60, 20), (90, 90, 90)]


def main():
    t = time.perf_counter()
    grid = OccupancyGrid.from_layout()
    print(f"map: {grid.w} x {grid.h} cells of 5 cm, built in {time.perf_counter() - t:.2f} s, "
          f"{grid.occupied.mean() * 100:.0f} % occupied, {grid.lethal.mean() * 100:.0f} % forbidden for the robot centre")

    poses = {k: v["pose"][:2] for k, v in layout.STATIONS.items()}
    bad = [k for k, p in poses.items() if not grid.is_free(*p)]
    print("station poses in forbidden zone:", bad or "none")

    planner = Planner(grid)
    fails, times = [], []
    for a, b in itertools.combinations(poses, 2):
        t = time.perf_counter()
        path = planner.plan(poses[a], poses[b])
        times.append(time.perf_counter() - t)
        if path is None:
            fails.append((a, b))
    print(f"{len(times)} station pairs planned, failures: {fails or 'none'}, "
          f"time per route: mean {sum(times) / len(times) * 1000:.0f} ms, max {max(times) * 1000:.0f} ms")

    img = grid.image()
    for k, name in enumerate([s for s in poses if s != "dock"]):
        path = planner.plan(poses["dock"], poses[name])
        raw = len(planner.last_raw)
        print(f"  dock -> {name:11s}: {path_length(path):5.2f} m, {raw} cells -> {len(path)} waypoints")
        col = COLORS[k % len(COLORS)]
        for p, q in zip(path, path[1:]):
            cv2.line(img, grid.world_to_image(*p), grid.world_to_image(*q), col, 3)
        for p in path:
            cv2.circle(img, grid.world_to_image(*p), 5, col, -1)
    for name, (x, y) in poses.items():
        u, v = grid.world_to_image(x, y)
        cv2.drawMarker(img, (u, v), (0, 0, 0), cv2.MARKER_CROSS, 16, 2)
        cv2.putText(img, name, (u + 6, v - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    out = ROOT / "media" / "carte_chemins.png"
    cv2.imwrite(str(out), img)
    print("Saved", out)

    if "--scene" in sys.argv:
        show_in_scene(planner.plan(poses["dock"], poses["sertissage"]))


def show_in_scene(path):
    """Draw the route as a thick blue line just above the floor in the open workshop scene.
    (Drawing objects show in CoppeliaSim's window, not in vision-sensor renders, and are not saved
    with the scene: they disappear when the scene is closed.)"""
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    sim = RemoteAPIClient().require("sim")
    line = sim.addDrawingObject(sim.drawing_lines, 6, 0, -1, 1000, [0.1, 0.4, 1.0])
    for p, q in zip(path, path[1:]):
        sim.addDrawingObjectItem(line, [p[0], p[1], 0.03, q[0], q[1], 0.03])
    print("Route dock -> sertissage drawn in the CoppeliaSim window (blue line on the floor)")


if __name__ == "__main__":
    main()
