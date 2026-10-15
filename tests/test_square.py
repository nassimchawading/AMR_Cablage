r"""Part 2 test: drive a 1 m square using only the simulated "UWB + gyro" pose (no odometry).

The robot keeps its heading and uses its mecanum wheels: forward, slide left, backward,
slide right. At the end we compare the true final position with the start, and draw the
true path (blue) and the noisy UWB measurements (orange dots) into media\test_square.png.
Needs scenes\test_youbot.ttt open.

Run (PowerShell, from the project folder):
  .\.venv\Scripts\python.exe tests\test_square.py
"""

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from snapshot import save_png
from youbot import YouBot

SIDE = 1.0
TIMEOUT = 20.0      # s of simulated time per corner


def draw(true_path, measured, corners, path, size=420, margin=60):
    """Tiny plot without extra libraries: white image, grey target square, orange UWB dots, blue path."""
    img = bytearray([255] * size * size * 3)
    xs = [p[0] for p in true_path + corners]
    ys = [p[1] for p in true_path + corners]
    x0, y0 = min(xs) - 0.1, min(ys) - 0.1
    s = (size - 2 * margin) / max(max(xs) - x0 + 0.1, max(ys) - y0 + 0.1)

    def put(x, y, col, r=1):
        u, v = int(margin + (x - x0) * s), int(margin + (y - y0) * s)      # v grows upward
        for du in range(-r, r + 1):
            for dv in range(-r, r + 1):
                if 0 <= u + du < size and 0 <= v + dv < size:
                    i = ((v + dv) * size + (u + du)) * 3                   # save_png flips rows
                    img[i:i + 3] = bytes(col)

    for a, b in zip(corners, corners[1:] + corners[:1]):
        for k in range(101):
            put(a[0] + (b[0] - a[0]) * k / 100, a[1] + (b[1] - a[1]) * k / 100, (190, 190, 190))
    for p in measured:
        put(p[0], p[1], (245, 150, 40), 1)
    for p in true_path:
        put(p[0], p[1], (30, 80, 200), 1)
    save_png(path, bytes(img), size, size)


def main():
    sim = RemoteAPIClient().require("sim")
    robot = YouBot(sim)
    sim.setStepping(True)
    sim.startSimulation()
    true_path, measured = [], []
    try:
        for _ in range(10):
            sim.step()
        x0, y0, h0 = robot.true_pose()
        c, s = math.cos(h0), math.sin(h0)
        # corners in the robot's start frame: forward, left, back, right (heading kept constant)
        local = [(SIDE, 0.0), (SIDE, SIDE), (0.0, SIDE), (0.0, 0.0)]
        corners = [(x0 + a * c - b * s, y0 + a * s + b * c) for a, b in local]
        for i, (tx, ty) in enumerate(corners):
            t0 = sim.getSimulationTime()
            reached = False
            while sim.getSimulationTime() - t0 < TIMEOUT:
                p = robot.pose()
                measured.append(p)
                true_path.append(robot.true_pose())
                if robot.go_to_step((tx, ty, h0), pose=p):
                    reached = True
                    break
                sim.step()
            xt, yt, ht = robot.true_pose()
            print(f"corner {i + 1}: {'reached' if reached else 'TIMEOUT'} after {sim.getSimulationTime() - t0:4.1f} s, "
                  f"true error {math.hypot(tx - xt, ty - yt) * 100:4.1f} cm, heading error {math.degrees(ht - h0):+.1f} deg")
        robot.stop()
        xe, ye, he = robot.true_pose()
        print(f"back at start: true position error {math.hypot(xe - x0, ye - y0) * 100:.1f} cm")
        draw(true_path, measured, corners, ROOT / "media" / "test_square.png")
        print("Path drawn in media\\test_square.png")
    finally:
        robot.stop()
        sim.stopSimulation()


if __name__ == "__main__":
    main()
