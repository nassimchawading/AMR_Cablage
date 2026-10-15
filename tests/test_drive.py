r"""Part 2 test: does robot.move() produce the requested motion?

Drives each basic motion for 2 s (forward, sideways left, turn left) and compares what the
robot really did (true pose) with what was asked. Needs scenes\test_youbot.ttt open.

Run (PowerShell, from the project folder):
  .\.venv\Scripts\python.exe tests\test_drive.py
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from youbot import YouBot

DURATION = 2.0
TESTS = [("forward 0.2 m/s", dict(forward=0.2)),
         ("left 0.2 m/s", dict(left=0.2)),
         ("turn 0.5 rad/s", dict(turn=0.5))]


def run_for(sim, seconds):
    t0 = sim.getSimulationTime()
    while sim.getSimulationTime() - t0 < seconds:
        sim.step()


def main():
    sim = RemoteAPIClient().require("sim")
    robot = YouBot(sim)
    sim.setStepping(True)
    sim.startSimulation()
    try:
        run_for(sim, 0.5)                                  # let the robot settle
        for name, cmd in TESTS:
            x0, y0, h0 = robot.true_pose()
            robot.move(**cmd)
            run_for(sim, DURATION)
            robot.stop()
            run_for(sim, 0.5)                              # coast to a stop
            x1, y1, h1 = robot.true_pose()
            dx, dy = x1 - x0, y1 - y0                      # displacement, expressed in the robot's start frame
            fwd = dx * math.cos(h0) + dy * math.sin(h0)
            left = -dx * math.sin(h0) + dy * math.cos(h0)
            turn = math.atan2(math.sin(h1 - h0), math.cos(h1 - h0))
            want = {k: v * DURATION for k, v in cmd.items()}
            print(f"{name:16s} asked {want}  ->  got forward {fwd:+.3f} m, left {left:+.3f} m, turn {turn:+.3f} rad")
    finally:
        robot.stop()
        sim.stopSimulation()


if __name__ == "__main__":
    main()
