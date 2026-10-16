r"""Part 7 test: real pick and place of a tote with the youBot arm (physics grasp, nothing attached).

At the Magasin table: dock (ArUco + lidar), grasp the tote's handle fin, put the tote on the
robot's platform, fold the arm, drive 1 m back and forth with the tote on board, dock again and
put the tote back on the table. The tote's TRUE position is checked after each step.
Saves media\pick_place_*.png (robot camera views).

Needs scenes\atelier_cablage.ttt open. Run (PowerShell, from the project folder):
  .\.venv\Scripts\python.exe tests\test_pick_place.py
"""

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import layout
from arm import YouBotArm
from docking import Docker
from equip_youbot import place_youbot
from sensors import Sensors
from youbot import YouBot
from transfer import CARRY, Transfer

STATION = "magasin"


def main():
    sim = RemoteAPIClient().require("sim")
    robot, sensors = YouBot(sim), Sensors(sim)
    tote = sim.getObject(f"/Atelier/bac_{STATION}")
    ref = sim.getObject("/youBot/youBot_ref")
    sim.setStepping(True)
    sim.startSimulation()
    checks = []

    def tote_in_robot_frame():
        mi = sim.getMatrixInverse(sim.getObjectMatrix(ref, sim.handle_world))
        p = sim.multiplyVector(mi, sim.getObjectPosition(tote, sim.handle_world))
        return p[1], -p[0], p[2]

    def check(name, ok, info=""):
        checks.append(ok)
        print(f"  {'OK ' if ok else 'BAD'} {name} {info}")

    try:
        st = layout.STATIONS[STATION]
        x, y, h = st["pose"]
        place_youbot(sim, robot.model, x + 0.05, y - 0.04, h + math.radians(4))
        for _ in range(5):
            sim.step()
        t0 = sim.getSimulationTime()
        tr = Transfer(sim, robot, sensors, Docker(sim, robot, sensors), YouBotArm(sim))
        z0 = sim.getObjectPosition(tote, sim.handle_world)[2]

        r = tr.load(st["marker_id"])
        f, l, z = tote_in_robot_frame()
        check("tote loaded on the platform", r["ok"] and abs(f - CARRY[0]) < 0.04 and abs(l - CARRY[1]) < 0.04 and z < 0.3,
              f"({r['info']}; tote at fwd {f:.3f} left {l:+.3f} up {z:.3f})")

        robot.move(-0.2, 0.0, 0.0)          # drive with the tote on board: 1 m back, 1 m forward
        for _ in range(100):
            sim.step()
        robot.move(0.2, 0.0, 0.0)
        for _ in range(100):
            sim.step()
        robot.stop()
        for _ in range(10):
            sim.step()
        f2, l2, z2 = tote_in_robot_frame()
        check("tote still on board after driving", math.dist((f, l, z), (f2, l2, z2)) < 0.02,
              f"(moved {math.dist((f, l, z), (f2, l2, z2)) * 100:.1f} cm on the platform)")

        r = tr.unload(st["marker_id"])
        p = sim.getObjectPosition(tote, sim.handle_world)
        check("tote back on the table", r["ok"] and abs(p[2] - z0) < 0.01 and abs(p[0] - x) < 0.04,
              f"({r['info']}; tote at x {p[0]:.3f} (station {x}), z {p[2]:.3f} (start {z0:.3f}))")
        print(f"total {sim.getSimulationTime() - t0:.1f} s simulated")
    finally:
        robot.stop()
        sim.stopSimulation()
    print("RESULT:", "PASSED" if checks and all(checks) else "FAILED")


if __name__ == "__main__":
    main()
