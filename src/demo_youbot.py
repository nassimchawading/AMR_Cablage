r"""youBot demo: everything the robot can do, in the demo arena (scenes\test_youbot.ttt).

  1. CAPTEURS      lidar, sonar ring and camera read the 3 known test obstacles
  2. MECANUM       1 m square by pure strafing (heading kept), then a full turn on the spot
  3. PLANIFICATION A* on the arena map, path through the chicane to POSTE A
  4. CHARGEMENT    ArUco + lidar docking at POSTE A, the arm loads the tote on the platform
  5. TRANSPORT     to POSTE B: a worker crosses the route and a box that is NOT on the map blocks
                   it (stop, wait, re-plan); docking, the arm unloads the tote
  6. RECHARGE      back to the charging dock, docking on its marker

Live window (2 x 2): robot camera with the detected ArUco markers | lidar + sonars around the robot
                     map: planned route, trail, unexpected obstacles | chapter, pose, events
Press q in the window (or Ctrl+C in PowerShell) to stop.

Needs scenes\test_youbot.ttt open in CoppeliaSim, simulation stopped (the program starts it).
Run (PowerShell, from the project folder; ~5 min real):
  .\.venv\Scripts\python.exe -u src\demo_youbot.py
  .\.venv\Scripts\python.exe -u src\demo_youbot.py --video     (also records media\demo_youbot.mp4)
  --no-predict: navigation WITHOUT the tracker's predictions (behaviour before plan B4, to compare)
"""

import math
import sys
import time
from pathlib import Path

import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import demo_layout as demo
from arm import YouBotArm
from docking import Docker
from live_view import LiveView
from navigator import Navigator
from occupancy import OccupancyGrid
from sensors import Sensors
from transfer import CARRY, Transfer
from youbot import YouBot

ROOT = Path(__file__).resolve().parent.parent
VIDEO = ROOT / "media" / "demo_youbot.mp4"
MAP_X, MAP_Y = (-0.3, 10.3), (-0.3, 8.3)


def wait(sim, seconds):
    t0 = sim.getSimulationTime()
    while sim.getSimulationTime() - t0 < seconds:
        sim.step()


def chapter(view, text):
    view.chapter = text
    view.log(f"===== {text}")


def sensors_check(sim, view, sensors):
    pts = sensors.lidar_points()
    a = np.degrees(np.arctan2(pts[:, 1], pts[:, 0])) if len(pts) else np.zeros(0)

    def nearest(lo, hi):
        sel = pts[(a >= lo) & (a <= hi)]
        return float(np.min(np.hypot(sel[:, 0], sel[:, 1]))) if len(sel) else float("nan")

    son = sensors.sonar_distances()
    view.log(f"lidar {len(pts)} points: boite devant {nearest(-10, 10):.2f} m (attendu 0.95), "
             f"poteau a gauche {nearest(80, 100):.2f} m (0.64)")
    view.log(f"sonars: avant {son.get('avant')}  gauche {son.get('gauche')}  arriere {son.get('arriere')} (0.65/0.42/0.65)")
    img = sensors.camera_image()
    view.log(f"camera {img.shape[1]}x{img.shape[0]} px")
    return abs(nearest(-10, 10) - 0.95) < 0.05


def mecanum(sim, view, robot):
    cx, cy, side = demo.MECANUM_SQUARE
    heading = demo.START[2]
    ok = True
    for i, (tx, ty) in enumerate(((cx, cy), (cx + side, cy), (cx + side, cy + side), (cx, cy + side), (cx, cy))):
        t0 = sim.getSimulationTime()
        while not robot.go_to_step((tx, ty, heading)):
            if sim.getSimulationTime() - t0 > 25:
                ok = False
                break
            sim.step()
        xt, yt, _ = robot.true_pose()
        view.log(f"{'depart' if i == 0 else f'coin {i}'}: ({tx:.1f}, {ty:.1f}) atteint, erreur reelle "
                 f"{math.hypot(tx - xt, ty - yt) * 100:.1f} cm, cap garde")
    view.log("tour complet sur place")
    h0 = robot.true_pose()[2]
    turned, prev = 0.0, h0
    robot.move(turn=0.8)
    while turned < 2 * math.pi - 0.15:
        sim.step()
        h = robot.true_pose()[2]
        turned += math.atan2(math.sin(h - prev), math.cos(h - prev))
        prev = h
    t0 = sim.getSimulationTime()
    while not robot.go_to_step((cx, cy, heading)) and sim.getSimulationTime() - t0 < 10:
        sim.step()
    robot.stop()
    return ok


def tote_on_robot(sim, view, robot, tote, when):
    """Where the tote sits on the robot (robot frame) compared with transfer.CARRY, where the arm takes it."""
    x, y, h = robot.true_pose()
    tx, ty, tz = sim.getObjectPosition(tote, sim.handle_world)
    f = (tx - x) * math.cos(h) + (ty - y) * math.sin(h)
    l = -(tx - x) * math.sin(h) + (ty - y) * math.cos(h)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, sim.getObjectMatrix(tote, sim.handle_world)[10]))))
    view.log(f"bac {when}: avant {f:+.3f} gauche {l:+.3f} haut {tz:.3f} m, incline {tilt:.0f} deg "
             f"(prise prevue {CARRY[0]:+.3f} {CARRY[1]:+.3f} {CARRY[2]:.3f})")


def nav_leg(view, nav, key):
    r = nav.go_to(demo.STATIONS[key]["pose"])
    view.log(f"--> {demo.STATIONS[key]['label']}: {'atteint' if r['reached'] else 'NON atteint'} en {r['time']} s, "
             f"{r['distance']} m, arrets {r['stops']}, re-planifications {r['replans']}, contacts {r['contacts_moving']}")
    return r


def main():
    view = LiveView(RemoteAPIClient().require("sim"), title="demo youBot", mosaic=True,
                    record=VIDEO if "--video" in sys.argv else None, map_window=(MAP_X, MAP_Y),
                    autosave="demo_live")
    sim = view
    if sim.getSimulationState() != sim.simulation_stopped:
        print("STOP: stop the simulation first (square Stop button), then run again.")
        return 1
    try:
        tote = sim.getObject("/Demo/" + demo.TOTE_NAME)
    except Exception:
        print("STOP: open scenes\\test_youbot.ttt (built by src\\build_test_youbot.py) in CoppeliaSim first.")
        return 1
    robot, sensors = YouBot(sim), Sensors(sim)
    walker = sim.getObject(f"/Demo/operateurs_mobiles/{demo.WALKERS[0].name}/Bill")
    grid = OccupancyGrid.from_layout(demo.PARTS, demo.MODELS)          # the stray box is NOT in it
    sim.setStepping(True)
    sim.startSimulation()
    results = {}
    wall0 = time.perf_counter()
    try:
        for _ in range(5):
            sim._sim.step()
        nav = Navigator(sim, robot, sensors, grid, walkers={demo.WALKERS[0].name: walker}, log=view.log,
                        predict="--no-predict" not in sys.argv)
        view.setup(robot, sensors, grid, nav=nav, walkers={demo.WALKERS[0].name: walker})
        docker = Docker(sim, robot, sensors, log=view.log)
        tr = Transfer(sim, robot, sensors, docker, YouBotArm(sim, log=view.log, world="/Demo"), log=view.log)

        chapter(view, "1/6 CAPTEURS: lidar, sonars, camera")
        wait(sim, 1.0)
        results["capteurs"] = sensors_check(sim, view, sensors)
        wait(sim, 2.0)

        chapter(view, "2/6 MECANUM: carre en crabe + tour sur place")
        results["mecanum"] = mecanum(sim, view, robot)

        chapter(view, "3/6 PLANIFICATION A*: chicane -> POSTE A")
        results["vers poste A"] = nav_leg(view, nav, "poste_a")["reached"]

        chapter(view, "4/6 ACCOSTAGE ArUco + lidar, CHARGEMENT au bras")
        r = tr.load(demo.STATIONS["poste_a"]["marker_id"])
        view.log(f"--> chargement: {'OK' if r['ok'] else 'ECHEC'} en {r['time']} s ({r['info']})")
        results["chargement"] = r["ok"]
        tote_on_robot(sim, view, robot, tote, "apres chargement")

        chapter(view, "5/6 TRANSPORT: operateur + carton imprevu -> POSTE B")
        r = nav_leg(view, nav, "poste_b")
        results["vers poste B"] = r["reached"] and r["contacts_moving"] == 0
        tote_on_robot(sim, view, robot, tote, "a l'arrivee")
        r = tr.unload(demo.STATIONS["poste_b"]["marker_id"])
        view.log(f"--> dechargement: {'OK' if r['ok'] else 'ECHEC'} en {r['time']} s ({r['info']})")
        results["dechargement"] = r["ok"]

        chapter(view, "6/6 RECHARGE: retour au dock")
        results["vers dock"] = nav_leg(view, nav, "dock")["reached"]
        r = docker.dock(demo.STATIONS["dock"]["marker_id"])
        view.log(f"--> accostage au dock: {r['phase']} en {r['time']} s")
        results["accostage dock"] = r["ok"]

        x, y, z = sim.getObjectPosition(tote, sim.handle_world)
        bx = demo.STATIONS["poste_b"]["pose"][0]
        upright = sim.getObjectMatrix(tote, sim.handle_world)[10] > 0.98
        results["bac sur POSTE B"] = abs(x - bx) < 0.3 and abs(y - demo.TABLE_Y) < 0.3 and z > 0.3 and upright
        chapter(view, "FIN DE LA DEMO")
        wait(sim, 2.0)
    except KeyboardInterrupt:
        print("Stopped by the user.")
    finally:
        robot.stop()
        sim.stopSimulation()
        view.close()
    print(f"\nSummary ({sim.getSimulationTime():.0f} s simulated, {time.perf_counter() - wall0:.0f} s real):")
    for k, v in results.items():
        print(f"  {k:16s} {'OK' if v else 'FAILED'}")
    if view.record:
        print("Video saved:", VIDEO)
    ok = len(results) == 9 and all(results.values())
    print("RESULT:", "PASSED" if ok else "FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
