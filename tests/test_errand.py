r"""Part 7 integration test: a complete transport errand, with the workers walking around.

dock -> Magasin: navigate (A* + avoidance), dock (ArUco + lidar), load the reel tote with the arm
     -> Coupe: navigate with the tote on board, dock, unload it on the Coupe table
     -> back to the dock.
Meanwhile the QC cell (camera + UR5) sorts the harness totes in the background (src\factory.py).
Check: the reel tote ends on the Coupe table, upright; no contact with a worker; all QC totes sorted.

Needs scenes\atelier_cablage.ttt open. Run (PowerShell, from the project folder; ~10 min real):
  .\.venv\Scripts\python.exe -u tests\test_errand.py
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import layout
from arm import YouBotArm
from docking import Docker
from factory import Factory
from navigator import Navigator
from occupancy import OccupancyGrid
from sensors import Sensors
from transfer import Transfer
from youbot import YouBot


def main():
    sim = Factory(RemoteAPIClient())        # the QC cell (camera + UR5) sorts totes in the background
    robot, sensors = YouBot(sim), Sensors(sim)
    walkers = {w.name: sim.getObject(f"/Atelier/operateurs_mobiles/{w.name}/Bill") for w in layout.WALKERS}
    tote = sim.getObject("/Atelier/bac_magasin")
    sim.setStepping(True)
    sim.startSimulation()
    ok = True
    try:
        for _ in range(5):
            sim.step()
        nav = Navigator(sim, robot, sensors, OccupancyGrid.from_layout(), walkers=walkers)
        tr = Transfer(sim, robot, sensors, Docker(sim, robot, sensors), YouBotArm(sim))
        t0 = sim.getSimulationTime()
        for step, station in (("go", "magasin"), ("load", "magasin"), ("go", "coupe"), ("unload", "coupe"), ("go", "dock")):
            st = layout.STATIONS[station]
            if step == "go":
                r = nav.go_to(st["pose"])
                good = r["reached"] and r["contacts_moving"] == 0
                print(f"--> {station}: {'reached' if r['reached'] else 'NOT reached'} in {r['time']} s, {r['distance']} m, "
                      f"stops {r['stops']}, re-plans {r['replans']}, min dist to a worker {r['min_dist_worker']} m, "
                      f"contacts {r['contacts_moving']}/{r['contacts_stopped']}")
            else:
                r = getattr(tr, step)(st["marker_id"])
                good = r["ok"]
                print(f"--> {step} at {station}: {'OK' if good else 'FAILED'} in {r['time']} s ({r['info']})")
            ok &= good
            if not good:
                break
        x, y, z = sim.getObjectPosition(tote, sim.handle_world)
        upright = sim.getObjectMatrix(tote, sim.handle_world)[10] > 0.98
        cx = layout.STATIONS["coupe"]["pose"][0]
        on = abs(x - cx) < 0.3 and abs(y - layout.DROP_TOP_Y) < 0.25 and abs(z - 0.355) < 0.02
        print(f"reel tote at ({x:.3f}, {y:.3f}, {z:.3f}): {'on the Coupe table' if on else 'NOT on the Coupe table'}"
              f"{'' if upright else ', TIPPED OVER'};  errand time {sim.getSimulationTime() - t0:.1f} s simulated")
        ok &= on and upright
        qc = sim.qc.stats
        # true positions, not the cell's own word (a dropped tote was once reported as "placed")
        on_tables = 0
        for i, (_, good_tag) in enumerate(layout.QC_TOTES):
            h = sim.getObject("/Atelier/bac_faisceau" + (f"_{i}" if i else ""))
            x, y, z = sim.getObjectPosition(h, sim.handle_world)
            table_x = layout.TEST_DROP_X if good_tag else layout.REBUTS_X
            on_tables += abs(x - table_x) < 0.4 and abs(y - layout.DROP_BOT_Y) < 0.25 and abs(z - 0.355) < 0.02
        print(f"QC cell meanwhile: inspected {qc['inspected']} totes, {on_tables}/{len(layout.QC_TOTES)} really on "
              f"the right table (drops {qc['dropped']}, suction retries {qc['failed']})")
        ok &= on_tables == len(layout.QC_TOTES)
    finally:
        robot.stop()
        sim.stopSimulation()
    print("RESULT:", "PASSED" if ok else "FAILED")


if __name__ == "__main__":
    main()
