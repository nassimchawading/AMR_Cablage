r"""Part 8.1 test: the mission manager runs transport orders one after the other.

Orders:
  - one scripted order: reel tote Magasin -> Coupe (added at the start);
  - QC orders created AUTOMATICALLY while the robot works: when the UR5 puts a tote on the OK table
    -> "OK table -> Expedition", on the REBUTS table -> "REBUTS -> Assemblage".
The QC cell has one spot per table: its second OK tote waits at the conveyor's end stop until the
robot has taken the first one (that order then gets the "unblock" priority class); the UR5 then puts
it on the OK table, which creates one more order (left waiting at the end of the test).
Check (true positions): reel tote on the Coupe table, first OK tote on the Expedition table, defect
tote on the Assemblage table, second OK tote on the OK table; 3 orders DONE; no contact.

Needs scenes\atelier_cablage.ttt open. Run (PowerShell, from the project folder; ~10 min real):
  .\.venv\Scripts\python.exe -u tests\test_missions.py [--live]
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
from live_view import LiveView
from missions import MissionManager, Order
from navigator import Navigator
from occupancy import OccupancyGrid
from sensors import Sensors
from transfer import Transfer
from youbot import YouBot


def on_table(sim, path, x, y):
    px, py, pz = sim.getObjectPosition(sim.getObject(path), sim.handle_world)
    upright = sim.getObjectMatrix(sim.getObject(path), sim.handle_world)[10] > 0.98
    ok = abs(px - x) < 0.4 and abs(py - y) < 0.25 and abs(pz - 0.355) < 0.02 and upright
    return ok, f"({px:.2f}, {py:.2f}, {pz:.3f}){'' if upright else ' TIPPED OVER'}"


def main():
    sim = Factory(RemoteAPIClient())
    live = "--live" in sys.argv
    if live:
        sim = LiveView(sim, autosave="live_missions")
    if sim.getSimulationState() != sim.simulation_stopped:
        print("STOP: stop the simulation first (square Stop button), then run again.")
        return 1
    robot, sensors = YouBot(sim), Sensors(sim)
    walkers = {w.name: sim.getObject(f"/Atelier/operateurs_mobiles/{w.name}/Bill") for w in layout.WALKERS}
    sim.setStepping(True)
    sim.startSimulation()
    ok = False
    mgr = None
    try:
        for _ in range(5):
            sim.step()
        grid = OccupancyGrid.from_layout()
        nav = Navigator(sim, robot, sensors, grid, walkers=walkers)
        if live:
            sim.setup(robot, sensors, grid, nav=nav, walkers=walkers)
        tr = Transfer(sim, robot, sensors, Docker(sim, robot, sensors), YouBotArm(sim))
        mgr = MissionManager(sim, robot, nav, tr, qc=sim.qc)
        mgr.add(Order("bobines", "magasin", "coupe", source="script"))
        mgr.run(stop=lambda m: len(m.finished) >= 3 and sim.qc.stats["sorted"] >= 3, timeout=900)

        print("\nOrders:")
        print("\n".join(mgr.summary()))
        for o in mgr.waiting:
            print(f"  {o.label():45s} {o.source:6s} still waiting")
        if mgr.fault:
            print("MANAGER FAULT:", mgr.fault)
        # true positions: QC totes reach the arm in order of decreasing x on the belt
        arrival = sorted(range(len(layout.QC_TOTES)), key=lambda i: -layout.QC_TOTES[i][0])
        name = lambda i: "/Atelier/bac_faisceau" + (f"_{i}" if i else "")
        first_ok = next(i for i in arrival if layout.QC_TOTES[i][1])
        defect = next(i for i in arrival if not layout.QC_TOTES[i][1])
        second_ok = [i for i in arrival if layout.QC_TOTES[i][1]][1]
        bot, top = layout.DROP_BOT_Y, layout.DROP_TOP_Y
        checks = [("reel tote on the Coupe table", "/Atelier/bac_magasin", layout.STATIONS["coupe"]["pose"][0], top),
                  ("1st OK tote on the Expedition table", name(first_ok), layout.STATIONS["expedition"]["pose"][0], bot),
                  ("defect tote on the Assemblage table", name(defect), layout.STATIONS["assemblage"]["pose"][0], bot),
                  ("2nd OK tote on the OK table (after the robot freed it)", name(second_ok), layout.TEST_DROP_X, bot)]
        ok = True
        print("\nTotes (true positions):")
        for text, path, x, y in checks:
            good, where = on_table(sim, path, x, y)
            ok &= good
            print(f"  {text:55s} {where}: {'yes' if good else 'NO'}")
        done = sum(o.state == "DONE" for o in mgr.finished)
        contacts = sum(r["contacts_moving"] + r["contacts_stopped"] for o in mgr.finished for r in o.legs)
        print(f"\n{done} orders done, {contacts} contact steps, QC sorted {sim.qc.stats['sorted']}, "
              f"QC fault {sim.qc.fault}, {sim.getSimulationTime():.0f} s simulated")
        ok &= done >= 3 and contacts == 0 and not mgr.fault and not sim.qc.fault
    except KeyboardInterrupt:
        print("Stopped by the user.")
        ok = False
    finally:
        robot.stop()
        sim.stopSimulation()
        if live:
            sim.close()
    print("RESULT:", "PASSED" if ok else "FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
