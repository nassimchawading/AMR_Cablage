r"""Part 8.4 test: simulated battery + automatic charge at the charging station.

The battery starts at START % (just above battery.LOW = 25 %). Two reel orders:
  #1 "bobines rouges" Magasin -> Coupe     the level goes below LOW during this order: the robot
                                           FINISHES it, then goes to charge (class 1, before #2)
  #2 "bobines noires" Magasin -> Expedition   after the charge
Check: #1 done before the charge, the charge before #2 started, robot docked on the charging station
(marker 0), charged to >= battery.FULL, both orders done, no contact, no fault. Prints the drain per
order and per cause (idle / drive / arm) for the report.

Needs scenes\atelier_cablage.ttt open. Run (PowerShell, project folder; ~8 min real):
  .\.venv\Scripts\python.exe -u tests\test_battery.py [--live]
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import battery as bat_model
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

START = 27.0
TIMEOUT = 900.0


def main():
    sim = Factory(RemoteAPIClient(), flow=True, count=0, battery=dict(level=START))
    live = "--live" in sys.argv
    if live:
        sim = LiveView(sim, autosave="live_batterie")
    if sim.getSimulationState() != sim.simulation_stopped:
        print("STOP: stop the simulation first (square Stop button), then run again.")
        return 1
    robot, sensors = YouBot(sim), Sensors(sim)
    walkers = {w.name: sim.getObject(f"/Atelier/operateurs_mobiles/{w.name}/Bill") for w in layout.WALKERS}
    sim.setStepping(True)
    sim.startSimulation()
    ok = False
    try:
        for _ in range(5):
            sim.step()
        grid = OccupancyGrid.from_layout()
        nav = Navigator(sim, robot, sensors, grid, walkers=walkers)
        if live:
            sim.setup(robot, sensors, grid, nav=nav, walkers=walkers)
        tr = Transfer(sim, robot, sensors, Docker(sim, robot, sensors), YouBotArm(sim))
        mgr = MissionManager(sim, robot, nav, tr, qc=sim.qc)
        o1 = mgr.add(Order("bobines rouges", "magasin", "coupe", source="script"))
        o2 = mgr.add(Order("bobines noires", "magasin", "expedition", source="script"))
        mgr.run(stop=lambda m: len(m.finished) >= 2, timeout=TIMEOUT)
        b = sim.battery

        print("\nOrders:")
        print("\n".join(mgr.summary()))
        for o in mgr.finished:
            print(f"  #{o.id}: battery {o.bat_start} % -> {o.bat_end} % ({o.bat_start - o.bat_end:.1f} % used)")
        if mgr.fault:
            print("MANAGER FAULT:", mgr.fault)
        print("\nCharges:")
        for c in mgr.charges:
            print(f"  {c['reason']}: left at {c['start']} s with {c['level_in']} %, at the station {c['arrived']} s "
                  f"(docked {c['docked']}), {c['level_out']} % at {c['end']} s")
        print(f"\nBattery now {b.level:.1f} %, used: idle {b.used['idle']:.1f} %, drive {b.used['drive']:.1f} % "
              f"({b.distance:.1f} m), arm {b.used['arm']:.1f} %; charged {b.charged:.1f} %")
        print("Level history (s, %):", b.history)
        contacts = sum(r["contacts_moving"] + r["contacts_stopped"] for o in mgr.finished for r in o.legs)
        print(f"{contacts} contact steps, {sim.getSimulationTime():.0f} s simulated")
        c = mgr.charges[0] if mgr.charges else None
        ok = (len(mgr.charges) == 1 and c["docked"] and c["level_out"] >= bat_model.FULL
              and o1.state == "DONE" and o2.state == "DONE"
              and o1.done <= c["start"] + 1 and o2.started >= c["end"] - 1
              and o1.bat_end < bat_model.LOW and contacts == 0 and not mgr.fault)
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
