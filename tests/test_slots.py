r"""Part 8.3 test: coded slots (one ArUco code per place), Magasin inventory, code check + refusal.

Magasin = 3 coded pick slots: 11 red reels, 12 black reels, 13 blue reels (layout.INVENTORY);
Expedition = 2 coded drop slots (46, 47). No QC tote stream here (belt empty), operators on.
The manager's MAP IS WRONG for slot 13 on purpose (it believes slot 13 is where slot 12 is, as after
re-arranging a table without updating the map): the code check must catch it.
Orders (scripted):
  #1 "bobines rouges"  Magasin -> Expedition  -> slot 11 -> first free place 46
  #2 "bobines bleues"  Magasin -> Coupe       -> robot in front of slot 12, reads code 12, expected 13
                                                 -> REFUSED, nothing taken
  #3 "bobines noires"  Magasin -> Expedition  -> slot 12 -> 46 still occupied -> place 47
  #4 "bobines rouges"  Magasin -> Coupe       -> slot 11 empty: waits until the storekeeper has put the
                                                 red tote back (taken from 46 by the Expedition operator)
Check: states + slots as above, 1 refusal (code 12 read), true positions (red tote on the Coupe table,
black on place 47 - or back on slot 12 if its operator has already taken it -, blue still on slot 13),
no contact, no fault.

Needs scenes\atelier_cablage.ttt open (rebuilt with the coded slots). Run (PowerShell, project folder):
  .\.venv\Scripts\python.exe -u tests\test_slots.py [--live]
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

TAKE_AFTER = 250.0          # long enough that place 46 is still occupied when order #3 arrives
RESTOCK_AFTER = 20.0
TIMEOUT = 1000.0


def main():
    sim = Factory(RemoteAPIClient(), flow=True, count=0, take_after=TAKE_AFTER, restock_after=RESTOCK_AFTER)
    live = "--live" in sys.argv
    if live:
        sim = LiveView(sim, autosave="live_emplacements")
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
        wrong_map = {c: layout.spot(c)[1] for s in layout.STATIONS for c in layout.spots(s)}
        wrong_map[13] = wrong_map[12]                       # the map error the code check must catch
        mgr = MissionManager(sim, robot, nav, tr, qc=sim.qc, spot_pose=wrong_map)
        orders = [mgr.add(Order("bobines rouges", "magasin", "expedition", source="script")),
                  mgr.add(Order("bobines bleues", "magasin", "coupe", source="script")),
                  mgr.add(Order("bobines noires", "magasin", "expedition", source="script")),
                  mgr.add(Order("bobines rouges", "magasin", "coupe", source="script"))]
        mgr.run(stop=lambda m: len(m.finished) >= 4, timeout=TIMEOUT)
        flow = sim.flow

        print("\nOrders:")
        print("\n".join(mgr.summary()))
        for o in mgr.finished:
            print(f"  #{o.id}: {o.result}")
        for o in mgr.waiting:
            print(f"  {o.label():45s} still waiting")
        if mgr.fault:
            print("MANAGER FAULT:", mgr.fault)
        print("\nRefusals (time, order, expected, read, seen):", mgr.refusals)
        print("Operators:")
        for t, e in flow.stats["events"]:
            print(f"  {t:7.1f} s  {e}")

        want = [("DONE", 11, 46), ("REFUSED", 13, None), ("DONE", 12, 47), ("DONE", 11, 2)]
        ok = True
        for o, (state, pick, drop) in zip(orders, want):
            good = o.state == state and o.pickup_slot == pick and (drop is None or o.drop_slot == drop)
            ok &= good
            print(f"  #{o.id}: {o.state} {o.pickup_slot} -> {o.drop_slot}, expected {state} {pick} -> {drop}: "
                  f"{'yes' if good else 'NO'}")
        restock = [t for t, e in flow.stats["events"] if "slot 11 restocked" in e]
        late = bool(restock) and orders[3].started is not None and orders[3].started >= restock[0]
        print(f"  #4 started {orders[3].started} s, slot 11 restocked at {restock}: {'yes' if late else 'NO'}")
        ok &= late
        ok &= len(mgr.refusals) == 1 and mgr.refusals[0][2:4] == (13, 12)

        print("\nTotes (true positions):")
        # red, black, blue; the black tote may already have been taken from place 47 by the operator
        # (TAKE_AFTER after its delivery) and put back on its slot 12 by the storekeeper
        back12 = any("slot 12 restocked" in e for _, e in flow.stats["events"])
        names = {"bac_magasin": 2, "bac_magasin_1": 12 if back12 else 47, "bac_magasin_2": 13}
        for h in flow.totes:
            name = sim.getObjectAlias(h)
            if name in names:
                where = flow._resting_on(h)
                good = where == names[name]
                ok &= good
                print(f"  {name:15s} on place {where}, expected {names[name]}: {'yes' if good else 'NO'}")
        contacts = sum(r["contacts_moving"] + r["contacts_stopped"] for o in mgr.finished for r in o.legs)
        print(f"\n{contacts} contact steps, {sim.getSimulationTime():.0f} s simulated")
        ok &= contacts == 0 and not mgr.fault
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
