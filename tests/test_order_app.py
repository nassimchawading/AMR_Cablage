r"""Part 8.6 test: the manual-order window, used the way an operator would (its buttons' functions are
called with the form filled in, while the window is visible).

No QC tote stream here (belt empty). Scripted reel orders every 60 s, the first at 5 s.
At ACT_T s (robot busy with the scripted order #1) the "operator":
  - pauses the scripted orders                    -> no other scripted order is created
  - adds A: "bobines bleues (emplacement 13)" -> Assemblage, place 42, URGENT
  - adds B: "bobines noires" -> Expedition, normal, then CANCELS it (selected in the queue)
  - adds C: "bobines rouges" -> Coupe, normal (stays behind A)
  - presses "Envoyer en charge"
Expected order of work after #1: charge (class 1), then A (urgent, class 2), then C (normal).
Check: #1 DONE, only 1 scripted order, B CANCELLED, charge after #1 and before A, A DONE 13 -> 42, C DONE
after A, no contact, no fault.

Needs scenes\atelier_cablage.ttt open. Run (PowerShell, project folder; ~15 min real):
  .\.venv\Scripts\python.exe -u tests\test_order_app.py
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
from missions import MissionManager
from navigator import Navigator
from occupancy import OccupancyGrid
from order_app import OrderApp
from sensors import Sensors
from shift import ScriptedOrders
from transfer import Transfer
from youbot import YouBot

ACT_T = 10.0
TIMEOUT = 1200.0


def main():
    sim = Factory(RemoteAPIClient(), flow=True, count=0, battery=dict(level=80.0))
    if not sim.getStringParam(sim.stringparam_scene_path_and_name).endswith("atelier_cablage.ttt"):
        print("STOP: open scenes\\atelier_cablage.ttt in front first.")
        return 1
    if sim.getSimulationState() != sim.simulation_stopped:
        print("STOP: stop the simulation first (square Stop button), then run again.")
        return 1
    robot, sensors = YouBot(sim), Sensors(sim)
    walkers = {w.name: sim.getObject(f"/Atelier/operateurs_mobiles/{w.name}/Bill") for w in layout.WALKERS}
    sim.setStepping(True)
    sim.startSimulation()
    ok, app = False, None
    try:
        for _ in range(5):
            sim.step()
        grid = OccupancyGrid.from_layout()
        nav = Navigator(sim, robot, sensors, grid, walkers=walkers)
        tr = Transfer(sim, robot, sensors, Docker(sim, robot, sensors), YouBotArm(sim))
        mgr = MissionManager(sim, robot, nav, tr, qc=sim.qc)
        script = ScriptedOrders(sim, mgr, every=60.0, first=5.0)
        sim.tickers.append(script.tick)
        app = OrderApp(sim, mgr, script)
        sim.tickers.append(app.tick)
        made = {}

        def operator():                       # what a person does in the window, once, at ACT_T
            if made or sim.getSimulationTime() < ACT_T:
                return
            app.toggle_script()
            label = next(k for k, v in app.pickups.items() if v[1] == 13)
            app.pick.set(label)
            app.drop.set("assemblage")
            app._update_places()
            app.place.set("42")
            app.urgent.set(True)
            made["A"] = app.add_order()
            app.pick.set(next(k for k, v in app.pickups.items() if v[1] == 12))
            app.drop.set("expedition")
            app._update_places()
            app.urgent.set(False)
            made["B"] = app.add_order()
            app.refresh()
            app.queue.selection_clear(0, "end")
            app.queue.selection_set(app._queue_orders.index(made["B"]))
            app.cancel_selected()
            app.pick.set(next(k for k, v in app.pickups.items() if v[1] == 11))
            app.drop.set("coupe")
            app._update_places()
            made["C"] = app.add_order()
            app.send_to_charge()
        sim.tickers.append(operator)

        mgr.run(stop=lambda m: "C" in made and made["C"].state in ("DONE", "FAILED", "REFUSED"), timeout=TIMEOUT)

        print("\nOrders:")
        print("\n".join(mgr.summary()))
        for o in mgr.waiting:
            print(f"  {o.label():45s} still waiting")
        print("Charges:", [{k: v for k, v in c.items() if k != "leg"} for c in mgr.charges])
        if mgr.fault:
            print("MANAGER FAULT:", mgr.fault)
        A, B, C = made.get("A"), made.get("B"), made.get("C")
        first = mgr.finished[0] if mgr.finished else None
        scripted = [o for o in mgr.finished + mgr.waiting if o.source == "script"]
        c = mgr.charges[0] if mgr.charges else None
        checks = {
            "#1 scripted order done": first is not None and first.source == "script" and first.state == "DONE",
            "only 1 scripted order (paused)": len(scripted) == 1,
            "B cancelled": B is not None and B.state == "CANCELLED",
            "charge after #1, before A": c is not None and c["start"] >= first.done - 1 and A.started >= c["end"] - 1,
            "A done 13 -> 42 (urgent)": A is not None and A.state == "DONE" and A.pickup_slot == 13 and A.drop_slot == 42,
            "C done after A": C is not None and C.state == "DONE" and C.started >= A.done - 1,
        }
        for k, v in checks.items():
            print(f"  {k:32s} {'yes' if v else 'NO'}")
        contacts = sum(r["contacts_moving"] + r["contacts_stopped"] for o in mgr.finished for r in o.legs)
        print(f"{contacts} contact steps, {sim.getSimulationTime():.0f} s simulated")
        ok = all(checks.values()) and contacts == 0 and not mgr.fault
    except KeyboardInterrupt:
        print("Stopped by the user.")
    finally:
        robot.stop()
        sim.stopSimulation()
        if app is not None:
            app.close()
    print("RESULT:", "PASSED" if ok else "FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
