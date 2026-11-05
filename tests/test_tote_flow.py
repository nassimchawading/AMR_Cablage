r"""Part 8.2 test: regular tote stream to the QC cell + operators, with the mission manager.

The belt starts EMPTY; the operators (src\tote_flow.py) put 3 harness totes on the QC conveyor, one
every INTERVAL s, with the fixed sequence OK, OK, DEFECT. Each tote the UR5 sorts creates a QC order
(OK table -> Expedition, REBUTS -> Assemblage). Two scripted reel orders Magasin -> Coupe: one at the
start, one added at SECOND_REEL s - it must WAIT until the storekeeper has restocked the Magasin (the
first reel tote is taken from the Coupe table TAKE_AFTER s after its delivery, then put back at the
Magasin). Delivered totes are taken away by the station operators.
Check: 5 orders DONE (2 reel + 3 QC), camera decisions = the sequence, 3 totes spawned, >= 2 totes
taken by operators (the last deliveries end the test before their operator comes), Magasin restocked, no tote on the floor / tipped over, no contact, no fault.
INTERVAL is shorter than the shift value (tote_flow.QC_INTERVAL) to keep the test ~20 min real.

Needs scenes\atelier_cablage.ttt open (rebuilt with the spare totes). Run (PowerShell, project folder):
  .\.venv\Scripts\python.exe -u tests\test_tote_flow.py [--live]
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

INTERVAL = 150.0
SEQUENCE = (True, True, False)
SECOND_REEL = 200.0
TIMEOUT = 1300.0


def main():
    sim = Factory(RemoteAPIClient(), flow=True, interval=INTERVAL, sequence=SEQUENCE, count=len(SEQUENCE))
    live = "--live" in sys.argv
    if live:
        sim = LiveView(sim, autosave="live_flux")
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
        mgr.add(Order("bobines", "magasin", "coupe", source="script"))
        second = []

        def stop(m):                 # also the "script": adds the 2nd reel order at SECOND_REEL s
            if not second and sim.getSimulationTime() >= SECOND_REEL:
                second.append(m.add(Order("bobines", "magasin", "coupe", source="script")))
            return sum(o.state == "DONE" for o in m.finished) >= 5 or any(o.state == "FAILED" for o in m.finished)

        mgr.run(stop=stop, timeout=TIMEOUT)
        flow, qc = sim.flow, sim.qc

        print("\nOrders:")
        print("\n".join(mgr.summary()))
        for o in mgr.waiting:
            print(f"  {o.label():45s} {o.source:6s} still waiting")
        if mgr.fault:
            print("MANAGER FAULT:", mgr.fault)
        print("\nOperators:")
        for t, e in flow.stats["events"]:
            print(f"  {t:7.1f} s  {e}")
        # every tote: in the reserve, or standing upright somewhere above the floor (table, belt, robot)
        lying = []
        for h in flow.totes:
            x, y, z = sim.getObjectPosition(h, sim.handle_world)
            if z > -0.5 and (z < 0.2 or sim.getObjectMatrix(h, sim.handle_world)[10] < 0.98):
                lying.append(f"{sim.getObjectAlias(h)} at ({x:.2f}, {y:.2f}, {z:.2f})")
        done = [o for o in mgr.finished if o.state == "DONE"]
        reels = [o for o in done if o.source == "script"]
        qc_orders = [o for o in done if o.source == "qc"]
        contacts = sum(r["contacts_moving"] + r["contacts_stopped"] for o in mgr.finished for r in o.legs)
        busy = sum(o.done - o.started for o in mgr.finished)
        qc_busy = sum(o.done - o.started for o in qc_orders)
        decisions = [e for _, e in qc.stats["events"] if e.startswith("camera:")]
        seq_ok = (qc.stats["ok"], qc.stats["defect"]) == (sum(SEQUENCE), len(SEQUENCE) - sum(SEQUENCE))
        if second:
            o = second[0]
            print(f"\n2nd reel order: created {o.created:.0f} s, started "
                  f"{o.started if o.started is None else round(o.started)} s (Magasin restocked first)")
        print(f"\nCamera: {decisions}")
        print(f"{len(done)} orders done ({len(reels)} reel, {len(qc_orders)} QC), {contacts} contact steps, "
              f"spawned {flow.stats['spawned']}, taken {flow.stats['taken']}, restocked {flow.stats['restocked']}, "
              f"QC fault {qc.fault}, {sim.getSimulationTime():.0f} s simulated")
        print(f"Robot busy {busy:.0f} s, of which QC orders {qc_busy:.0f} s "
              f"({100 * qc_busy / max(busy, 1):.0f} %); totes lying / tipped: {lying or 'none'}")
        ok = (len(reels) == 2 and len(qc_orders) == 3 and seq_ok and flow.stats["spawned"] == 3
              and flow.stats["taken"] >= 2 and flow.stats["restocked"] >= 1 and not lying
              and contacts == 0 and not mgr.fault and not qc.fault
              and second and second[0].started is not None and second[0].started > SECOND_REEL + 1)
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
