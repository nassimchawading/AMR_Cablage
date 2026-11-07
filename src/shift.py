r"""Part 8.5 - one SHIFT ("poste de travail") of the workshop: everything runs together for X minutes.

    .\.venv\Scripts\python.exe -u src\shift.py                      (30 min simulated, live windows)
    .\.venv\Scripts\python.exe -u src\shift.py --minutes 20 --battery 60 --no-live
    .\.venv\Scripts\python.exe -u src\shift.py --mosaic --record poste.mp4  (one window, saved in media\)

What runs (all in the workshop scene, scenes\atelier_cablage.ttt):
  - the QC cell (camera + UR5) and the operators (src\tote_flow.py): one harness tote every
    --interval s on the QC conveyor (fixed sequence OK, OK, DEFECT), delivered totes taken away,
    Magasin slots restocked; every tote the UR5 sorts creates a transport order;
  - the SCRIPTED reel orders (ScriptedOrders): every REEL_EVERY s the next order of REEL_SCRIPT
    ("bobines rouges: Magasin -> Coupe", ...); can be paused (manual app, Part 8.6);
  - the battery (src\battery.py) with the automatic charge;
  - the mission manager (src\missions.py) driving the robot (navigation, docking, arm).
The shift ends at --minutes: the robot finishes the order it is doing, then the program stops and
writes the end-of-shift summary (results\shift_<date>_<time>.txt + .json): orders done / refused /
failed / waiting, waiting and execution times, QC cell, operators, battery and charges, navigation
(distance, stops, re-plans, minimum distance to a worker, contacts). Numbers for Part 11.
Live windows (default): camera, lidar + sonars, map, and the "ordres" panel (current order, battery,
queue, last orders). The manual-order window (src\order_app.py, Part 8.6) lets you add orders (urgent or
not), cancel waiting ones, pause the scripted orders and send the robot to charge (--no-app: without it). q in a window or Ctrl+C in the terminal stops the shift early (summary still written).
"""

import argparse
import json
import statistics
import sys
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import layout
import tote_flow
from arm import YouBotArm
from docking import Docker
from factory import Factory
from live_view import LiveView
from missions import MissionManager, Order
from navigator import Navigator
from order_app import OrderApp
from occupancy import OccupancyGrid
from sensors import Sensors
from transfer import Transfer
from youbot import YouBot

SCENE = "atelier_cablage.ttt"
REEL_EVERY = 240.0           # s between two scripted reel orders
REEL_FIRST = 5.0             # s: first scripted order
# the reel orders of a shift, repeated: (what, drop station). Reels feed the cutting machines; the blue
# terminal reels go to Sertissage (that route passes the stray box: re-plan shown)
REEL_SCRIPT = [("bobines rouges", "coupe"), ("bobines noires", "coupe"), ("bobines bleues", "sertissage")]


class ScriptedOrders:
    """Adds the scripted reel orders at fixed times (called before every simulation step)."""

    def __init__(self, sim, mgr, every=REEL_EVERY, first=REEL_FIRST, script=REEL_SCRIPT):
        self.sim, self.mgr, self.every, self.script = sim, mgr, every, script
        self.next_t = sim.getSimulationTime() + first
        self.k = 0
        self.paused = False          # manual app (8.6): pause / resume

    def tick(self):
        t = self.sim.getSimulationTime()
        if self.paused or t < self.next_t:
            return
        what, drop = self.script[self.k % len(self.script)]
        self.mgr.add(Order(what, "magasin", drop, source="script"))
        self.k += 1
        self.next_t = t + self.every


def _stats(values):
    return dict(n=len(values), mean=round(statistics.mean(values), 1) if values else None,
                max=round(max(values), 1) if values else None)


def summary(sim, mgr, minutes, wall_s):
    """End-of-shift summary: (printable lines, dict for the JSON file)."""
    fin = mgr.finished
    done = [o for o in fin if o.state == "DONE"]
    legs = [r for o in fin for r in o.legs] + [c["leg"] for c in mgr.charges if "leg" in c]
    t_end = sim.getSimulationTime()
    busy = sum(o.done - o.started for o in fin) + sum(c["end"] - c["start"] for c in mgr.charges)
    by_source = {}
    for src in ("qc", "script", "manual"):
        ds = [o for o in done if o.source == src]
        by_source[src] = dict(done=len(ds), wait=_stats([o.started - o.created for o in ds]),
                              exec=_stats([o.done - o.started for o in ds]))
    bat, flow, qc = sim.battery, sim.flow, sim.qc
    data = dict(
        shift_min=minutes, simulated_s=round(t_end, 1), wall_s=round(wall_s),
        orders=dict(created=mgr._next_id - 1, done=len(done),
                    refused=sum(o.state == "REFUSED" for o in fin), failed=sum(o.state == "FAILED" for o in fin),
                    cancelled=sum(o.state == "CANCELLED" for o in fin),
                    waiting=len(mgr.waiting), by_source=by_source,
                    in_progress=mgr.current.label() if mgr.current is not None else None),
        interventions=mgr.interventions,
        robot_busy_pct=round(100 * busy / max(t_end, 1), 1),
        qc=dict(inspected=qc.stats["inspected"], ok=qc.stats["ok"], defect=qc.stats["defect"],
                sorted=qc.stats["sorted"], fault=qc.fault),
        operators=dict(spawned=flow.stats["spawned"], taken=flow.stats["taken"], restocked=flow.stats["restocked"],
                       by_hand=flow.stats.get("by_hand", 0)),
        battery=dict(end=round(bat.level, 1), used={k: round(v, 1) for k, v in bat.used.items()},
                     charged=round(bat.charged, 1), distance_m=round(bat.distance, 1),
                     charges=[{k: v for k, v in c.items() if k != "leg"} for c in mgr.charges]),
        navigation=dict(legs=len(legs), distance_m=round(sum(r["distance"] for r in legs), 1),
                        stops=sum(r["stops"] for r in legs), replans=sum(r["replans"] for r in legs),
                        escapes=sum(r["escapes"] for r in legs),
                        min_dist_worker=min((r["min_dist_worker"] for r in legs), default=None),
                        contacts_moving=sum(r["contacts_moving"] for r in legs),
                        contacts_stopped=sum(r["contacts_stopped"] for r in legs)),
        refusals=mgr.refusals, fault=mgr.fault,
        order_list=[dict(id=o.id, what=o.what, source=o.source, pickup=o.pickup, pickup_slot=o.pickup_slot,
                         drop=o.drop, drop_slot=o.drop_slot, state=o.state, result=o.result,
                         created=round(o.created, 1), started=o.started and round(o.started, 1),
                         done=o.done and round(o.done, 1), battery=(o.bat_start, o.bat_end)) for o in fin])
    o, n, b = data["orders"], data["navigation"], data["battery"]
    lines = [f"END OF SHIFT - {minutes:.0f} min planned, {t_end / 60:.1f} min simulated ({wall_s / 60:.0f} min real)",
             f"Orders: {o['created']} created, {o['done']} done, {o['refused']} refused, {o['failed']} failed, "
             f"{o['cancelled']} cancelled, {o['waiting']} still waiting" + (f", in progress: {o['in_progress']}" if o["in_progress"] else "")]
    for src, s in by_source.items():
        if s["done"]:
            lines.append(f"  {src:6s}: {s['done']} done, waiting mean {s['wait']['mean']} s (max {s['wait']['max']}), "
                         f"execution mean {s['exec']['mean']} s (max {s['exec']['max']})")
    lines += [f"Robot busy {data['robot_busy_pct']} % of the shift (orders + charging)",
              f"QC cell: {data['qc']['inspected']} inspected ({data['qc']['ok']} OK, {data['qc']['defect']} defect), "
              f"{data['qc']['sorted']} sorted, fault {data['qc']['fault']}",
              f"Operators: {data['operators']['spawned']} totes put on the belt, {data['operators']['taken']} taken, "
              f"{data['operators']['restocked']} restocks, {data['operators']['by_hand']} taken by hand "
              f"(operator called: {mgr.interventions or 'never'})",
              f"Battery: end {b['end']} %, used idle {b['used']['idle']} / drive {b['used']['drive']} / arm "
              f"{b['used']['arm']} %, {len(b['charges'])} charge(s), charged {b['charged']} %",
              f"Navigation: {n['legs']} drives, {n['distance_m']} m, {n['stops']} stops, {n['replans']} re-plans, "
              f"{n['escapes']} escapes, min distance to a worker {n['min_dist_worker']} m, "
              f"contacts {n['contacts_moving']} moving / {n['contacts_stopped']} stopped",
              f"Refusals: {mgr.refusals or 'none'}", f"Manager fault: {mgr.fault}", "", "Orders:"]
    lines += mgr.summary()
    if mgr.current is not None:
        lines.append(f"  {mgr.current.label():45s} {mgr.current.source:6s} IN PROGRESS ({mgr.current.state}) "
                     f"when the shift stopped")
    lines += [f"  {w.label():45s} {w.source:6s} still waiting" for w in mgr.waiting]
    return lines, data


def main():
    ap = argparse.ArgumentParser(description="One shift of the cable-harness workshop")
    ap.add_argument("--minutes", type=float, default=30.0, help="shift length (simulated minutes)")
    ap.add_argument("--battery", type=float, default=100.0, help="battery level at the start (%%)")
    ap.add_argument("--interval", type=float, default=tote_flow.QC_INTERVAL, help="s between two QC totes")
    ap.add_argument("--reel-every", type=float, default=REEL_EVERY, help="s between two scripted reel orders")
    ap.add_argument("--no-live", action="store_true", help="no live windows (faster)")
    ap.add_argument("--mosaic", action="store_true", help="ONE 2x2 window (camera, lidar, map, orders) instead of 4")
    ap.add_argument("--record", metavar="FILE.mp4", help="record the mosaic window as a video (media\...)")
    ap.add_argument("--no-app", action="store_true", help="no manual-order window (Part 8.6)")
    args = ap.parse_args()

    sim = Factory(RemoteAPIClient(), flow=True, interval=args.interval, battery=dict(level=args.battery))
    live = not args.no_live
    if live:
        record = args.record and str(ROOT / "media" / Path(args.record).name)
        sim = LiveView(sim, title="poste", mosaic=args.mosaic or bool(record), record=record, autosave=None,
                       map_scale=2)
    scene = sim.getStringParam(sim.stringparam_scene_path_and_name)
    if not scene.endswith(SCENE):
        print(f"STOP: the scene in front in CoppeliaSim is {scene}, not {SCENE}. Switch the scene tab, then run again.")
        return 1
    if sim.getSimulationState() != sim.simulation_stopped:
        print("STOP: stop the simulation first (square Stop button), then run again.")
        return 1
    robot, sensors = YouBot(sim), Sensors(sim)
    walkers = {w.name: sim.getObject(f"/Atelier/operateurs_mobiles/{w.name}/Bill") for w in layout.WALKERS}
    sim.setStepping(True)
    sim.startSimulation()
    wall0 = time.perf_counter()
    mgr = app = None
    try:
        for _ in range(5):
            sim.step()
        grid = OccupancyGrid.from_layout()
        log = sim.log if live else print
        nav = Navigator(sim, robot, sensors, grid, walkers=walkers, log=log)
        tr = Transfer(sim, robot, sensors, Docker(sim, robot, sensors, log=log), YouBotArm(sim), log=log)
        mgr = MissionManager(sim, robot, nav, tr, qc=sim.qc, log=log)
        script = ScriptedOrders(sim, mgr, every=args.reel_every)
        sim.tickers.append(script.tick)
        if not args.no_app:
            app = OrderApp(sim, mgr, script)
            sim.tickers.append(app.tick)
        if live:
            sim.setup(robot, sensors, grid, nav=nav, walkers=walkers, mgr=mgr)
        end = args.minutes * 60.0
        print(f"Shift started: {args.minutes:.0f} min simulated, battery {args.battery:.0f} %, "
              f"QC tote every {args.interval:.0f} s, reel order every {args.reel_every:.0f} s")
        mgr.run(stop=lambda m: sim.getSimulationTime() >= end, timeout=end + 900.0)
    except KeyboardInterrupt:
        print("Shift stopped early by the user.")
    finally:
        robot.stop()
        lines = data = None
        if mgr is not None and sim.battery is not None:
            try:
                lines, data = summary(sim, mgr, args.minutes, time.perf_counter() - wall0)
            except Exception as e:        # never leave the simulation running because of the report
                print("Summary failed:", repr(e))
        sim.stopSimulation()
        if app is not None:
            app.close()
        if live:
            sim.close()
    if lines:
        print("\n" + "\n".join(lines))
        out = ROOT / "results"
        out.mkdir(exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M")
        (out / f"shift_{stamp}.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
        (out / f"shift_{stamp}.json").write_text(json.dumps(data, indent=1, default=str), encoding="utf-8")
        print(f"\nSummary saved: results\\shift_{stamp}.txt and .json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
