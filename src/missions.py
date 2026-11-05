r"""Part 8 - mission manager: transport orders, priorities, one order at a time.

    mgr = MissionManager(sim, robot, nav, transfer, qc=sim.qc)     # sim = Factory (QC cell running)
    mgr.add(Order("bobines", "magasin", "coupe", source="script"))
    mgr.run(stop=lambda m: len(m.finished) >= 3, timeout=900)

Order life (one state machine per order, the robot carries ONE tote at a time):
    WAITING -> GO_TO_PICKUP -> LOAD -> GO_TO_DROP -> UNLOAD -> DONE   (or FAILED)
Each step uses the existing building blocks: Navigator.go_to (A* + avoidance + prediction) and
Transfer.load / unload (ArUco + lidar docking, arm). Meanwhile the QC cell keeps sorting in the
background (Factory runs it before every simulation step).

Where orders come from:
  - the QC cell (automatic): a tote put on the OK table -> "OK table -> Expedition", on the REBUTS table
    -> "REBUTS -> Assemblage" (rework); created through QCCell.on_placed;
  - scripts / the user (Part 8.3 / 8.6): MissionManager.add(Order(...)).
Priority classes (lower first, then arrival order):
  1 low battery (Part 8.4)
  2 an order that UNBLOCKS the QC cell: its table is full and the next tote waits at the end stop
  3 everything else, QC orders included (user, 2026-11-05: QC work must not monopolise the robot)
"""

from dataclasses import dataclass, field

from layout import STATIONS

QC_PICKUP = {True: "test", False: "rebuts"}          # OK table / REBUTS table (layout.STATIONS keys)
QC_DROP = {True: "expedition", False: "assemblage"}
CLASS_BATTERY, CLASS_UNBLOCK, CLASS_NORMAL = 1, 2, 3
DROP_WAIT_MAX = 300.0           # s in front of an occupied drop table before giving up (fault)
MAX_TRIES = 2                   # load attempts per order (a failed grasp leaves the tote on the table)


@dataclass
class Order:
    what: str                   # e.g. "bobines", "faisceau OK"
    pickup: str                 # station key (layout.STATIONS)
    drop: str
    source: str = "manual"      # "qc", "script", "manual"
    id: int = 0
    created: float = 0.0        # simulation times (s)
    started: float | None = None
    loaded: float | None = None
    done: float | None = None
    state: str = "WAITING"
    result: str = ""
    tries: int = 0              # failed load attempts so far
    legs: list = field(default_factory=list)     # navigator statistics of the two drives

    def label(self):
        return f"#{self.id} {self.what}: {self.pickup} -> {self.drop}"


class MissionManager:
    def __init__(self, sim, robot, nav, transfer, qc=None, log=print):
        self.sim, self.robot, self.nav, self.transfer, self.qc, self.log = sim, robot, nav, transfer, qc, log
        self.waiting, self.finished = [], []
        self.current = None
        self.fault = None           # set when the robot is stuck with a tote on board: the manager stops
        self._next_id = 1
        if qc is not None:
            qc.on_placed = self._qc_order

    # ------------------------------------------------------------ queue
    def add(self, order):
        order.id, order.created = self._next_id, self.sim.getSimulationTime()
        self._next_id += 1
        self.waiting.append(order)
        self.log(f"[orders] new {order.label()} ({order.source})")
        return order

    def _qc_order(self, ok):
        self.add(Order("faisceau OK" if ok else "faisceau a reprendre", QC_PICKUP[ok], QC_DROP[ok], source="qc"))

    def priority(self, order):
        if order.source == "qc" and self.qc is not None and self.qc.blocked is not None \
                and QC_PICKUP[self.qc.blocked] == order.pickup:
            return CLASS_UNBLOCK
        return CLASS_NORMAL

    def ready(self, order):
        """An order can start once its pickup table has a tote on it (with the operators of tote_flow:
        e.g. a reel order waits until the Magasin has been restocked). QC orders exist only once the
        UR5 has put their tote down."""
        flow = getattr(self.sim, "flow", None)
        return flow is None or order.source == "qc" or flow.occupied(order.pickup)

    def next_order(self):
        ready = [o for o in self.waiting if self.ready(o)]
        if not ready:
            return None
        return min(ready, key=lambda o: (self.priority(o), o.created, o.id))

    def _wait_drop_free(self, order):
        """With the operators: never put a tote on a table where the last delivery is still standing;
        wait in front of it until the station's operator has taken it (tote_flow.TAKE_AFTER)."""
        flow = getattr(self.sim, "flow", None)
        if flow is None or not flow.occupied(order.drop):
            return True
        order.state = "WAIT_DROP"
        self.log(f"[orders] {order.drop} table still occupied: waiting for its operator")
        t0 = self.sim.getSimulationTime()
        while flow.occupied(order.drop):
            if self.sim.getSimulationTime() - t0 > DROP_WAIT_MAX:
                return False
            self.robot.stop()
            self.sim.step()
        return True

    # ------------------------------------------------------------ execution
    def run(self, stop=lambda manager: False, timeout=900.0):
        """Execute orders until stop(manager) is true, the timeout (simulated s) or a fault.
        With nothing to do, the robot waits where it is (the factory keeps running)."""
        t0 = self.sim.getSimulationTime()
        idle = False
        while not stop(self) and self.fault is None and self.sim.getSimulationTime() - t0 < timeout:
            order = self.next_order()
            if order is None:
                if not idle:
                    self.log("[orders] nothing to do: waiting")
                idle = True
                self.sim.step()
                continue
            idle = False
            self.waiting.remove(order)
            cls = self.priority(order)
            self.log(f"[orders] start {order.label()} (class {cls}, waited "
                     f"{self.sim.getSimulationTime() - order.created:.0f} s, {len(self.waiting)} still waiting)")
            self.execute(order)

    def execute(self, order):
        sim = self.sim
        self.current = order
        order.started = sim.getSimulationTime()
        pick, drop = STATIONS[order.pickup], STATIONS[order.drop]

        order.state = "GO_TO_PICKUP"
        r = self.nav.go_to(pick["pose"])
        order.legs.append(r)
        if not r["reached"]:
            return self._end(order, "FAILED", f"{order.pickup} not reached")

        order.state = "LOAD"
        r = self.transfer.load(pick["marker_id"])
        if not r["ok"]:
            order.tries += 1
            if order.tries < MAX_TRIES:     # robot still empty, tote still on the table: try again later
                order.state, order.started = "WAITING", None
                self.waiting.append(order)
                self.current = None
                self.log(f"[orders] load failed ({r['info']}): {order.label()} back in the queue "
                         f"(attempt {order.tries + 1} of {MAX_TRIES})")
                return
            return self._end(order, "FAILED", f"load failed {order.tries} times: {r['info']}")
        order.loaded = sim.getSimulationTime()
        if order.source == "qc" and self.qc is not None:
            self.qc.taken(order.pickup == QC_PICKUP[True])        # the UR5 may use the spot again

        order.state = "GO_TO_DROP"
        r = self.nav.go_to(drop["pose"])
        order.legs.append(r)
        if not r["reached"]:
            self.fault = f"{order.label()}: {order.drop} not reached, tote still on board"
            return self._end(order, "FAILED", self.fault)

        if not self._wait_drop_free(order):
            self.fault = f"{order.label()}: {order.drop} table never freed, tote still on board"
            return self._end(order, "FAILED", self.fault)
        order.state = "UNLOAD"
        r = self.transfer.unload(drop["marker_id"])
        if not r["ok"]:
            self.fault = f"{order.label()}: unload failed ({r['info']}), tote still on board"
            return self._end(order, "FAILED", self.fault)
        return self._end(order, "DONE", f"delivered in {sim.getSimulationTime() - order.started:.0f} s")

    def _end(self, order, state, result):
        order.state, order.result, order.done = state, result, self.sim.getSimulationTime()
        self.current = None
        self.finished.append(order)
        self.log(f"[orders] {state} {order.label()}: {result}")

    # ------------------------------------------------------------ report
    def summary(self):
        """Lines: one per finished order (waiting time, execution time, stops, re-plans, contacts)."""
        out = []
        for o in self.finished:
            stops = sum(r["stops"] for r in o.legs)
            replans = sum(r["replans"] for r in o.legs)
            contacts = sum(r["contacts_moving"] + r["contacts_stopped"] for r in o.legs)
            out.append(f"  {o.label():45s} {o.source:6s} {o.state:6s} waited {o.started - o.created:5.0f} s, "
                       f"executed {o.done - o.started:5.0f} s, stops {stops}, re-plans {replans}, contacts {contacts}")
        return out
