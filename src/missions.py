r"""Part 8 - mission manager: transport orders, priorities, one order at a time.

    mgr = MissionManager(sim, robot, nav, transfer, qc=sim.qc)     # sim = Factory (QC cell running)
    mgr.add(Order("bobines", "magasin", "coupe", source="script"))
    mgr.run(stop=lambda m: len(m.finished) >= 3, timeout=900)

Order life (one state machine per order, the robot carries ONE tote at a time):
    WAITING -> GO_TO_PICKUP -> [code check] -> LOAD -> GO_TO_DROP (WAIT_DROP if every place is taken)
            -> [code check] -> UNLOAD -> DONE          (or REFUSED: wrong code at the pickup, or FAILED)
CODED SLOTS (Part 8.3): every place for a tote has a code (layout.SLOTS: ArUco id on the table's face;
stations without slots: their own marker). An order names stations (and optionally slots); the manager
picks the slot - from the Magasin INVENTORY for the pickup ("bobines rouges" -> slot 11), the first free
place for the drop. In front of the slot, BEFORE docking, the robot reads the code most centred in its
camera image: it must be the expected one. Wrong code at the pickup -> the order is REFUSED (nothing
taken) and reported; at the drop -> that place is refused and reported, another free place is used.
The robot then docks on that slot's marker (ArUco + lidar), so the camera field of view is no issue.
Each step uses the existing building blocks: Navigator.go_to (A* + avoidance + prediction) and
Transfer.load / unload (ArUco + lidar docking, arm). Meanwhile the QC cell keeps sorting in the
background (Factory runs it before every simulation step).

Where orders come from:
  - the QC cell (automatic): a tote put on the OK table -> "OK table -> Expedition", on the REBUTS table
    -> "REBUTS -> Assemblage" (rework); created through QCCell.on_placed;
  - scripts / the user (Part 8.3 / 8.6): MissionManager.add(Order(...)).
Priority classes (lower first, then arrival order):
  1 low battery (Part 8.4): below battery.LOW the robot goes to charge once its CURRENT order is done
    (also on request: charge_requested), docks on the charging station (marker 0), charges to FULL
  2 an order that UNBLOCKS the QC cell: its table is full and the next tote waits at the end stop,
    or a manual order marked URGENT (Part 8.6)
  3 everything else, QC orders included (user, 2026-11-05: QC work must not monopolise the robot)
"""

from dataclasses import dataclass, field

import layout
from battery import FULL

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
    pickup_slot: int | None = None   # slot codes (None: chosen by the manager)
    drop_slot: int | None = None
    bat_start: float | None = None   # battery level (%) at the start / end of the order (Part 8.4)
    bat_end: float | None = None
    legs: list = field(default_factory=list)     # navigator statistics of the two drives
    urgent: bool = False        # manual order marked urgent (app, Part 8.6): priority class 2

    def label(self):
        p = f"{self.pickup}[{self.pickup_slot}]" if self.pickup_slot is not None else self.pickup
        d = f"{self.drop}[{self.drop_slot}]" if self.drop_slot is not None else self.drop
        return f"#{self.id} {self.what}: {p} -> {d}"


class MissionManager:
    def __init__(self, sim, robot, nav, transfer, qc=None, log=print, spot_pose=None):
        self.sim, self.robot, self.nav, self.transfer, self.qc, self.log = sim, robot, nav, transfer, qc, log
        # where the manager BELIEVES each place is (its map): slot code -> robot pose in front of it.
        # A test can pass a wrong map (e.g. a re-arranged table) to check that the code check catches it.
        self.spot_pose = spot_pose or {c: layout.spot(c)[1] for s in layout.STATIONS for c in layout.spots(s)}
        self.bad_codes = set()      # drop places refused (wrong code read there)
        self.refusals = []          # (time, order id, expected code, code read, codes seen)
        self.charges = []           # one dict per visit to the charging station (Part 8.4)
        self.interventions = []     # (time, order id, spot code): operator called to take a tote by hand
        self.charge_requested = False   # "send to charge" (manual app, Part 8.6)
        self.activity = "IDLE"      # what the robot is doing, for the panel
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

    def cancel(self, order):
        """Remove a WAITING order from the queue (manual app, Part 8.6). Returns True if it was waiting."""
        if order not in self.waiting:
            return False
        self.waiting.remove(order)
        t = self.sim.getSimulationTime()
        order.state, order.result, order.started, order.done = "CANCELLED", "cancelled by the operator", t, t
        self.finished.append(order)
        self.log(f"[orders] CANCELLED {order.label()} (by the operator)")
        return True

    def priority(self, order):
        if order.urgent:
            return CLASS_UNBLOCK            # same class as unblocking the QC cell: before normal orders
        if order.source == "qc" and self.qc is not None and self.qc.blocked is not None \
                and QC_PICKUP[self.qc.blocked] == order.pickup:
            return CLASS_UNBLOCK
        return CLASS_NORMAL

    # ------------------------------------------------------------ coded slots (Part 8.3)
    @property
    def flow(self):
        return getattr(self.sim, "flow", None)

    def pickup_code(self, order):
        """Slot code to take the tote from: the order's own, or from the INVENTORY (Magasin: the slot
        holding what the order asks for, e.g. "bobines rouges"; "bobines" = any reel slot) - with the
        operators, a slot that has a tote now. Stations without slots: their marker id. None = nothing
        to take yet."""
        if order.pickup_slot is not None:   # slot fixed by the order (manual app) or by a first attempt
            if self.flow is not None and order.source != "qc" and not self.flow.occupied(order.pickup_slot):
                return None                 # nothing on it (yet): wait for the restock
            return order.pickup_slot
        codes = layout.spots(order.pickup)
        held = [c for c in codes if c in layout.INVENTORY]
        if held:
            codes = [c for c in held if layout.INVENTORY[c] == order.what or order.what == "bobines"]
        if self.flow is not None and order.source != "qc":
            codes = [c for c in codes if self.flow.occupied(c)]
        return codes[0] if codes else None

    def drop_code(self, order):
        """Slot code to put the tote on: the order's own, or the station's first FREE place (with the
        operators; a delivery still standing there is never covered). None = all places occupied."""
        if order.drop_slot is not None:     # place fixed by the order (manual app): only once it is free
            if order.drop_slot in self.bad_codes or (self.flow is not None and self.flow.occupied(order.drop_slot)):
                return None
            return order.drop_slot
        codes = [c for c in layout.spots(order.drop) if c not in self.bad_codes]
        if self.flow is not None:
            codes = [c for c in codes if not self.flow.occupied(c)]
        return codes[0] if codes else None

    def read_code(self, expected, frames=20):
        """CODE CHECK in front of a slot, before docking: the marker most centred in the camera image
        (the robot stands in front of the slot it believes is `expected`). Returns (ok, code read,
        all codes seen)."""
        seen, read = set(), None
        for _ in range(frames):
            ms = self.transfer.docker.markers.detect(self.transfer.sensors.camera_image())
            if ms:
                seen |= {m.id for m in ms}
                read = min(ms, key=lambda m: abs(m.left)).id
                if read == expected:
                    break
            self.robot.stop()
            self.sim.step()
        return read == expected, read, sorted(seen)

    def ready(self, order):
        """An order can start once there is a tote to take (with the operators of tote_flow: e.g. a reel
        order waits until its Magasin slot has been restocked). QC orders exist only once the UR5 has put
        their tote down."""
        return order.source == "qc" or self.pickup_code(order) is not None

    def next_order(self):
        ready = [o for o in self.waiting if self.ready(o)]
        if not ready:
            return None
        return min(ready, key=lambda o: (self.priority(o), o.created, o.id))

    def _go_to_free_drop(self, order):
        """Drive to a free place of the drop station and check its code; if every place is occupied,
        wait in front of the station until its operator has taken a tote (tote_flow.TAKE_AFTER).
        Returns (code, None) or (None, reason)."""
        t0 = self.sim.getSimulationTime()
        waiting = False
        while True:
            code = self.drop_code(order)
            if code is None:
                if order.drop_slot in self.bad_codes or all(c in self.bad_codes for c in layout.spots(order.drop)):
                    return None, f"no usable place at {order.drop}"
                if not waiting:
                    order.state = "WAIT_DROP"
                    self.log(f"[orders] {'place ' + str(order.drop_slot) if order.drop_slot else 'every place'} "
                             f"at {order.drop} occupied: waiting for its operator")
                    r = self.nav.go_to(self.spot_pose[order.drop_slot or layout.spots(order.drop)[0]])
                    order.legs.append(r)
                    waiting = True
                if self.sim.getSimulationTime() - t0 > DROP_WAIT_MAX:
                    return None, f"{order.drop} never freed"
                self.robot.stop()
                self.sim.step()
                continue
            order.state = "GO_TO_DROP"
            r = self.nav.go_to(self.spot_pose[code])
            order.legs.append(r)
            if not r["reached"]:
                return None, f"{order.drop} not reached"
            ok, read, seen = self.read_code(code)
            if ok:
                return code, None
            # wrong / missing code: never put the tote there; report it and try another place
            self.bad_codes.add(code)
            self.refusals.append((round(self.sim.getSimulationTime(), 1), order.id, code, read, seen))
            self.log(f"[orders] REFUSED drop place {code} for {order.label()}: code read {read} (seen {seen})")

    # ------------------------------------------------------------ execution
    def run(self, stop=lambda manager: False, timeout=900.0):
        """Execute orders until stop(manager) is true, the timeout (simulated s) or a fault.
        With nothing to do, the robot waits where it is (the factory keeps running)."""
        t0 = self.sim.getSimulationTime()
        idle = False
        while not stop(self) and self.fault is None and self.sim.getSimulationTime() - t0 < timeout:
            # class 1: the battery comes first, but only BETWEEN orders (the current one is always finished)
            bat = self.battery
            if bat is not None and bat.level <= 0.0:
                self.fault = "battery empty: robot stopped"
                self.log(f"[battery] {self.fault}")
                break
            if bat is not None and (bat.low or self.charge_requested):
                self.charge("low battery" if bat.low else "requested")
                idle = False
                continue
            order = self.next_order()
            if order is None:
                if not idle:
                    self.log("[orders] nothing to do: waiting")
                    self.activity = "IDLE"
                idle = True
                self.sim.step()
                continue
            idle = False
            self.waiting.remove(order)
            cls = self.priority(order)
            self.log(f"[orders] start {order.label()} (class {cls}, waited "
                     f"{self.sim.getSimulationTime() - order.created:.0f} s, {len(self.waiting)} still waiting)")
            self.execute(order)

    # ------------------------------------------------------------ battery (Part 8.4)
    @property
    def battery(self):
        return getattr(self.sim, "battery", None)

    def charge(self, reason):
        """Go to the charging station (dock, marker 0), dock on it, charge up to battery.FULL, undock.
        Orders created meanwhile (QC cell) wait in the queue."""
        sim, bat = self.sim, self.battery
        rec = dict(reason=reason, start=round(sim.getSimulationTime(), 1), level_in=round(bat.level, 1))
        self.log(f"[battery] {bat.level:.0f} % ({reason}) -> going to the charging station")
        self.activity = "GO_CHARGE"
        r = self.nav.go_to(self.spot_pose[0])
        rec["leg"] = r
        rec["reached"] = r["reached"]
        d = self.transfer.docker.dock(0) if r["reached"] else dict(ok=False, phase="not reached")
        rec["docked"] = d["ok"]
        if not d["ok"]:
            # contacts not reached: charge anyway (a real AMR would retry / call for help) but report it
            self.log(f"[battery] docking on the charging station failed ({d['phase']}): charging in place")
        rec["arrived"] = round(sim.getSimulationTime(), 1)
        self.activity = "CHARGE"
        bat.charging = True
        while bat.level < FULL:
            self.robot.stop()
            sim.step()
        bat.charging = False
        self.charge_requested = False
        rec.update(charged_at=round(sim.getSimulationTime(), 1), level_out=round(bat.level, 1))   # before undocking
        if d["ok"]:
            self.transfer.docker.undock()
        rec["end"] = round(sim.getSimulationTime(), 1)
        self.charges.append(rec)
        self.log(f"[battery] charged to {rec['level_out']:.0f} % in {rec['charged_at'] - rec['arrived']:.0f} s: back to work")
        self.activity = "IDLE"

    def execute(self, order):
        sim = self.sim
        self.current = order
        self.activity = f"ORDER #{order.id}"
        if self.battery is not None:
            order.bat_start = round(self.battery.level, 1)
        order.started = sim.getSimulationTime()
        code = self.pickup_code(order)
        order.pickup_slot = code

        order.state = "GO_TO_PICKUP"
        r = self.nav.go_to(self.spot_pose[code])
        order.legs.append(r)
        if not r["reached"]:
            return self._end(order, "FAILED", f"{order.pickup} not reached")

        # code check before taking anything: the code read must be the slot the order asks for
        ok, read, seen = self.read_code(code)
        if not ok:
            self.refusals.append((round(sim.getSimulationTime(), 1), order.id, code, read, seen))
            return self._end(order, "REFUSED", f"code read {read} in front of slot {code} (codes seen {seen}): "
                                               f"nothing taken, map / labels to check")
        order.state = "LOAD"
        r = self.transfer.load(code)
        if not r["ok"]:
            order.tries += 1
            if order.tries < MAX_TRIES:     # robot still empty, tote still on the table: try again later
                order.state, order.started = "WAITING", None
                self.waiting.append(order)
                self.current = None
                self.log(f"[orders] load failed ({r['info']}): {order.label()} back in the queue "
                         f"(attempt {order.tries + 1} of {MAX_TRIES})")
                return
            why = f"load failed {order.tries} times: {r['info']}"
            if order.source == "qc" and self.qc is not None:
                # the tote would stay on the cell's table and block the QC cell for good: call an operator,
                # who takes it by hand (reported); the UR5 may then use the spot again
                if self.flow is not None and self.flow.take_by_hand(code):
                    why += " - operator called, tote taken by hand"
                    self.interventions.append((round(sim.getSimulationTime(), 1), order.id, code))
                    self.qc.taken(order.pickup == QC_PICKUP[True])
            return self._end(order, "FAILED", why)
        order.loaded = sim.getSimulationTime()
        if order.source == "qc" and self.qc is not None:
            self.qc.taken(order.pickup == QC_PICKUP[True])        # the UR5 may use the spot again

        code, why = self._go_to_free_drop(order)
        if code is None:
            self.fault = f"{order.label()}: {why}, tote still on board"
            return self._end(order, "FAILED", self.fault)
        order.drop_slot = code
        order.state = "UNLOAD"
        r = self.transfer.unload(code)
        if not r["ok"]:
            self.fault = f"{order.label()}: unload failed ({r['info']}), tote still on board"
            return self._end(order, "FAILED", self.fault)
        return self._end(order, "DONE", f"delivered in {sim.getSimulationTime() - order.started:.0f} s")

    def _end(self, order, state, result):
        order.state, order.result, order.done = state, result, self.sim.getSimulationTime()
        if self.battery is not None:
            order.bat_end = round(self.battery.level, 1)
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
            out.append(f"  {o.label():45s} {o.source:6s} {o.state:8s} waited {o.started - o.created:5.0f} s, "
                       f"executed {o.done - o.started:5.0f} s, stops {stops}, re-plans {replans}, contacts {contacts}")
        return out
