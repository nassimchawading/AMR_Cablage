r"""Part 8.2 - the people around the AMR: totes arriving at the QC cell, delivered totes taken away.

    sim = Factory(client, flow=True)       # Factory runs ToteFlow.tick() before every simulation step
    sim.flow.occupied("expedition")        # is a tote still on that station's table?

What the "operators" do (simulated, no walking person: totes are moved directly):
  1. TOTE STREAM: every `interval` seconds an operator puts a harness tote at the START of the QC
     conveyor. Its QC result follows a FIXED repeating sequence (QC_SEQUENCE: OK, OK, DEFECT, ...), so
     two runs are the same (Part 11 measurements). The camera still decides from the tag colour: the
     flow only paints the tag. The 3 totes the scene starts with on the belt go to the reserve first:
     the stream alone feeds the cell, slowly enough that QC work leaves the robot time for its other
     errands (user, 2026-11-05).
  2. TAKING DELIVERED TOTES: a tote resting on a CONSUMER station's table (Coupe, Sertissage,
     Assemblage, Expedition) for TAKE_AFTER seconds is taken by the station's operator (the table is
     free again for the next delivery).
  3. RESTOCKING THE MAGASIN: when the Magasin table has been empty for RESTOCK_AFTER seconds and a reel
     tote is in the reserve, the storekeeper puts one back (a reel order can then run again).
Totes that are "taken" go to a hidden reserve (under the floor, outside the hall): static, not
respondable, invisible to the sensors. The stream and the storekeeper reuse them: the scene has a
fixed pool of totes (the 3 on the belt + layout.QC_SPARES spares + the Magasin reel tote).
"""

import re

import layout

QC_SEQUENCE = (True, True, False)      # QC result of successive stream totes (True = OK), repeated
QC_INTERVAL = 400.0                    # s between two totes: a QC order keeps the robot ~135 s (8.1) -> ~1/3 of its time
FIRST_TOTE = 10.0                      # s after the start: first tote on the belt
TAKE_AFTER = 75.0                      # s a delivered tote stays on a consumer table
RESTOCK_AFTER = 30.0                   # s the Magasin table stays empty before it is restocked
CONSUMERS = ("coupe", "sertissage", "assemblage", "expedition")
CHECK_EVERY = 10                       # steps between two looks at the tables (0.5 s)
RESERVE_X, RESERVE_Y, RESERVE_Z = layout.RESERVE
REST_Z = layout.TABLE_H + layout.TOTE[2] + layout.FIN[2] / 2      # tote origin (post centre) on a table
SPAWN_FREE = 0.45                      # m: no tote this close to the conveyor start -> room for a new one
GREEN, RED = layout.C["green"], layout.C["red"]


# every place a tote can stand on a station table: code (slot code or station marker id) -> (station, (x, y))
SPOTS = {c: (s, layout.spot(c)[2]) for s in layout.STATIONS if s != "dock" for c in layout.spots(s)}


class ToteFlow:
    def __init__(self, sim, interval=QC_INTERVAL, sequence=QC_SEQUENCE, first=FIRST_TOTE,
                 take_after=TAKE_AFTER, restock_after=RESTOCK_AFTER, stream=True, count=None, log=print):
        self.sim, self.interval, self.sequence, self.log = sim, interval, sequence, log
        self.take_after, self.restock_after, self.stream, self.count = take_after, restock_after, stream, count
        self.sensed = (sim.objectspecialproperty_collidable | sim.objectspecialproperty_measurable
                       | sim.objectspecialproperty_detectable)
        self.totes, self.tags, self.content = [], {}, {}
        for h, name in scene_totes(sim):
            self.totes.append(h)
            self.content[h] = "bobines" if name.startswith("bac_magasin") else "faisceau"
            kids = sim.getObjectsInTree(h, sim.sceneobject_shape, 1)
            tag = [k for k in kids if "etiquette" in sim.getObjectAlias(k)]
            if tag:
                self.tags[h] = tag[0]
        belt = [h for h in self.totes if sim.getObjectAlias(h).startswith("bac_faisceau")]
        # where an operator puts a new tote: the place of the last tote on the belt at the start
        start = min(belt, key=lambda h: sim.getObjectPosition(h, sim.handle_world)[0])
        self.spawn_matrix = sim.getObjectMatrix(start, sim.handle_world)
        self.reserve = {"faisceau": [], "bobines": []}
        self.slots = {}                      # tote -> its place in the reserve (fixed, no overlap)
        for i, h in enumerate(self.totes):
            self.slots[h] = (RESERVE_X - 0.3 * i, RESERVE_Y, RESERVE_Z)
        for h in self.totes:
            if sim.getObjectPosition(h, sim.handle_world)[2] < -0.5:      # spares: already in the reserve
                self._park(h, quiet=True)
            elif stream and h in belt:                                    # stream mode: the belt starts empty
                self._park(h, quiet=True)
        self.k = 0                           # number of stream totes so far
        self.next_spawn = sim.getSimulationTime() + first
        self.resting = {}                    # tote -> (spot code, time it was first seen resting there)
        # each reel tote belongs to the Magasin slot it starts on (its reel colour = that slot's content)
        self.home = {h: self._resting_on(h) for h in self.totes if self.content[h] == "bobines"}
        self.empty_since = {}                # Magasin slot -> empty since (s)
        self.steps = 0
        self.on_table = {}                   # spot code -> totes resting there (last look)
        self.stats = dict(spawned=0, taken=0, restocked=0, events=[])

    # ------------------------------------------------------------ helpers
    def event(self, msg):
        self.stats["events"].append((round(self.sim.getSimulationTime(), 1), msg))
        self.log(f"  [{self.sim.getSimulationTime():6.1f} s] operators: {msg}")

    def _park(self, h, quiet=False):
        sim = self.sim
        sim.setObjectInt32Param(h, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 0)
        sim.setObjectSpecialProperty(h, 0)
        sim.setObjectOrientation(h, [0.0, 0.0, 0.0], sim.handle_world)
        sim.setObjectPosition(h, list(self.slots[h]), sim.handle_world)
        sim.resetDynamicObject(h)
        if h not in self.reserve[self.content[h]]:
            self.reserve[self.content[h]].append(h)

    def _release(self, h, matrix):
        """Take a tote out of the reserve and put it down (dynamic again, seen by the sensors)."""
        sim = self.sim
        self.reserve[self.content[h]].remove(h)
        sim.setObjectMatrix(h, matrix, sim.handle_world)
        sim.setObjectSpecialProperty(h, self.sensed)
        sim.setObjectInt32Param(h, sim.shapeintparam_static, 0)
        sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 1)
        sim.resetDynamicObject(h)

    def _resting_on(self, h):
        """Spot code (slot or station place) this tote rests on (free: not held, not clamped, upright),
        or None."""
        sim = self.sim
        if sim.getObjectInt32Param(h, sim.shapeintparam_static):
            return None                      # in the reserve, in the gripper or clamped on the robot
        x, y, z = sim.getObjectPosition(h, sim.handle_world)
        if abs(z - REST_Z) > 0.02 or sim.getObjectMatrix(h, sim.handle_world)[10] < 0.98:
            return None
        for code, (_, (tx, ty)) in SPOTS.items():
            if abs(x - tx) < 0.12 and abs(y - ty) < 0.2:
                return code
        return None

    def occupied(self, where):
        """True if a tote stands on this spot (slot code / station marker id), or on any spot of this
        station (name). The robot must not put a tote on an occupied spot; a pickup waits for a tote.
        Updated every CHECK_EVERY steps."""
        if isinstance(where, str):
            return any(self.on_table.get(c) for c in layout.spots(where))
        return bool(self.on_table.get(where))

    def take_by_hand(self, code):
        """An operator takes away the tote(s) standing on this spot (called by the mission manager when
        the robot could not take it). Returns how many totes were taken."""
        totes = [h for h in self.totes if self._resting_on(h) == code]
        for h in totes:
            self._park(h)
        self.on_table.pop(code, None)
        if totes:
            self.stats["by_hand"] = self.stats.get("by_hand", 0) + len(totes)
            self.event(f"operator called: tote taken BY HAND from place {code}")
        return len(totes)

    def free_spot(self, station):
        """First free place of a station (slot code / marker id), or None if all are occupied."""
        return next((c for c in layout.spots(station) if not self.on_table.get(c)), None)

    # ------------------------------------------------------------ every step
    def tick(self):
        t = self.sim.getSimulationTime()
        if self.stream and t >= self.next_spawn and (self.count is None or self.k < self.count):
            self._spawn(t)
        self.steps += 1
        if self.steps % CHECK_EVERY:
            return
        self.on_table = {}
        for h in self.totes:
            where = self._resting_on(h)
            if where is not None:
                self.on_table.setdefault(where, []).append(h)
            if where is not None and SPOTS[where][0] in CONSUMERS:
                st, since = self.resting.get(h, (where, t))
                if st != where:
                    since = t
                self.resting[h] = (where, since)
                if t - since >= self.take_after:
                    del self.resting[h]
                    self._park(h)
                    self.stats["taken"] += 1
                    self.event(f"tote taken from the {SPOTS[where][0]} table (place {where})")
            else:
                self.resting.pop(h, None)
        for code in layout.spots("magasin"):
            if self.on_table.get(code):
                self.empty_since.pop(code, None)
                continue
            since = self.empty_since.setdefault(code, t)
            back = [h for h in self.reserve["bobines"] if self.home.get(h) == code]
            if t - since >= self.restock_after and back:
                x, y = SPOTS[code][1]
                m = [1.0, 0.0, 0.0, x, 0.0, 1.0, 0.0, y, 0.0, 0.0, 1.0, REST_Z + 0.002]
                self._release(back[0], m)
                self.empty_since.pop(code, None)
                self.stats["restocked"] += 1
                self.event(f"Magasin slot {code} restocked ({layout.INVENTORY.get(code, 'bobines')})")

    def _spawn(self, t):
        sim = self.sim
        x0 = self.spawn_matrix[3]
        busy = any(abs(sim.getObjectPosition(h, sim.handle_world)[0] - x0) < SPAWN_FREE
                   and abs(sim.getObjectPosition(h, sim.handle_world)[1] - layout.QC_Y) < 0.3
                   for h in self.totes if h not in self.reserve[self.content[h]])
        if busy or not self.reserve["faisceau"]:
            self.next_spawn = t + 5.0        # belt start occupied (queue backed up) or no tote left: later
            if not self.reserve["faisceau"]:
                self.event("no empty harness tote left in the reserve: stream waits")
            return
        ok = self.sequence[self.k % len(self.sequence)]
        h = self.reserve["faisceau"][0]
        sim.setShapeColor(self.tags[h], None, sim.colorcomponent_ambient_diffuse, list(GREEN if ok else RED))
        self._release(h, self.spawn_matrix)
        self.k += 1
        self.stats["spawned"] += 1
        self.next_spawn = t + self.interval
        self.event(f"tote {self.k} put on the QC conveyor ({'OK' if ok else 'DEFECT'} harness), "
                   f"next in {self.interval:.0f} s")


TOTE_NAME = re.compile(r"^bac_(faisceau|reserve|magasin)(_\d+)?$")


def scene_totes(sim):
    """[(handle, alias)] of every transport tote of the workshop (belt, spares, Magasin)."""
    root = sim.getObject("/Atelier")
    out = []
    for h in sim.getObjectsInTree(root, sim.sceneobject_shape, 0):
        name = sim.getObjectAlias(h)
        if TOTE_NAME.match(name):
            out.append((h, name))
    return out


def restore_scene(sim):
    """Put the totes back in their build state at every start: the flow repaints tags and switches
    totes between dynamic and static, and CoppeliaSim restores poses when the simulation stops but not
    colours (nor, to be safe, these flags). Spares (under the floor) static and hidden, the others dynamic
    and seen by the sensors; belt tote tags painted as in layout.QC_TOTES."""
    want = {"bac_faisceau" + (f"_{i}" if i else ""): ok for i, (_, ok) in enumerate(layout.QC_TOTES)}
    sensed = (sim.objectspecialproperty_collidable | sim.objectspecialproperty_measurable
              | sim.objectspecialproperty_detectable)
    for h, name in scene_totes(sim):
        spare = sim.getObjectPosition(h, sim.handle_world)[2] < -0.5
        sim.setObjectInt32Param(h, sim.shapeintparam_static, 1 if spare else 0)
        sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 0 if spare else 1)
        sim.setObjectSpecialProperty(h, 0 if spare else sensed)
        if name in want:
            for k in sim.getObjectsInTree(h, sim.sceneobject_shape, 1):
                if "etiquette" in sim.getObjectAlias(k):
                    sim.setShapeColor(k, None, sim.colorcomponent_ambient_diffuse,
                                      list(GREEN if want[name] else RED))
