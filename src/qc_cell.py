r"""Quality-control cell: camera decision + UR5 sorting (Part 9).

    cell = QCCell(sim, simIK)
    while ...:
        cell.tick()          # once per simulation step, before sim.step()
    (or let it run in the background of any program: see src\factory.py)

Flow of a harness tote:
  1. it rides the inspection conveyor under the QC camera (looking down). OpenCV finds the QC tag
     on its lid by colour (HSV thresholds): green = OK, red = defect. The decision is taken when the
     tag crosses the middle of the image, and pushed into a FIFO queue (totes cannot overtake each
     other on a belt, so the queue order = arrival order at the arm).
  2. it stops against the end stop. A light barrier (ray sensor across the belt) sees it; the
     measured distance also gives its exact sideways position on the belt.
  3. the UR5 takes the oldest decision from the queue, picks the tote by its handle post with the
     suction pad, and puts it on the OK table (-> Expédition) or the REBUTS table (-> rework):
     first on the front spot (where the AMR picks it up), then on the buffer spot behind it.

UR5 motion: the suction pad moves along STRAIGHT LINES between waypoints. At every simulation step
the target is moved a little along the line and CoppeliaSim's IK solver (simIK, damped least
squares) computes the joint angles, sent to the arm's position-controlled motors. The pad always
points down with the same yaw, so the tote keeps its orientation.
"""

import math

import cv2
import numpy as np

import layout

HOME_DEG = [-20, 83, -67, -107, 90, -20]        # pad down above the conveyor end (IK, Part 9)
SPEED = 0.30                                    # m/s, pad speed along a line
CLEAR_Z = 0.68                                  # travel height between conveyor and tables
BELT_POST_TOP = 0.56                            # post top of a tote on the belt (measured: post centre 0.535)
TABLE_POST_TOP = layout.TABLE_H + layout.TOTE[2] + layout.FIN[2]
END_X = layout.QC_X1 - layout.TOTE[0] / 2       # tote centre against the end stop
BARRIER_Y0 = layout.QC_Y - 0.30                 # light barrier position (see build_scene.qc_cell_extras)
BARRIER_MAX = 0.40                              # beyond this the ray sees the far side of the belt, not a tote
TABLES = {True: layout.TEST_DROP_X, False: layout.REBUTS_X}
# HSV thresholds (OpenCV: H 0-180). Green tag ~ H 65, red tag ~ H 0/180; the tote is blue (H ~110)
GREEN = ((45, 120, 80), (85, 255, 255))
RED = [((0, 120, 80), (8, 255, 255)), ((172, 120, 80), (180, 255, 255))]
MIN_AREA = 40                                   # px, tag blob (a 5 cm tag is ~400 px in the 256 x 256 image)
BAND = 25                                       # px: "in the middle of the image" band (half width)


def find_tags(img):
    """QC tags in a BGR image: list of (ok, u, v, area)."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    masks = {True: cv2.inRange(hsv, *GREEN),
             False: cv2.bitwise_or(*[cv2.inRange(hsv, lo, hi) for lo, hi in RED])}
    out = []
    for ok, mask in masks.items():
        n, _, stats, cent = cv2.connectedComponentsWithStats(mask)
        for i in range(1, n):
            if stats[i, cv2.CC_STAT_AREA] >= MIN_AREA:
                out.append((ok, float(cent[i][0]), float(cent[i][1]), int(stats[i, cv2.CC_STAT_AREA])))
    return out


class QCCell:
    def __init__(self, sim, simIK, log=print):
        self.sim, self.simIK, self.log = sim, simIK, log
        self.ur5 = sim.getObject("/Atelier/ur5")
        self.joints = sim.getObjectsInTree(self.ur5, sim.sceneobject_joint, 0)
        self.tip = sim.getObject("/Atelier/ur5/tip")
        self.target = sim.getObject("/Atelier/ur5/target")
        self.pad_script = sim.getObjectsInTree(sim.getObject("/Atelier/ur5/suctionPad"), sim.sceneobject_script, 0)[0]
        self.pad_link = sim.getObject("/Atelier/ur5/suctionPad/LoopClosureDummy1")
        self.pad = sim.getObject("/Atelier/ur5/suctionPad")
        self.barrier = sim.getObject("/Atelier/barriere_fin_convoyeur")
        self.conveyor = sim.getObject("/Atelier/convoyeur_controle")
        self.belt_on = True
        self.camera = sim.getObjectsInTree(sim.getObject("/Atelier/camera_controle_qualite"), sim.sceneobject_visionsensor, 0)[0]
        # IK runs on a separate kinematic copy of the arm ("IK world"), which always continues from its
        # own last solution; only the resulting joint angles go to the real (dynamic) arm's motors.
        # (Re-reading the moving physical arm at every step made the solver diverge.)
        self.env = simIK.createEnvironment()
        self.group = simIK.createGroup(self.env)
        simIK.setGroupCalculation(self.env, self.group, simIK.method_damped_least_squares, 0.02, 99)
        _, s2i, _ = simIK.addElementFromScene(self.env, self.group, self.ur5, self.tip, self.target, simIK.constraint_pose)
        self.ik_target, self.ik_tip = s2i[self.target], s2i[self.tip]
        self.ik_joints = [s2i[j] for j in self.joints]
        self.queue = []                 # QC decisions (True = OK), oldest first
        self.in_band = False
        self.used = {True: 0, False: 0}  # spots used on each table
        self.stats = dict(inspected=0, ok=0, defect=0, sorted=0, failed=0, dropped=0, events=[])
        self.last_image = None
        self.job = None                 # current arm job (generator), advanced one step per tick
        self.full_reported = False
        self.dropped = False

    # ------------------------------------------------------------ sensors
    def camera_image(self):
        img, (w, h) = self.sim.getVisionSensorImg(self.camera)
        img = np.frombuffer(img, dtype=np.uint8).reshape(h, w, 3)
        return np.ascontiguousarray(img[::-1, :, ::-1])

    def inspect(self):
        """Camera step: record a decision when a tag crosses the middle of the image."""
        img = self.camera_image()
        self.last_image = img
        mid = img.shape[1] / 2
        tags = [t for t in find_tags(img) if abs(t[1] - mid) < BAND]
        if tags and not self.in_band:
            ok = max(tags, key=lambda t: t[3])[0]
            self.queue.append(ok)
            self.stats["inspected"] += 1
            self.stats["ok" if ok else "defect"] += 1
            self.event(f"camera: tote {self.stats['inspected']} -> {'OK (green tag)' if ok else 'DEFECT (red tag)'}")
        self.in_band = bool(tags)

    def tote_waiting(self):
        """Light barrier: (True, tote centre y) if a tote stands at the end stop."""
        res = self.sim.readProximitySensor(self.barrier)
        if res and res[0] > 0 and res[1] < BARRIER_MAX:
            return True, BARRIER_Y0 + res[1] + layout.TOTE[1] / 2
        return False, None

    # ------------------------------------------------------------ arm
    def event(self, msg):
        self.stats["events"].append((round(self.sim.getSimulationTime(), 1), msg))
        self.log(f"  [{self.sim.getSimulationTime():6.1f} s] {msg}")

    def suction(self, on):
        self.sim.callScriptFunction("setSuction", self.pad_script, bool(on))

    def belt(self, on):
        """Start / stop the conveyor (library conveyor: velocity command in its '__ctrl__' data)."""
        if on != self.belt_on:
            self.sim.setBufferProperty(self.conveyor, "customData.__ctrl__",
                                       self.sim.packTable({"vel": layout.QC_SPEED if on else 0.0}))
            self.belt_on = on
            self.event("conveyor " + ("restarted" if on else "stopped: tote at the end stop"))

    def holding(self):
        return self.sim.getObjectParent(self.pad_link) != self.pad

    def _pose(self, p):
        """Target pose: pad face at p, pointing down (tip +z up), fixed yaw (x along the world x)."""
        return [1.0, 0.0, 0.0, p[0], 0.0, 1.0, 0.0, p[1], 0.0, 0.0, 1.0, p[2]]

    def move_to(self, p, speed=SPEED, carrying=False):
        """Straight line of the pad to p, ONE simulation step per iteration (generator: the caller
        steps the simulation, so the cell never blocks the rest of the factory).
        carrying=True: check at every step that the tote is still on the pad; if it fell, stop the
        move and set self.dropped (the job then goes back for it)."""
        m = self.simIK.getObjectMatrix(self.env, self.ik_tip)
        start = (m[3], m[7], m[11])
        n = max(1, int(math.dist(start, p) / (speed * self.sim.getSimulationTimeStep())))
        for k in range(1, n + 1):
            s = k / n
            s = s * s * (3 - 2 * s)
            q = [a + s * (b - a) for a, b in zip(start, p)]
            self.simIK.setObjectMatrix(self.env, self.ik_target, self._pose(q))
            self.simIK.handleGroup(self.env, self.group)
            for j, ij in zip(self.joints, self.ik_joints):
                self.sim.setJointTargetPosition(j, self.simIK.getJointPosition(self.env, ij))
            yield
            if carrying and not self.holding():
                self.dropped = True
                v = [round(math.degrees(self.sim.getJointVelocity(j))) for j in self.joints]
                tp = self.sim.getObjectPosition(self.tip, self.sim.handle_world)
                self.event(f"UR5: TOTE DROPPED at pad ({tp[0]:.2f}, {tp[1]:.2f}, {tp[2]:.2f}), "
                           f"joint speeds {v} deg/s, {k}/{n} of the move")
                return
        yield from self.wait(0.5)               # let the motors catch up

    def wait(self, seconds):
        for _ in range(int(seconds / self.sim.getSimulationTimeStep())):
            yield

    # ------------------------------------------------------------ cell logic
    def tick(self):
        """Call ONCE PER SIMULATION STEP (before sim.step()): the camera inspects, and the arm advances
        its current job by one step, or starts sorting a tote waiting at the end stop."""
        self.inspect()
        waiting, y = self.tote_waiting()
        if waiting and self.belt_on:
            # accumulation stop: the belt waits while the arm works, so nothing pushes the tote being
            # picked, and the next totes cannot shove each other out of order
            self.belt(False)
        if self.job is not None:
            try:
                next(self.job)
            except StopIteration:
                self.job = None
            return
        if waiting and self.queue:
            self.job = self._sort(y)

    def _sort(self, y):
        ok = self.queue.pop(0)
        slot = self.used[ok]
        if slot >= len(layout.QC_SLOTS_DY):
            if not self.full_reported:
                self.event(f"{'OK' if ok else 'REBUTS'} table full: waiting for the AMR")
                self.full_reported = True
            self.queue.insert(0, ok)
            yield from self.wait(1.0)
            return
        self.full_reported = False
        tx, ty = TABLES[ok], layout.DROP_BOT_Y + layout.QC_SLOTS_DY[slot]
        self.event(f"UR5: tote at the end stop (y {y:.3f}) -> {'OK' if ok else 'REBUTS'} table, spot {slot + 1}")
        yield from self.move_to((END_X, y, CLEAR_Z))
        yield from self.move_to((END_X, y, BELT_POST_TOP + 0.03), speed=0.15)
        self.suction(True)
        # the pad's sensor only detects 1-5 mm below its face (measured), and the post's height varies
        # by a few mm (totes jostle at the end stop): sweep slowly down THROUGH that window to light
        # contact; the pad latches on wherever the post really is
        yield from self.move_to((END_X, y, BELT_POST_TOP + 0.008), speed=0.05)
        yield from self.move_to((END_X, y, BELT_POST_TOP - 0.002), speed=0.01)
        yield from self.wait(0.3)
        if not self.holding():
            self.suction(False)
            yield from self.move_to((END_X, y, CLEAR_Z))
            self.stats["failed"] += 1
            self.queue.insert(0, ok)
            self.event("UR5: suction failed, retrying")
            return
        self.dropped = False
        for target, speed in (((END_X, y, CLEAR_Z), 0.2), ((tx, ty, CLEAR_Z), SPEED),
                              ((tx, ty, TABLE_POST_TOP + 0.03), 0.15), ((tx, ty, TABLE_POST_TOP + 0.004), 0.05)):
            yield from self.move_to(target, speed, carrying=True)
            if target[0] == END_X and self.belt_on is False and not self.dropped:
                self.belt(True)                  # tote lifted off the belt: bring the next one
            if self.dropped:                     # it falls back on the belt and rides to the end stop:
                self.suction(False)              # go back to the start position and pick it again
                self.stats["dropped"] += 1
                self.queue.insert(0, ok)
                yield from self.move_to((END_X, layout.QC_Y, CLEAR_Z))
                return
        self.suction(False)
        yield from self.wait(0.3)
        yield from self.move_to((tx, ty, CLEAR_Z), speed=0.2)
        yield from self.move_to((END_X, layout.QC_Y, CLEAR_Z))
        self.used[ok] += 1
        self.stats["sorted"] += 1
        self.event(f"UR5: tote placed on the {'OK' if ok else 'REBUTS'} table")
