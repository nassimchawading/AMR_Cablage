r"""Quality-control cell: camera decision + UR5 sorting (Part 9).

    cell = QCCell(sim, simIK)
    while ...:
        cell.update()        # one decision per simulation step (call before sim.step())

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
        self.stats = dict(inspected=0, ok=0, defect=0, sorted=0, failed=0, events=[])
        self.last_image = None

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

    def holding(self):
        return self.sim.getObjectParent(self.pad_link) != self.pad

    def _pose(self, p):
        """Target pose: pad face at p, pointing down (tip +z up), fixed yaw (x along the world x)."""
        return [1.0, 0.0, 0.0, p[0], 0.0, 1.0, 0.0, p[1], 0.0, 0.0, 1.0, p[2]]

    def move_to(self, p, speed=SPEED, on_step=None):
        """Straight line of the pad to p; the camera keeps inspecting meanwhile (on_step)."""
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
            (on_step or self.inspect)()
            self.sim.step()
        for _ in range(10):                     # let the motors catch up
            self.inspect()
            self.sim.step()

    def wait(self, seconds):
        for _ in range(int(seconds / self.sim.getSimulationTimeStep())):
            self.inspect()
            self.sim.step()

    # ------------------------------------------------------------ cell logic
    def update(self):
        """Call once per simulation step: inspect, and sort a waiting tote if there is one."""
        self.inspect()
        waiting, y = self.tote_waiting()
        if waiting and self.queue:
            self.sort_one(y)

    def sort_one(self, y):
        ok = self.queue.pop(0)
        slot = self.used[ok]
        if slot >= len(layout.QC_SLOTS_DY):
            self.event(f"{'OK' if ok else 'REBUTS'} table full: waiting for the AMR")
            self.queue.insert(0, ok)
            return
        tx, ty = TABLES[ok], layout.DROP_BOT_Y + layout.QC_SLOTS_DY[slot]
        self.event(f"UR5: tote at the end stop (y {y:.3f}) -> {'OK' if ok else 'REBUTS'} table, spot {slot + 1}")
        self.move_to((END_X, y, CLEAR_Z))
        self.move_to((END_X, y, BELT_POST_TOP + 0.03), speed=0.15)
        self.suction(True)
        self.move_to((END_X, y, BELT_POST_TOP - 0.003), speed=0.05)
        self.wait(0.3)
        if not self.holding():
            self.suction(False)
            self.move_to((END_X, y, CLEAR_Z))
            self.stats["failed"] += 1
            self.queue.insert(0, ok)
            self.event("UR5: suction failed, retrying")
            return
        self.move_to((END_X, y, CLEAR_Z), speed=0.2)
        self.move_to((tx, ty, CLEAR_Z))
        self.move_to((tx, ty, TABLE_POST_TOP + 0.03), speed=0.15)
        self.move_to((tx, ty, TABLE_POST_TOP + 0.004), speed=0.05)
        self.suction(False)
        self.wait(0.3)
        self.move_to((tx, ty, CLEAR_Z), speed=0.2)
        self.move_to((END_X, layout.QC_Y, CLEAR_Z))
        self.used[ok] += 1
        self.stats["sorted"] += 1
        self.event(f"UR5: tote placed on the {'OK' if ok else 'REBUTS'} table")
