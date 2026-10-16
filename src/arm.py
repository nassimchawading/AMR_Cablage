r"""youBot arm (5 joints) + 2-finger gripper: inverse kinematics and pick / place motions.

    arm = YouBotArm(sim)
    arm.pick(fwd, left, z)       # grasp a tote's handle fin centred at (fwd, left, z), robot frame
    arm.place(fwd, left, z)      # put it down so that the fin centre ends at (fwd, left, z)
    arm.fold()                   # back to the transport pose

Geometry (measured on the model, Part 7):
  joint 0 (base, vertical axis) at 0.166 m forward, turns the whole arm plane;
  joints 1-3 (pitch) form a planar 3-link arm in that plane: shoulder 0.033 m from the joint-0
  axis and 0.246 m above the floor, links L1 = 0.155 m, L2 = 0.135 m, then the gripper:
  grasp point L3 from the wrist joint; joint 4 rolls the gripper.
  Angle convention: theta_k = j1 + ... + jk is the tilt of link k from vertical, positive =
  leaning back (towards the robot); link direction in the plane = (-sin theta, cos theta).

Inverse kinematics (closed form, "geometric" method): for a grasp point P and a gripper pitch phi
(angle below horizontal), the wrist must be at W = P - L3 * (cos phi, -sin phi). The shoulder-
elbow-wrist triangle has known sides L1, L2, |SW| -> law of cosines gives the elbow angle
(elbow up). The pitch is chosen as steep as possible while the wrist stays reachable.
"""

import math

J0_POS = (0.1662, 0.0, 0.1447)     # joint-0 axis: forward, left, up (m), robot frame
SHOULDER_R, SHOULDER_Z = 0.033, 0.2459
L1, L2 = 0.155, 0.135
L3 = 0.19                           # wrist joint -> grasp point between the finger pads
# joint limits (rad); CoppeliaSim gives (minimum, range): joint 0 turns +-169 deg only, so the arm
# reaches BEHIND the robot (platform) by leaning back over itself, not by turning around
LIMITS = [(-2.95, 2.95), (-1.57, 1.31), (-2.28, 2.28), (-1.78, 1.78), (-1.57, 1.57)]
FOLDED = [0.0, 0.539, 0.915, 1.269, 0.0]    # library transport pose (arm folded over the platform)
REACH_MAX = L1 + L2 - 0.012         # keep the arm slightly bent (away from the straight-arm singularity)
APPROACH = 0.06                     # m: pre-grasp point back along the gripper axis
LIFT = 0.07                         # m: lift after grasping / before placing
# finger joint: -0.05 = open (pad gap 7.1 cm), 0.0 = closed on nothing (gap 2.1 cm); gap = 0.0213 - joint
EMPTY_CLOSED = 0.0


class Unreachable(Exception):
    pass


def ik(fwd, left, z, phi_max=math.radians(70), phi_min=math.radians(-10)):
    """Joint angles [j0..j4] putting the grasp point at (fwd, left, z), robot frame.
    Tries the steepest gripper pitch first. Raises Unreachable."""
    dx, dy = fwd - J0_POS[0], left - J0_POS[1]
    j0, s = math.atan2(dy, dx), 1          # s = +1: reach out in front of the arm plane, -1: lean back
    if abs(j0) > LIMITS[0][1]:             # target behind: keep the plane, reach backwards over the robot
        j0, s = math.atan2(-dy, -dx), -1
    r = s * math.hypot(dx, dy) - SHOULDER_R  # signed horizontal distance from the shoulder, in the arm plane
    h = z - SHOULDER_Z
    phi = phi_max
    while phi >= phi_min:
        wr, wz = r - s * L3 * math.cos(phi), h + L3 * math.sin(phi)
        d = math.hypot(wr, wz)
        if d <= REACH_MAX and d > abs(L1 - L2) + 0.01:
            t_sw = math.atan2(-wr, wz)                                   # direction shoulder -> wrist
            g = math.acos(max(-1.0, min(1.0, (L1 * L1 + d * d - L2 * L2) / (2 * L1 * d))))
            t1 = t_sw + s * g                                            # elbow up
            er, ez = -L1 * math.sin(t1), L1 * math.cos(t1)
            t2 = math.atan2(-(wr - er), wz - ez)
            t3 = math.atan2(-s * math.cos(phi), -math.sin(phi))          # gripper axis (s cos phi, -sin phi)
            q = [j0, t1, t2 - t1, t3 - t2, 0.0]
            q[3] = math.atan2(math.sin(q[3]), math.cos(q[3]))
            if all(lo <= v <= hi for v, (lo, hi) in zip(q, LIMITS)):
                return q, phi
        phi -= math.radians(2)
    raise Unreachable(f"({fwd:.3f}, {left:.3f}, {z:.3f}) out of reach")


def fk(q):
    """Grasp point (fwd, left, z) for joint angles q (inverse of ik, used by the tests)."""
    t1, t2, t3 = q[1], q[1] + q[2], q[1] + q[2] + q[3]
    r = SHOULDER_R - L1 * math.sin(t1) - L2 * math.sin(t2) - L3 * math.sin(t3)
    z = SHOULDER_Z + L1 * math.cos(t1) + L2 * math.cos(t2) + L3 * math.cos(t3)
    return J0_POS[0] + r * math.cos(q[0]), J0_POS[1] + r * math.sin(q[0]), z


class YouBotArm:
    def __init__(self, sim, robot_path="/youBot", log=print):
        self.sim, self.log = sim, log
        self.joints = [sim.getObject(f"{robot_path}/youBotArmJoint{i}") for i in range(5)]
        self.finger = sim.getObject(robot_path + "/youBotGripperJoint2")
        script = None
        for s in sim.getObjectsInTree(sim.getObject(robot_path), sim.sceneobject_script, 0):
            txt = sim.getObjectStringParam(s, sim.scriptstringparam_text)
            txt = txt.decode() if isinstance(txt, bytes) else txt
            if "youBotGripperJoint2" in txt:
                script = s
        self.gripper_script = script
        self.q = [sim.getJointPosition(j) for j in self.joints]
        self.pads = [sim.getObject(robot_path + "/Rectangle5"), sim.getObject(robot_path + "/Rectangle6")]
        self.palm = sim.getObject(robot_path + "/Rectangle7")     # last arm link (carries the fingers)
        self.held = None
        self._loose = None

    # ------------------------------------------------------------ low level
    def _step(self):
        self.sim.step()

    def move_joints(self, target, duration=None, vmax=0.8):
        """Smooth joint-space motion (all joints start and stop together), then wait to settle."""
        start = list(self.q)
        span = max(abs(t - s) for t, s in zip(target, start))
        duration = duration or max(0.5, span / vmax)
        dt = self.sim.getSimulationTimeStep()
        n = max(1, int(round(duration / dt)))
        for k in range(1, n + 1):
            s = k / n
            s = s * s * (3 - 2 * s)                                     # smoothstep: no jerk at both ends
            for j, a, b in zip(self.joints, start, target):
                self.sim.setJointTargetPosition(j, a + s * (b - a))
            self._step()
        self.q = list(target)
        for _ in range(40):                                             # wait until the joints are there (2 s max)
            if max(abs(self.sim.getJointPosition(j) - t) for j, t in zip(self.joints, target)) < 0.01:
                break
            self._step()

    def gripper(self, close, wait=1.6):
        self.sim.callScriptFunction("setGripper", self.gripper_script, bool(not close))
        for _ in range(int(wait / self.sim.getSimulationTimeStep())):
            self._step()

    def opening(self):
        """Finger joint position: -0.05 = fully open (gap ~7 cm); larger = more closed."""
        return self.sim.getJointPosition(self.finger)

    def go(self, fwd, left, z, phi=None, duration=None):
        q, _ = ik(fwd, left, z) if phi is None else ik(fwd, left, z, phi, phi)
        self.move_joints(q, duration)
        return q

    # ------------------------------------------------------------ tasks
    def _approach_point(self, fwd, left, z, phi):
        """Pre-grasp point: APPROACH metres closer to the arm base, same height. The open fingers
        then slide horizontally along the fin (the fin points at the arm) - this keeps the wrist
        inside its reach, unlike backing off along the tilted gripper axis."""
        a = math.atan2(left - J0_POS[1], fwd - J0_POS[0])
        return fwd - APPROACH * math.cos(a), left - APPROACH * math.sin(a), z

    # ------------------------------------------------------------ holding the tote
    # Two fingertips on a small post hold a tote well against sliding, but the physics engine lets it
    # PIVOT around the post (measured: 17 deg after lifting, 120 deg when swung over the platform,
    # then it fell). So once the fingers have closed on the post (the finger opening proves it), the
    # tote touching the pads is LOCKED to the gripper: it becomes a rigid part of the hand until
    # release, then a normal physical object again (it settles on the surface by itself).
    def _candidates(self):
        """Loose objects of the workshop (dynamic shapes): the only things the hand can pick up."""
        if self._loose is None:
            shapes = self.sim.getObjectsInTree(self.sim.getObject("/Atelier"), self.sim.sceneobject_shape, 0)
            self._loose = [s for s in shapes if self.sim.getObjectInt32Param(s, self.sim.shapeintparam_static) == 0]
        return self._loose

    def _lock(self):
        for c in self._candidates():
            res = self.sim.checkDistance(c, self.pads[0], 0.01)
            if res[0] > 0 and res[1][6] < 0.004:                     # touching the finger pad
                self.held = (c, self.sim.getObjectParent(c))
                self.sim.setObjectInt32Param(c, self.sim.shapeintparam_static, 1)
                self.sim.setObjectInt32Param(c, self.sim.shapeintparam_respondable, 0)
                self.sim.setObjectParent(c, self.palm, True)
                self.sim.resetDynamicObject(c)
                return True
        return False

    def _unlock(self):
        if self.held:
            c, parent = self.held
            self.sim.setObjectParent(c, parent, True)
            self.sim.setObjectInt32Param(c, self.sim.shapeintparam_static, 0)
            self.sim.setObjectInt32Param(c, self.sim.shapeintparam_respondable, 1)
            self.sim.resetDynamicObject(c)
            self.held = None

    def pick(self, fwd, left, z, phi=None):
        """Grasp the post centred at (fwd, left, z) and lift it. Returns True if a tote is held.
        phi: gripper pitch (rad); use the SAME pitch to place the tote, so that it lands level."""
        q, phi = ik(fwd, left, z) if phi is None else ik(fwd, left, z, phi, phi)
        self.gripper(close=False, wait=0.3)
        self.go(*self._approach_point(fwd, left, z, phi), phi=phi)
        self.move_joints(q, 1.0)
        self.gripper(close=True)
        held = self.opening() < EMPTY_CLOSED - 0.004 and self._lock()   # fingers stopped by the post (~-0.009)
        self.go(fwd, left, z + LIFT, duration=1.0)        # free pitch in between: the tote is locked
        return held

    def place(self, fwd, left, z, phi=None):
        """Lower the held tote so that the post centre ends at (fwd, left, z), release, back off."""
        q, phi = ik(fwd, left, z) if phi is None else ik(fwd, left, z, phi, phi)
        self.go(fwd, left, z + LIFT)
        self.move_joints(q, 1.0)
        self._unlock()
        self.gripper(close=False)
        self.go(*self._approach_point(fwd, left, z, phi), phi=phi, duration=0.8)

    def fold(self):
        self.move_joints(list(FOLDED))
