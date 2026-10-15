"""Python driver for CoppeliaSim's built-in KUKA youBot (mecanum wheels).

    robot = YouBot(sim)
    robot.move(forward=0.2, left=0.0, turn=0.0)   # m/s, m/s, rad/s, in the ROBOT's frame
    x, y, heading = robot.pose()                   # position from the simulated "UWB + gyro"

Mecanum kinematics: each wheel has small rollers at 45 deg, so combining the 4 wheel speeds
gives any motion in the plane (forward, sideways, rotation, or a mix). For a wheel of radius R
on a robot whose wheels are LX ahead/behind and LY left/right of the centre:
    w_fl = (vx - vy - (LX+LY) wz) / R      w_fr = (vx + vy + (LX+LY) wz) / R
    w_rl = (vx + vy - (LX+LY) wz) / R      w_rr = (vx - vy + (LX+LY) wz) / R
(The youBot model's wheel joints turn the opposite way, hence the minus sign in move().)

Localization (project decision: no wheel odometry): the factory is assumed to have UWB beacons
(position, ~2 cm noise) and the robot a gyro (heading, ~0.5 deg noise). In simulation this is
the true pose of the robot's reference frame plus Gaussian noise.
"""

import math
import random

WHEEL_R = 0.05            # wheel radius (m)
LX, LY = 0.228, 0.158     # wheel offsets from the centre: along / across the robot (m), from the model
UWB_SIGMA = 0.02          # position noise (m)
GYRO_SIGMA = math.radians(0.5)   # heading noise (rad)


class YouBot:
    def __init__(self, sim, path="/youBot", noise=True):
        self.sim = sim
        self.model = sim.getObject(path)
        self.ref = sim.getObject(path + "/youBot_ref")
        self.wheels = [sim.getObject(path + "/rollingJoint_" + w) for w in ("fl", "rl", "rr", "fr")]
        self.noise = noise

    def move(self, forward=0.0, left=0.0, turn=0.0):
        """Drive with a body velocity: forward/left in m/s, turn in rad/s (counter-clockwise)."""
        k = LX + LY
        w = {
            "fl": (forward - left - k * turn) / WHEEL_R,
            "rl": (forward + left - k * turn) / WHEEL_R,
            "rr": (forward - left + k * turn) / WHEEL_R,
            "fr": (forward + left + k * turn) / WHEEL_R,
        }
        for joint, name in zip(self.wheels, ("fl", "rl", "rr", "fr")):
            self.sim.setJointTargetVelocity(joint, -w[name])

    def stop(self):
        self.move(0.0, 0.0, 0.0)

    def true_pose(self):
        """Exact pose from the simulator (x, y, heading). Use only for testing / evaluation."""
        m = self.sim.getObjectMatrix(self.ref, self.sim.handle_world)
        # the robot drives along the +y axis of youBot_ref (measured in tests\test_drive.py),
        # so the heading is the direction of that axis
        return m[3], m[7], math.atan2(m[5], m[1])

    def go_to_step(self, target, pose=None, v_max=0.3, w_max=0.8, k_v=1.2, k_w=1.5, tol=0.03, tol_h=0.03):
        """One control step towards target = (x, y, heading). Proportional controller: the position
        error is rotated into the robot's frame, so the mecanum base drives straight at the target
        (moving and turning at the same time). Returns True when the target is reached."""
        x, y, h = pose or self.pose()
        tx, ty, th = target
        ex, ey = tx - x, ty - y
        eh = math.atan2(math.sin(th - h), math.cos(th - h))
        dist = math.hypot(ex, ey)
        if dist < tol and abs(eh) < tol_h:
            self.stop()
            return True
        fwd = ex * math.cos(h) + ey * math.sin(h)        # error in the robot's frame
        left = -ex * math.sin(h) + ey * math.cos(h)
        v = min(v_max, k_v * dist)
        scale = v / dist if dist > 1e-6 else 0.0
        turn = max(-w_max, min(w_max, k_w * eh))
        self.move(fwd * scale, left * scale, turn)
        return False

    def pose(self):
        """Pose as the robot measures it: UWB position + gyro heading, with realistic noise."""
        x, y, h = self.true_pose()
        if self.noise:
            x += random.gauss(0.0, UWB_SIGMA)
            y += random.gauss(0.0, UWB_SIGMA)
            h += random.gauss(0.0, GYRO_SIGMA)
        return x, y, math.atan2(math.sin(h), math.cos(h))
