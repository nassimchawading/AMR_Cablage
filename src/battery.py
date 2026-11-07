r"""Part 8.4 - simulated battery of the AMR (state of charge in %), drained by its activity.

    sim = Factory(client, battery=dict(level=60.0))   # Factory updates it before every simulation step
    sim.battery.level, sim.battery.low, sim.battery.charging

Simple model (the youBot model has no battery; a real AMR reports its state of charge):
    drain = IDLE_DRAIN x time  +  DRIVE_DRAIN x distance driven  +  ARM_DRAIN x arm joint motion
Distance = true displacement of the robot base, arm motion = sum of |joint angle changes| (rad), both
measured every CHECK_EVERY steps. While `charging` is set (the robot stands docked at the charging
station, set by the mission manager), the level rises at CHARGE_RATE instead.
The rates are chosen so that a shift shows the auto charge: ~75 % used in ~20 min of work (~10 orders),
and an ACCELERATED charge (~2 min from 25 % to 95 %; a real one takes about an hour).
"""

import math

IDLE_DRAIN = 0.02        # %/s   electronics, sensors, computer (always)
DRIVE_DRAIN = 0.15       # %/m   wheels
ARM_DRAIN = 0.03         # %/rad arm joints (sum over the 5 joints)
CHARGE_RATE = 0.6        # %/s   accelerated charge
LOW = 25.0               # %  below this: go and charge once the current order is finished
FULL = 95.0              # %  charge until this level
CHECK_EVERY = 10         # steps (0.5 s)


class Battery:
    def __init__(self, sim, level=100.0, robot_path="/youBot", log=print):
        self.sim, self.level, self.log = sim, float(level), log
        self.ref = sim.getObject(robot_path + "/youBot_ref")
        self.joints = [sim.getObject(f"{robot_path}/youBotArmJoint{i}") for i in range(5)]
        self.charging = False
        self.steps = 0
        self.t = sim.getSimulationTime()
        self.pos = sim.getObjectPosition(self.ref, sim.handle_world)[:2]
        self.q = [sim.getJointPosition(j) for j in self.joints]
        self.used = dict(idle=0.0, drive=0.0, arm=0.0)      # % used by each cause
        self.distance = 0.0
        self.charged = 0.0                                   # % added by charging
        self.history = [(round(self.t, 1), round(self.level, 1))]   # (time, level) every 10 s, for the report

    @property
    def low(self):
        return self.level < LOW

    def tick(self):
        self.steps += 1
        if self.steps % CHECK_EVERY:
            return
        sim = self.sim
        t = sim.getSimulationTime()
        dt, self.t = t - self.t, t
        pos = sim.getObjectPosition(self.ref, sim.handle_world)[:2]
        q = [sim.getJointPosition(j) for j in self.joints]
        d = math.dist(pos, self.pos)
        dq = sum(abs(a - b) for a, b in zip(q, self.q))
        self.pos, self.q = pos, q
        if self.charging:
            add = min(CHARGE_RATE * dt, 100.0 - self.level)
            self.level += add
            self.charged += add
        else:
            use = dict(idle=IDLE_DRAIN * dt, drive=DRIVE_DRAIN * d, arm=ARM_DRAIN * dq)
            for k, v in use.items():
                self.used[k] += v
            self.level = max(0.0, self.level - sum(use.values()))
            self.distance += d
        if t - self.history[-1][0] >= 10.0:
            self.history.append((round(t, 1), round(self.level, 1)))
