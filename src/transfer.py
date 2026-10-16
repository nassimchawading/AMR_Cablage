r"""Load / unload a tote at a station: docking + arm, as one operation.

    tr = Transfer(sim, robot, sensors, docker, arm)
    tr.load(marker_id)       # tote: station table -> robot platform
    tr.unload(marker_id)     # tote: robot platform -> station table

Where the tote is:
  - on a table: centred on the station's marker (layout.tote_on_table), TOTE_INSET behind the
    table's front face. After docking, the lidar gives the face distance, so the fin is at
    forward = face distance + TOTE_INSET, left = 0 (docking centred the robot on the marker);
  - on the robot: CARRY, on the rear platform, behind the folded gripper's rest.
"""

import math

import layout
from arm import J0_POS, Unreachable

FIN_UP = layout.TOTE[2] + layout.FIN[2] / 2            # post centre above the tote's bottom
# grasp point 3 mm above the post's centre: with the gripper tilted ~30 deg, the palm stays clear
# of the post's top and the fingertips pass ~1 cm above the tote (gripper shapes measured in Part 7)
GRASP_UP = 0.003
TABLE_FIN_Z = layout.TABLE_H + FIN_UP + GRASP_UP
PLATFORM_TOP = 0.154                                   # measured: a tote dropped on the platform rests there
# The tote is locked in the hand while carried, so it must be released with the hand at the SAME
# pitch as when it was grasped, or it lands tilted. Same pitch everywhere: GRASP_PHI. To reach the
# platform behind the robot with that pitch, the arm turns its base to CARRY_ANGLE (limit +-169 deg)
# instead of leaning back over itself; the tote then only turns about the vertical, staying level.
GRASP_PHI = math.radians(30)
CARRY_ANGLE, CARRY_RADIUS = math.radians(167), 0.38    # from the arm base: tote behind the gripper rest
CARRY = (J0_POS[0] + CARRY_RADIUS * math.cos(CARRY_ANGLE), CARRY_RADIUS * math.sin(CARRY_ANGLE),
         PLATFORM_TOP + FIN_UP + GRASP_UP)
RELEASE = 0.006                                        # m: open the gripper this high above the resting height


class Transfer:
    def __init__(self, sim, robot, sensors, docker, arm, log=print):
        self.sim, self.robot, self.sensors, self.docker, self.arm, self.log = sim, robot, sensors, docker, arm, log

    def _table_fin(self, res):
        a, _ = self.docker.face()
        return (a + layout.TOTE_INSET, 0.0, TABLE_FIN_Z)

    def _run(self, marker_id, pick_at, place_at, what):
        t0 = self.sim.getSimulationTime()
        res = self.docker.dock(marker_id)
        if not res["ok"]:
            return dict(ok=False, info=f"docking failed: {res['phase']}", time=round(self.sim.getSimulationTime() - t0, 1))
        try:
            src = pick_at(res)
            dst = place_at(res)
            held = self.arm.pick(*src, phi=GRASP_PHI)
            if not held:
                self.arm.gripper(close=False, wait=0.3)
                self.arm.fold()
                info = "grasp failed (nothing between the fingers)"
            else:
                self.arm.place(dst[0], dst[1], dst[2] + RELEASE, phi=GRASP_PHI)
                self.arm.fold()
                info = f"{what} (docked in {res['time']} s)"
        except Unreachable as e:
            self.arm.fold()
            held, info = False, f"arm: {e}"
        self.docker.undock()
        return dict(ok=held, info=info, time=round(self.sim.getSimulationTime() - t0, 1))

    def load(self, marker_id):
        return self._run(marker_id, self._table_fin, lambda r: CARRY, "table -> platform")

    def unload(self, marker_id):
        return self._run(marker_id, lambda r: CARRY, self._table_fin, "platform -> table")
