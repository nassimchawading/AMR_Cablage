r"""The factory's background machines keep working while the AMR does its job.

    client = RemoteAPIClient()
    sim = Factory(client)            # use it exactly like `sim` (same functions)
    nav = Navigator(sim, ...)        # ... robot code as usual

Every program advances the simulation with sim.step(). Factory wraps the `sim` object: before each
real step it gives every background machine (here the quality-control cell: camera + UR5) its own
step. So the QC cell sorts totes all the time, whatever the robot is doing (driving, docking, using
its arm), like a real cell with its own controller - and nothing waits for anything else.
"""

from qc_cell import QCCell


class Factory:
    def __init__(self, client, log=print):
        self._sim = client.require("sim")
        self._client = client
        self.qc = None
        self.log = log

    def __getattr__(self, name):            # everything else: the normal sim API
        return getattr(self._sim, name)

    def startSimulation(self):
        r = self._sim.startSimulation()
        # the cell needs a running simulation (IK world copied from the arm's current pose)
        self.qc = QCCell(self._sim, self._client.require("simIK"), log=self.log)
        return r

    def step(self):
        if self.qc is not None:
            self.qc.tick()
        self._sim.step()
