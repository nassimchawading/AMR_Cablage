r"""The factory's background machines keep working while the AMR does its job.

    client = RemoteAPIClient()
    sim = Factory(client)            # use it exactly like `sim` (same functions)
    sim = Factory(client, flow=True) # + the operators: QC tote stream, delivered totes taken (Part 8.2)
    nav = Navigator(sim, ...)        # ... robot code as usual

Every program advances the simulation with sim.step(). Factory wraps the `sim` object: before each
real step it gives every background machine (the quality-control cell: camera + UR5, and with
flow=True the operators of tote_flow.py) its own step. So the QC cell sorts totes all the time,
whatever the robot is doing (driving, docking, using its arm), like a real cell with its own
controller - and nothing waits for anything else.
"""

import tote_flow
from qc_cell import QCCell


class Factory:
    def __init__(self, client, log=print, flow=False, **flow_args):
        self._sim = client.require("sim")
        self._client = client
        self.qc = None
        self.flow = None
        self._flow_args = flow_args if flow else None
        self.log = log

    def __getattr__(self, name):            # everything else: the normal sim API
        return getattr(self._sim, name)

    def startSimulation(self):
        r = self._sim.startSimulation()
        tote_flow.restore_scene(self._sim)  # totes as built (a previous flow run repainted / parked some)
        # the cell needs a running simulation (IK world copied from the arm's current pose)
        self.qc = QCCell(self._sim, self._client.require("simIK"), log=self.log)
        if self._flow_args is not None:
            self.flow = tote_flow.ToteFlow(self._sim, log=self.log, **self._flow_args)
        return r

    def step(self):
        if self.qc is not None:
            self.qc.tick()
        if self.flow is not None:
            self.flow.tick()
        self._sim.step()
