r"""Part 1 test: can Python talk to CoppeliaSim?

What it does:
  1. connects to CoppeliaSim (which must already be open)
  2. starts the simulation in stepping mode
  3. advances it step by step for 3 simulated seconds, printing the sim time
  4. stops the simulation

Run (PowerShell, from the project folder):
  .\.venv\Scripts\python.exe tests\test_connection.py
"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

print("Connecting to CoppeliaSim... (if this hangs, CoppeliaSim is not open: Ctrl+C to stop)")
client = RemoteAPIClient()      # talks to CoppeliaSim on this PC (localhost, port 23000)
sim = client.require("sim")     # 'sim' = CoppeliaSim's main API (objects, simulation, ...)
print("Connected.")

# Stepping mode: the simulation only advances when Python calls sim.step().
# Python and the simulation stay in sync, so our robot "brain" never misses a step.
sim.setStepping(True)
sim.startSimulation()

while (t := sim.getSimulationTime()) < 3.0:
    print(f"sim time = {t:.2f} s")
    sim.step()                  # advance the simulation by one time step (0.05 s by default)

sim.stopSimulation()
print("Simulation stopped. Part 1 test OK.")
