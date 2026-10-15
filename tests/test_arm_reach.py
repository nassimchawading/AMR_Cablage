r"""Part 7 test: inverse kinematics of the youBot arm, checked on the simulated arm.

The robot stands in the aisle (nothing around). The arm moves its grasp point to several targets
(the table grasp point, the platform carry point, ...). The real grasp point (middle of the two
finger pads, read from the simulator) is compared with the target. Then the gripper is closed
and opened once.

Needs scenes\atelier_cablage.ttt open. Run (PowerShell, from the project folder):
  .\.venv\Scripts\python.exe tests\test_arm_reach.py
"""

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from arm import YouBotArm, fk, ik
from equip_youbot import place_youbot

TARGETS = {
    "table (tote fin)": (0.50, 0.0, 0.365),
    "table, 3 cm left": (0.50, 0.03, 0.365),
    "table, high": (0.45, 0.0, 0.42),
    "low, near": (0.40, 0.0, 0.30),
    "platform (carry)": (-0.20, 0.0, 0.324),
}


def grasp_point(sim, ref, pads):
    mi = sim.getMatrixInverse(sim.getObjectMatrix(ref, sim.handle_world))
    ps = [sim.multiplyVector(mi, sim.getObjectPosition(p, sim.handle_world)) for p in pads]
    x, y, z = [(a + b) / 2 for a, b in zip(*ps)]
    return y, -x, z                      # youBot_ref frame -> robot frame (forward, left, up)


def main():
    sim = RemoteAPIClient().require("sim")
    robot = sim.getObject("/youBot")
    ref = sim.getObject("/youBot/youBot_ref")
    pads = [sim.getObject("/youBot/Rectangle5"), sim.getObject("/youBot/Rectangle6")]
    sim.setStepping(True)
    sim.startSimulation()
    worst = 0.0
    try:
        place_youbot(sim, robot, 6.0, 6.0, 0.0)
        for _ in range(5):
            sim.step()
        arm = YouBotArm(sim)
        print("folded pose, fk =", [round(v, 3) for v in fk(arm.q)], " real =", [round(v, 3) for v in grasp_point(sim, ref, pads)])
        for name, t in TARGETS.items():
            q, phi = ik(*t)
            arm.move_joints(q)
            g = grasp_point(sim, ref, pads)
            err = math.dist(g, t)
            worst = max(worst, err)
            print(f"{name:18s} target {t}  pitch {math.degrees(phi):4.0f} deg  joints {[round(math.degrees(v)) for v in q]}"
                  f"  real {tuple(round(v, 3) for v in g)}  error {err * 100:.1f} cm")
        arm.gripper(close=True)
        closed = arm.opening()
        arm.gripper(close=False)
        print(f"gripper: closed {closed:+.3f}, open {arm.opening():+.3f}")
        arm.fold()
    finally:
        sim.stopSimulation()
    print(f"worst error {worst * 100:.1f} cm")
    print("RESULT:", "PASSED" if worst < 0.01 and closed > arm.opening() + 0.02 else "FAILED")


if __name__ == "__main__":
    main()
