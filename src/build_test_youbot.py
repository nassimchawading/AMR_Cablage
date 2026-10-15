r"""Part 2: build the youBot test scene (built-in floor + built-in KUKA youBot) and save it
as scenes\test_youbot.ttt.

Before running: in CoppeliaSim, open an EMPTY new scene (File > New scene).
Re-running it in scenes\test_youbot.ttt rebuilds the robot.

Run (PowerShell, from the project folder):
  .\.venv\Scripts\python.exe src\build_test_youbot.py
"""

import math
import sys
from pathlib import Path

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from build_scene import look_at_matrix
from equip_youbot import add_youbot

ROOT = Path(__file__).resolve().parent.parent
SCENE_FILE = ROOT / "scenes" / "test_youbot.ttt"
DEFAULT_OBJECTS = {"DefaultCamera", "XYZCameraProxy", "DefaultLights", "Floor"}
ROBOT = "youBot"
OBSTACLES = "obstacles_test"


def main():
    sim = RemoteAPIClient().require("sim")
    if sim.getSimulationState() != sim.simulation_stopped:
        print("STOP: stop the simulation first (square Stop button), then run again.")
        return 1
    tops = {sim.getObjectAlias(h): h for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 2)}
    for name in (ROBOT, OBSTACLES):
        if name in tops:
            print(f"Previous {name} found: removing it and rebuilding.")
            sim.removeModel(tops.pop(name))
    others = set(tops) - DEFAULT_OBJECTS
    if others:
        print("STOP: this scene already contains other objects:", sorted(others))
        print("In CoppeliaSim do File > New scene, then run this script again.")
        return 1

    # built-in youBot, demo script removed, sensors mounted (Part 3), at the origin facing +y
    add_youbot(sim, 0.0, 0.0, math.pi / 2, ROBOT)

    # Test obstacles, in their own group (NOT under the robot, or they would move with it):
    # robot forward = world +y, left = world -x at load (see youbot.py)
    obstacles = sim.createDummy(0.05)
    sim.setObjectAlias(obstacles, OBSTACLES)
    sim.setModelProperty(obstacles, 0)
    for name, kind, size, pos in (("obstacle_avant", "box", [0.5, 0.5, 0.5], [0.0, 1.2, 0.25]),
                                  ("poteau_gauche", "cyl", [0.12, 0.12, 1.0], [-0.7, 0.0, 0.5]),
                                  ("mur_arriere", "box", [2.0, 0.1, 0.5], [0.0, -1.0, 0.25])):
        prim = sim.primitiveshape_cuboid if kind == "box" else sim.primitiveshape_cylinder
        h = sim.createPrimitiveShape(prim, size, 0)
        sim.setShapeColor(h, None, sim.colorcomponent_ambient_diffuse, [0.85, 0.55, 0.1])
        sim.setObjectInt32Param(h, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 1)
        sim.setObjectSpecialProperty(h, sim.objectspecialproperty_collidable | sim.objectspecialproperty_measurable
                                     | sim.objectspecialproperty_detectable)
        sim.setObjectPosition(h, pos, sim.handle_world)
        sim.setObjectAlias(h, name)
        sim.setObjectParent(h, obstacles, True)

    cam = sim.getObject("/DefaultCamera")
    sim.setObjectMatrix(cam, look_at_matrix((-2.5, -3.0, 2.5), (0.5, 0.0, 0.0)), sim.handle_world)

    sim.setObjectSelection([])
    SCENE_FILE.parent.mkdir(exist_ok=True)
    sim.saveScene(SCENE_FILE.as_posix())
    print("Saved", SCENE_FILE)
    return 0


if __name__ == "__main__":
    sys.exit(main())
