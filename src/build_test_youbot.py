r"""Build the youBot test / demo scene from src\demo_layout.py and save it as scenes\test_youbot.ttt.

A 10 x 8 m arena with one zone per capability of the robot (see demo_layout.py): sensor zone
(Part 3 test obstacles, where the robot starts), mecanum square, chicane, two transfer tables with
ArUco markers and a tote, a walking worker, a box that is not on the map, a charging dock.
The youBot (lidar, sonars, camera, arm) starts in the sensor zone.

Before running: in CoppeliaSim, open scenes\test_youbot.ttt (it is rebuilt) or an EMPTY new
scene (File > New scene). The simulation must be stopped.

Run (PowerShell, from the project folder):
  .\.venv\Scripts\python.exe src\build_test_youbot.py
"""

import sys
from pathlib import Path

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import build_scene
import demo_layout as demo
from build_scene import create_model, create_part, create_walker, look_at_matrix
from equip_youbot import add_youbot

ROOT = Path(__file__).resolve().parent.parent
SCENE_FILE = ROOT / "scenes" / "test_youbot.ttt"
DEFAULT_OBJECTS = {"DefaultCamera", "XYZCameraProxy", "DefaultLights", "Floor"}
ROOT_NAME = "Demo"
ROBOT = "youBot"
OLD = ("obstacles_test",)            # the Part 2-3 version of this scene (3 obstacles only)
VIEW_EYE, VIEW_TARGET = (5.0, -5.5, 8.5), (5.0, 3.6, 0.0)


def main():
    client = RemoteAPIClient()
    sim = client.require("sim")
    if sim.getSimulationState() != sim.simulation_stopped:
        print("STOP: stop the simulation first (square Stop button), then run again.")
        return 1
    tops = {sim.getObjectAlias(h): h for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 2)}
    for name in (ROBOT, ROOT_NAME) + OLD:
        if name in tops:
            print(f"Previous {name} found: removing it and rebuilding.")
            sim.removeModel(tops.pop(name))
    others = set(tops) - DEFAULT_OBJECTS
    if others:
        print("STOP: this scene already contains other objects:", sorted(others))
        print("In CoppeliaSim do File > New scene, then run this script again.")
        return 1
    if "Floor" in tops:              # the default 5 x 5 m floor: the arena has its own 10 x 8 m one
        sim.removeModel(tops["Floor"])

    root = sim.createDummy(0.1)
    sim.setObjectAlias(root, ROOT_NAME)
    sim.setModelProperty(root, 0)    # one model: easy to delete / rebuild as one block
    groups = {}
    for name in dict.fromkeys([p.group for p in demo.PARTS + demo.UNMAPPED] + [m.group for m in demo.MODELS]):
        g = sim.createDummy(0.05)
        sim.setObjectAlias(g, name)
        sim.setObjectParent(g, root, True)
        groups[name] = g

    handles = {}
    for p in demo.PARTS + demo.UNMAPPED:
        create_part(sim, p, handles, groups)
    print(f"Created {len(demo.PARTS) + len(demo.UNMAPPED)} shapes (incl. the box that is not on the map).")
    models_dir = sim.getStringParam(sim.stringparam_application_path) + "/models/"
    for m in demo.MODELS:
        create_model(sim, m, models_dir, groups)
    print(f"Loaded {len(demo.MODELS)} library models (floor, ArUco markers).")

    build_scene.SENSED = (sim.objectspecialproperty_collidable | sim.objectspecialproperty_measurable
                          | sim.objectspecialproperty_detectable)
    walkers = sim.createDummy(0.05)
    sim.setObjectAlias(walkers, "operateurs_mobiles")
    sim.setObjectParent(walkers, root, True)
    for w in demo.WALKERS:
        create_walker(sim, w, models_dir, walkers)
    print(f"Added {len(demo.WALKERS)} walking worker.")

    try:
        text = client.require("textUtils")
        for lb in demo.LABELS:
            t = text.generateTextShape(lb.text, list(lb.color), lb.height, True)
            sim.setObjectPosition(t, [lb.x, lb.y, 0.006], sim.handle_world)
            sim.setObjectParent(t, groups[lb.group], True)
            for o in sim.getObjectsInTree(t, sim.handle_all, 0):     # floor paint: not seen by the sensors
                sim.setObjectSpecialProperty(o, 0)
        print(f"Placed {len(demo.LABELS)} floor labels.")
    except Exception as e:           # decoration: never block the build for it
        print("Labels skipped:", e)

    add_youbot(sim, *demo.START, ROBOT)
    print("youBot (lidar, sonars, camera, arm) in the sensor zone.")

    cam = sim.getObject("/DefaultCamera")
    sim.setObjectMatrix(cam, look_at_matrix(VIEW_EYE, VIEW_TARGET), sim.handle_world)
    sim.setObjectSelection([])
    SCENE_FILE.parent.mkdir(exist_ok=True)
    sim.saveScene(SCENE_FILE.as_posix())
    print("Saved", SCENE_FILE)
    return 0


if __name__ == "__main__":
    sys.exit(main())
