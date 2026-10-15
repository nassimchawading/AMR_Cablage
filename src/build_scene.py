r"""Build the workshop scene in CoppeliaSim from src\layout.py and save it.

Before running: in CoppeliaSim, open an EMPTY new scene (File > New scene).
The script refuses to run in a scene that contains other things, so it never
damages your work. Re-running it in scenes\atelier_cablage.ttt rebuilds the workshop.

Run (PowerShell, from the project folder):
  .\.venv\Scripts\python.exe src\build_scene.py
"""

import math
import re
import sys
from pathlib import Path

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import layout
from equip_youbot import add_youbot

ROOT = Path(__file__).resolve().parent.parent
SCENE_FILE = ROOT / "scenes" / "atelier_cablage.ttt"
DEFAULT_OBJECTS = {"DefaultCamera", "XYZCameraProxy", "DefaultLights", "Floor"}
ROOT_NAME = "Atelier"
ROBOT_NAME = "youBot"      # the AMR, parked at its dock (separate from 'Atelier': it moves)

# Where the GUI camera looks from / at when the scene opens (overview of the hall)
VIEW_EYE, VIEW_TARGET = (10.0, -7.0, 12.0), (10.0, 5.0, 0.0)


def look_at_matrix(eye, target):
    """12-value pose matrix for a camera/vision sensor at `eye` looking at `target`
    (CoppeliaSim cameras look along their +z axis, image 'up' = their +y axis)."""
    z = [t - e for t, e in zip(target, eye)]
    n = math.sqrt(sum(v * v for v in z))
    z = [v / n for v in z]
    up = (0.0, 0.0, 1.0)
    x = [up[1] * z[2] - up[2] * z[1], up[2] * z[0] - up[0] * z[2], up[0] * z[1] - up[1] * z[0]]
    n = math.sqrt(sum(v * v for v in x))
    x = [v / n for v in x]
    y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]]
    return [x[0], y[0], z[0], eye[0],
            x[1], y[1], z[1], eye[1],
            x[2], y[2], z[2], eye[2]]


def check_scene(sim):
    """Return True if it is safe to build here; remove a previous build if there is one."""
    tops = {sim.getObjectAlias(h): h for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 2)}
    for name in (ROOT_NAME, ROBOT_NAME):
        if name in tops:
            print(f"Previous {name} found: removing it and rebuilding.")
            sim.removeModel(tops.pop(name))
    others = set(tops) - DEFAULT_OBJECTS
    if others:
        print("STOP: this scene already contains other objects:", sorted(others))
        print("In CoppeliaSim do File > New scene, then run this script again.")
        return False
    if "Floor" in tops:            # the default 5x5 m floor: we make our own 20x12 m one
        sim.removeModel(tops["Floor"])
    return True


def create_part(sim, p, handles, groups):
    kind = sim.primitiveshape_cuboid if p.shape == "box" else sim.primitiveshape_cylinder
    h = sim.createPrimitiveShape(kind, list(p.size), 0)
    sim.setObjectAlias(h, p.name)
    sim.setShapeColor(h, None, sim.colorcomponent_ambient_diffuse, list(p.color))
    # static = never moved by physics; only "dynamic" parts (bins on the conveyor) are
    sim.setObjectInt32Param(h, sim.shapeintparam_static, 0 if p.kind == "dynamic" else 1)
    sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 0 if p.kind == "paint" else 1)
    if p.kind == "dynamic":
        sim.setShapeMass(h, p.mass)
    sensed = (sim.objectspecialproperty_collidable | sim.objectspecialproperty_measurable
              | sim.objectspecialproperty_detectable)
    sim.setObjectSpecialProperty(h, 0 if p.kind == "paint" else sensed)
    ref = handles[p.parent] if p.parent else sim.handle_world
    sim.setObjectOrientation(h, list(p.rot), ref)
    sim.setObjectPosition(h, list(p.pos), ref)
    if p.kind == "merge":
        # one rigid body (a dynamic shape's child would be a separate, loose body): group the two
        # shapes into a compound shape that keeps the parent's name, place, dynamics and mass
        parent = handles[p.parent]
        alias, mass = sim.getObjectAlias(parent), sim.getShapeMass(parent)
        up = sim.getObjectParent(parent)
        g = sim.groupShapes([parent, h], False)
        sim.setObjectAlias(g, alias)
        sim.setObjectInt32Param(g, sim.shapeintparam_static, 0)
        sim.setObjectInt32Param(g, sim.shapeintparam_respondable, 1)
        sim.setShapeMass(g, mass)
        sim.setObjectSpecialProperty(g, sensed)
        sim.setObjectParent(g, up, True)
        handles[p.parent] = g
        return
    sim.setObjectParent(h, handles[p.parent] if p.parent else groups[p.group], True)
    handles[p.name] = h


def _script(sim, h):
    return sim.getObjectsInTree(h, sim.sceneobject_script, 0)[0]


def _update_config(sim, h, values):
    """Library models keep their settings in a '__config__' table that their script reads."""
    cfg = sim.readCustomTableData(h, "__config__")
    cfg.update(values)
    sim.writeCustomTableData(h, "__config__", cfg)
    return cfg


def setup_conveyor(sim, h, values):
    _update_config(sim, h, values)
    sim.initScript(_script(sim, h))          # restarting its script rebuilds the conveyor at the new size


def setup_aruco(sim, h, values):
    cfg = _update_config(sim, h, values)
    s = _script(sim, h)
    sim.initScript(s)
    sim.callScriptFunction("generate", s, cfg)   # redraws the black/white cells for this marker id


def setup_table(sim, h, v):
    """The 'customizable table' has its size written in its script: edit those numbers."""
    s = _script(sim, h)
    txt = sim.getObjectStringParam(s, sim.scriptstringparam_text)
    txt = txt.decode() if isinstance(txt, bytes) else txt
    for key in ("length", "width", "height"):
        txt = re.sub(rf"tableConfig\.{key}=[\d.]+", f"tableConfig.{key}={v[key]}", txt)
    for key in ("color1", "color2"):
        txt = re.sub(rf"tableConfig\.{key}=\{{[^}}]*\}}", f"tableConfig.{key}={{{','.join(str(c) for c in v[key])}}}", txt)
    sim.setObjectStringParam(s, sim.scriptstringparam_text, txt)
    sim.initScript(s)


UR5_D6 = 0.0823        # UR5 datasheet: distance from the last joint to the tool flange (m)


def frame_from_z(z, origin):
    """12-value pose matrix whose z axis is `z` (unit vector), placed at `origin`."""
    helper = (0.0, 0.0, 1.0) if abs(z[2]) < 0.9 else (1.0, 0.0, 0.0)
    x = [helper[1] * z[2] - helper[2] * z[1], helper[2] * z[0] - helper[0] * z[2], helper[0] * z[1] - helper[1] * z[0]]
    n = math.sqrt(sum(v * v for v in x))
    x = [v / n for v in x]
    y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]]
    return [x[0], y[0], z[0], origin[0], x[1], y[1], z[1], origin[1], x[2], y[2], z[2], origin[2]]


def setup_ur5(sim, h, values):
    """Called right after loading, while the arm is still at the world origin in its zero pose.
    1) remove the library demo script (it swings the arm around as soon as the simulation starts);
    2) mount a library suction pad on the tool flange, pointing outward."""
    for s in sim.getObjectsInTree(h, sim.sceneobject_script, 1 | 2):   # scripts directly under the arm
        sim.removeObjects([s])
    if not values.get("suction_pad"):
        return
    last_joint = sim.getObjectsInTree(h, sim.sceneobject_joint, 0)[-1]
    m = sim.getObjectMatrix(last_joint, sim.handle_world)
    axis = (m[2], m[6], m[10])                                   # last joint axis = tool direction
    flange = [m[3 + 4 * i] + UR5_D6 * axis[i] for i in range(3)]
    pad = sim.loadModel(sim.getStringParam(sim.stringparam_application_path) + "/models/components/grippers/suction pad.ttm")
    # the pad's top is 0.035 m above its origin along its +z; its suction face looks along -z
    origin = [flange[i] + 0.035 * axis[i] for i in range(3)]
    sim.setObjectMatrix(pad, frame_from_z([-a for a in axis], origin), sim.handle_world)
    # attach through the arm's own tool force sensor 'connection': a dynamic shape parented directly
    # to another one is NOT held by it (the pad fell to the floor as a separate body, Part 9)
    sim.setObjectParent(pad, sim.getObject("./connection", {"proxy": h}), True)


def setup_camera(sim, h, values):
    """Library 'Blob detection camera': remove its built-in blob filter (it paints the whole
    image red) so it delivers the RAW colour image; our Python code does the QC detection.
    Its floating view window in CoppeliaSim is kept. Optional: new resolution."""
    cam = sim.getObjectsInTree(h, sim.sceneobject_visionsensor, 0)[0]
    s = sim.getObjectsInTree(cam, sim.sceneobject_script, 0)[0]
    txt = sim.getObjectStringParam(s, sim.scriptstringparam_text)
    txt = txt.decode() if isinstance(txt, bytes) else txt
    txt = re.sub(r"\nfunction sysCall_vision\(inData\).*?\nend", "\n", txt, flags=re.S)
    sim.setObjectStringParam(s, sim.scriptstringparam_text, txt)
    sim.initScript(s)
    if "resolution" in values:
        sim.setObjectInt32Param(cam, sim.visionintparam_resolution_x, values["resolution"])
        sim.setObjectInt32Param(cam, sim.visionintparam_resolution_y, values["resolution"])


SETUP = {"conveyor": setup_conveyor, "aruco": setup_aruco, "table": setup_table, "ur5": setup_ur5,
         "camera": setup_camera}


def create_model(sim, m, models_dir, groups):
    h = sim.loadModel(models_dir + layout.LIB[m.key][0])
    for kind, values in m.setup.items():
        SETUP[kind](sim, h, values)
    if m.scale:
        sim.scaleObject(h, *m.scale, 0)
    sim.setObjectOrientation(h, list(m.rot), sim.handle_world)
    x, y, z = m.pos
    if z is None:                          # put the model's bottom on the floor
        z = -sim.getObjectFloatParam(h, sim.objfloatparam_modelbbox_min_z)
    sim.setObjectPosition(h, [x, y, z], sim.handle_world)
    sim.setObjectAlias(h, m.name)
    sim.setObjectParent(h, groups[m.group], True)


def qc_cell_extras(sim, group):
    """What the QC cell needs to be driven from Python (Part 9):
    - suction pad: starts OFF, function setSuction(on) (the library pad sucks anything it touches);
    - UR5 IK: dummy 'tip' on the suction face (pad frame: +z away from the face) and dummy 'target'
      (the pose Python asks the tip to reach);
    - light barrier at the conveyor end: ray proximity sensor beside the belt, looking across it at
      tote height, 10 cm before the end stop: 'a tote is waiting at the end stop'."""
    ur5 = sim.getObject(f"/{ROOT_NAME}/ur5")
    # the library base is a free dynamic body: once the arm leans out, the whole UR5 tipped off its
    # pedestal. A real UR5 is bolted down: base static (set here, at the end: earlier it got reset)
    sim.setObjectInt32Param(ur5, sim.shapeintparam_static, 1)
    pad = sim.getObject(f"/{ROOT_NAME}/ur5/suctionPad")
    s = next(o for o in sim.getObjectsInTree(pad, sim.sceneobject_script, 0))
    txt = sim.getObjectStringParam(s, sim.scriptstringparam_text)
    txt = txt.decode() if isinstance(txt, bytes) else txt
    txt = txt.replace("enabled=true", "enabled=false", 1)
    txt += "\n\nfunction setSuction(on)  -- added by build_scene.py\n    enabled=on\nend\n"
    sim.setObjectStringParam(s, sim.scriptstringparam_text, txt)
    face = sim.getObject(f"/{ROOT_NAME}/ur5/suctionPad/Sensor")
    tip = sim.createDummy(0.02)
    m = sim.getObjectMatrix(pad, sim.handle_world)
    p = sim.getObjectPosition(face, sim.handle_world)
    m[3], m[7], m[11] = p
    sim.setObjectMatrix(tip, m, sim.handle_world)
    sim.setObjectAlias(tip, "tip")
    sim.setObjectParent(tip, pad, True)
    target = sim.createDummy(0.02)
    sim.setObjectMatrix(target, m, sim.handle_world)
    sim.setObjectAlias(target, "target")
    sim.setObjectParent(target, ur5, True)
    for d in (tip, target):
        sim.setObjectInt32Param(d, sim.objintparam_visibility_layer, 0)      # invisible markers
    # start pose: pad down above the conveyor end (instead of the library's arm-straight-up pose)
    from qc_cell import HOME_DEG
    for j, a in zip(sim.getObjectsInTree(ur5, sim.sceneobject_joint, 0), HOME_DEG):
        sim.setJointPosition(j, math.radians(a))
        sim.setJointTargetPosition(j, math.radians(a))
    sim.setObjectMatrix(target, sim.getObjectMatrix(tip, sim.handle_world), sim.handle_world)
    # light barrier: ray across the belt, 10 cm before the end stop, 3 cm above the belt
    ray = sim.createProximitySensor(sim.proximitysensor_ray, 16, 0, [0] * 8,
                                    [0.0, 0.6, 0.01, 0.01, 0.01, 0.01, 0.0, 0.01, 0.01, 0.0, 0.0, 0.0, 0.002, 0.0, 0.0])
    sim.setObjectMatrix(ray, frame_from_z([0.0, 1.0, 0.0], [layout.QC_X1 - 0.10, layout.QC_Y - 0.30, layout.PICK_H + 0.01]),
                        sim.handle_world)
    sim.setObjectAlias(ray, "barriere_fin_convoyeur")
    sim.setObjectParent(ray, group, True)


SENSED = None   # set in main(): collidable | measurable | detectable


def make_detectable(sim, model):
    """Library people have all their parts invisible to sensors: make them seen by the lidar/sonars."""
    for s in sim.getObjectsInTree(model, sim.sceneobject_shape, 0):
        sim.setObjectSpecialProperty(s, SENSED)


YIELD_LUA = f"""
-- added by build_scene.py: the worker stops while the AMR is right in front of him
YIELD_DIST, YIELD_WIDTH, YIELD_MAX = {layout.YIELD_DIST}, {layout.YIELD_WIDTH}, {layout.YIELD_MAX}

function robotAhead()
    if not robotRef then robotRef=sim.getObject('/youBot_ref', {{noError=true}}) end
    if robotRef==-1 then return false end
    local b=sim.getObjectPosition(BillHandle)
    local r=sim.getObjectPosition(robotRef)
    local a=sim.getObjectOrientation(BillHandle)[3]
    local dx, dy=r[1]-b[1], r[2]-b[2]
    local f=dx*math.cos(a)+dy*math.sin(a)
    local l=-dx*math.sin(a)+dy*math.cos(a)
    return f>0 and f<YIELD_DIST and math.abs(l)<YIELD_WIDTH
end

function yieldToRobot()
    local st=sim.getSimulationTime()
    if robotAhead() then
        moveBody()
        while robotAhead() and sim.getSimulationTime()-st<YIELD_MAX do
            sim.step()
        end
    end
    return sim.getSimulationTime()-st
end
"""


def create_walker(sim, w, models_dir, parent):
    """Library 'Bill on path' walking our own route: replace its path, set speed and pauses."""
    h = sim.loadModel(models_dir + "people/Bill on path.ttm")
    sim.setObjectPosition(h, [w.route[0][0], w.route[0][1], 0.0], sim.handle_world)
    old_path = sim.getObject("./path", {"proxy": h})
    try:
        sim.removeModel(old_path)
    except Exception:                       # not a model base: remove it and its children directly
        sim.removeObjects(sim.getObjectsInTree(old_path, sim.handle_all, 0))
    # control points: x, y, z + orientation quaternion (identity), path dummy at the world origin
    pts = []
    for x, y in w.route:
        pts += [x, y, 0.0, 0.0, 0.0, 0.0, 1.0]
    path = sim.createPath(pts, 0, 100, 0.0)       # smoothness 0: straight segments between waypoints
    sim.setObjectAlias(path, "path")
    sim.setObjectParent(path, h, True)
    # Bill's walking script: in this model the root object IS the script
    s = h if sim.getObjectType(h) == sim.sceneobject_script else sim.getObjectsInTree(h, sim.sceneobject_script, 1 | 2)[0]
    txt = sim.getObjectStringParam(s, sim.scriptstringparam_text)
    txt = txt.decode() if isinstance(txt, bytes) else txt
    for key, val in (("velocity", w.speed), ("startPosPause", w.pause_start), ("goalPosPause", w.pause_end)):
        txt = re.sub(rf"\n(\s*){key}=[\d.]+", rf"\n\g<1>{key}={val}", txt, count=1)
    # the library model stops at 2 "pause points" placed for ITS original path; they would make
    # Bill stop at random places on our route: unregister them (pauses only at the route ends)
    txt = re.sub(r"\n\s*pausePoints\[\d\]=\{[^\n]*\}", "", txt)
    # Bill gives way to the robot: before each walking step, wait while the robot is in front of him
    txt = txt.replace("function sysCall_thread()", YIELD_LUA + "\nfunction sysCall_thread()", 1)
    for loop in ("while dist<pathL do", "while dist>0 do"):       # (script text has \r\n line ends)
        txt = re.sub(rf"({re.escape(loop)}\r?\n)", r"\1            st=st+yieldToRobot()\n", txt, count=1)
    assert txt.count("st=st+yieldToRobot()") == 2, "Bill script changed: yield hooks not inserted"
    sim.setObjectStringParam(s, sim.scriptstringparam_text, txt)
    make_detectable(sim, h)
    # The walker is moved by small "teleports" every step. If his body is respondable, walking into
    # the robot makes the physics engine fight the overlap with huge forces (this froze CoppeliaSim
    # during a navigation test). He stays visible to the sensors and to collision checks, but takes
    # no part in physical contacts.
    for shape in sim.getObjectsInTree(h, sim.sceneobject_shape, 0):
        sim.setObjectInt32Param(shape, sim.shapeintparam_respondable, 0)
    sim.setObjectAlias(h, w.name)
    sim.setObjectParent(h, parent, True)


def main():
    global SENSED
    client = RemoteAPIClient()
    sim = client.require("sim")
    if sim.getSimulationState() != sim.simulation_stopped:
        print("STOP: stop the simulation first (square Stop button), then run again.")
        return 1
    if not check_scene(sim):
        return 1

    root = sim.createDummy(0.1)
    sim.setObjectAlias(root, ROOT_NAME)
    sim.setModelProperty(root, 0)          # makes 'Atelier' a model: easy to delete/rebuild as one block
    groups = {}
    for name in dict.fromkeys([p.group for p in layout.PARTS] + [m.group for m in layout.MODELS]):
        g = sim.createDummy(0.05)
        sim.setObjectAlias(g, name)
        sim.setObjectParent(g, root, True)
        groups[name] = g

    print(f"Creating {len(layout.PARTS)} shapes...")
    handles = {}
    for p in layout.PARTS:
        create_part(sim, p, handles, groups)

    models_dir = sim.getStringParam(sim.stringparam_application_path) + "/models/"
    SENSED = sim.objectspecialproperty_collidable | sim.objectspecialproperty_measurable | sim.objectspecialproperty_detectable
    print(f"Loading {len(layout.MODELS)} library models...")
    for m in layout.MODELS:
        create_model(sim, m, models_dir, groups)
        if m.key.startswith("bill"):                        # static workers: visible to the sensors
            make_detectable(sim, sim.getObject(f"/{ROOT_NAME}/{m.group}/{m.name}"))

    walkers = sim.createDummy(0.05)
    sim.setObjectAlias(walkers, "operateurs_mobiles")
    sim.setObjectParent(walkers, root, True)
    for w in layout.WALKERS:
        create_walker(sim, w, models_dir, walkers)
    print(f"Added {len(layout.WALKERS)} walking workers.")
    qc_cell_extras(sim, groups["test"])
    print("QC cell: suction hook, IK tip/target, light barrier at the conveyor end.")

    try:
        text = client.require("textUtils")
        for lb in layout.LABELS:
            t = text.generateTextShape(lb.text, list(lb.color), lb.height, True)
            sim.setObjectPosition(t, [lb.x, lb.y, 0.006], sim.handle_world)
            sim.setObjectParent(t, groups[lb.group], True)
            for o in sim.getObjectsInTree(t, sim.handle_all, 0):     # paint on the floor: the sonars
                sim.setObjectSpecialProperty(o, 0)                   # detected the letters (phantom obstacles)
        print(f"Placed {len(layout.LABELS)} floor labels.")
    except Exception as e:  # labels are decoration: never block the build for them
        print("Labels skipped:", e)

    x, y, heading = layout.STATIONS["dock"]["pose"]
    add_youbot(sim, x, y, heading, ROBOT_NAME)
    print("youBot (with lidar, sonars, camera) parked at the dock.")

    cam = sim.getObject("/DefaultCamera")
    sim.setObjectMatrix(cam, look_at_matrix(VIEW_EYE, VIEW_TARGET), sim.handle_world)

    sim.setObjectSelection([])
    SCENE_FILE.parent.mkdir(exist_ok=True)
    sim.saveScene(SCENE_FILE.as_posix())
    print("Saved", SCENE_FILE)
    return 0


if __name__ == "__main__":
    sys.exit(main())
