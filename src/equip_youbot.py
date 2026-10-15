r"""Mount the sensors on the youBot (all from CoppeliaSim's library / built-in objects).

  - lidar: library 'Hokuyo URG 04LX UG01_Fast' (240 deg, 684 points, 5 m), front, 13 cm high
  - proximity ring: 8 ultrasonic sensors copied from the library Pioneer P3-DX (which has 16),
    around the base: a mecanum robot also moves sideways and backwards
  - camera: built-in vision sensor (640 x 480, 60 deg), front, above the lidar, looking forward

Positions are given in the ROBOT frame: forward = +y of youBot_ref, left = -x, up = +z
(measured in Part 2). Used by build_test_youbot.py and later by the workshop build.
"""

import math

LIDAR_POS = (0.33, 0.0, 0.13)       # forward, left, up (m)
CAMERA_POS = (0.33, 0.0, 0.22)
CAMERA_RES, CAMERA_ANGLE = (640, 480), math.radians(60)
CAMERA_TILT = math.radians(15)      # looks slightly down: the station markers (low, on the transfer
                                    # tables) stay in view down to ~0.25 m (Part 7, docking)
# (name, forward, left, height, direction in deg: 0 = forward, 90 = left)
# The front one sits above the lidar and camera: at 15 cm the lidar body hid its view.
SONARS = [("avant", 0.30, 0.00, 0.25, 0), ("avant_gauche", 0.29, 0.20, 0.15, 45), ("gauche", 0.0, 0.22, 0.15, 90),
          ("arriere_gauche", -0.29, 0.20, 0.15, 135), ("arriere", -0.30, 0.00, 0.15, 180),
          ("arriere_droite", -0.29, -0.20, 0.15, -135), ("droite", 0.0, -0.22, 0.15, -90),
          ("avant_droite", 0.29, -0.20, 0.15, -45)]
LIB = "/models/"


def _frame(sim, ref):
    """World vectors of the robot frame: origin, forward, left, up."""
    m = sim.getObjectMatrix(ref, sim.handle_world)
    o = (m[3], m[7], m[11])
    fwd = (m[1], m[5], m[9])
    left = (-m[0], -m[4], -m[8])
    up = (m[2], m[6], m[10])
    return o, fwd, left, up


def _to_world(frame, f, l, u):
    o, fwd, left, up = frame
    return [o[i] + f * fwd[i] + l * left[i] + u * up[i] for i in range(3)]


def _dir_world(frame, deg):
    _, fwd, left, _ = frame
    a = math.radians(deg)
    return [math.cos(a) * fwd[i] + math.sin(a) * left[i] for i in range(3)]


def _matrix_z_forward(z, origin, up=(0.0, 0.0, 1.0)):
    """Pose with +z looking along z and +y up (cameras and proximity sensors look along +z)."""
    x = [up[1] * z[2] - up[2] * z[1], up[2] * z[0] - up[0] * z[2], up[0] * z[1] - up[1] * z[0]]
    n = math.sqrt(sum(v * v for v in x))
    x = [v / n for v in x]
    y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]]
    return [x[0], y[0], z[0], origin[0], x[1], y[1], z[1], origin[1], x[2], y[2], z[2], origin[2]]


def add_youbot(sim, x, y, heading, name="youBot"):
    """Load the library youBot, remove its demo script, place it so that its reference point is
    at (x, y) facing `heading` (rad), and mount the sensors. Returns the robot handle."""
    app = sim.getStringParam(sim.stringparam_application_path)
    robot = sim.loadModel(app + LIB + "robots/mobile/KUKA YouBot.ttm")
    # the main script directly under the robot is the library DEMO (slides, then spins forever);
    # the small scripts on the wheels (mecanum roller effect) and on the gripper are kept
    for s in sim.getObjectsInTree(robot, sim.sceneobject_script, 1 | 2):
        sim.removeObjects([s])
    place_youbot(sim, robot, x, y, heading)
    sim.setObjectAlias(robot, name)
    equip(sim, robot, sim.getObject("./youBot_ref", {"proxy": robot}))
    _gripper_hook(sim, robot)
    return robot


def _gripper_hook(sim, robot):
    """The library gripper script keeps the fingers opening (global 'opening'): add a function so
    that Python can open / close it: sim.callScriptFunction('setGripper', script, open)."""
    for s in sim.getObjectsInTree(robot, sim.sceneobject_script, 0):
        txt = sim.getObjectStringParam(s, sim.scriptstringparam_text)
        txt = txt.decode() if isinstance(txt, bytes) else txt
        if "youBotGripperJoint2" in txt and "setGripper" not in txt:
            txt += "\n\nfunction setGripper(open)  -- added by equip_youbot.py\n    opening=open\nend\n"
            sim.setObjectStringParam(s, sim.scriptstringparam_text, txt)


def place_youbot(sim, robot, x, y, heading):
    """Put the robot so that youBot_ref is at (x, y) facing `heading` (rad). Also works during a
    simulation (tests start the robot in front of a station): its physics state is then reset."""
    ref = sim.getObject("./youBot_ref", {"proxy": robot})
    m = sim.getObjectMatrix(ref, sim.handle_world)
    h0 = math.atan2(m[5], m[1])                                  # robot forward = +y of youBot_ref
    rm = sim.rotateAroundAxis(sim.getObjectMatrix(robot, sim.handle_world), [0, 0, 1], [m[3], m[7], m[11]], heading - h0)
    rm[3] += x - m[3]
    rm[7] += y - m[7]
    sim.setObjectMatrix(robot, rm, sim.handle_world)
    if sim.getSimulationState() != sim.simulation_stopped:
        sim.resetDynamicObject(robot | sim.handleflag_model)


def equip(sim, robot, ref):
    """Add lidar, sonar ring and camera to the youBot `robot` (model handle), `ref` = youBot_ref."""
    app = sim.getStringParam(sim.stringparam_application_path)
    frame = _frame(sim, ref)
    heading = math.atan2(frame[1][1], frame[1][0])

    # --- lidar: turn it so that the middle of its 240 deg scan looks forward
    lidar = sim.loadModel(app + LIB + "components/sensors/Hokuyo URG 04LX UG01_Fast.ttm")
    sensors = [sim.getObject("./sensor1", {"proxy": lidar}), sim.getObject("./sensor2", {"proxy": lidar})]
    dirs = []
    for s in sensors:
        m = sim.getObjectMatrix(s, lidar)
        dirs.append((m[2], m[6]))                                  # view axis, in the lidar's frame
    centre = math.atan2(dirs[0][1] + dirs[1][1], dirs[0][0] + dirs[1][0])
    sim.setObjectOrientation(lidar, [0.0, 0.0, heading - centre], sim.handle_world)
    sim.setObjectPosition(lidar, _to_world(frame, *LIDAR_POS), sim.handle_world)
    sim.setObjectAlias(lidar, "lidar")
    sim.setObjectParent(lidar, robot, True)

    # --- sonar ring: copy 8 ultrasonic sensors from the library Pioneer P3-DX, then delete the Pioneer
    p3dx = sim.loadModel(app + LIB + "robots/mobile/pioneer p3dx.ttm")
    sonars = [o for o in sim.getObjectsInTree(p3dx, sim.sceneobject_proximitysensor, 0)][:len(SONARS)]
    sonars = sim.copyPasteObjects(sonars, 0)
    sim.removeModel(p3dx)
    for h, (name, f, l, z, deg) in zip(sonars, SONARS):
        sim.setObjectMatrix(h, _matrix_z_forward(_dir_world(frame, deg), _to_world(frame, f, l, z)), sim.handle_world)
        sim.setObjectAlias(h, "sonar_" + name)
        sim.setObjectParent(h, robot, True)

    # --- camera: built-in vision sensor looking forward
    cam = sim.createVisionSensor(2, [CAMERA_RES[0], CAMERA_RES[1], 0, 0],
                                 [0.02, 10.0, CAMERA_ANGLE, 0.03, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    up = frame[3]
    look = [math.cos(CAMERA_TILT) * f - math.sin(CAMERA_TILT) * u for f, u in zip(_dir_world(frame, 0), up)]
    sim.setObjectMatrix(cam, _matrix_z_forward(look, _to_world(frame, *CAMERA_POS)), sim.handle_world)
    sim.setObjectAlias(cam, "camera")
    sim.setObjectParent(cam, robot, True)

    # --- a small bracket holding the front sensors (decoration, not sensed)
    bracket = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [0.06, 0.12, 0.14], 0)
    sim.setShapeColor(bracket, None, sim.colorcomponent_ambient_diffuse, [0.2, 0.2, 0.22])
    sim.setObjectInt32Param(bracket, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(bracket, sim.shapeintparam_respondable, 0)
    sim.setObjectSpecialProperty(bracket, 0)
    sim.setObjectOrientation(bracket, [0.0, 0.0, heading], sim.handle_world)
    sim.setObjectPosition(bracket, _to_world(frame, LIDAR_POS[0] - 0.035, 0.0, 0.11), sim.handle_world)
    sim.setObjectAlias(bracket, "support_capteurs")
    sim.setObjectParent(bracket, robot, True)
    return dict(lidar=lidar, sonars=sonars, camera=cam)
