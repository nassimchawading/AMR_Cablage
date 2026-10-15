r"""Workshop layout ("usine de câblage"): the single source of truth.

Every object of the workshop is described here as plain data. build_scene.py turns this
data into a CoppeliaSim scene, and later the planner will turn the SAME data into the
occupancy grid (fixed map). So the scene and the map can never disagree.

Two kinds of objects:
  - Model: a model from CoppeliaSim's library (walls, doors, racks, tables, conveyor,
    ArUco markers, people...). Sizes measured with src\inspect_models.py
    (see media\library\models.json).
  - Part: our own simple shape (box / cylinder), only for what the library does not have
    (cable machines, crimping presses, wiring boards, cable reels, pallets, drop tables, paint).

Frame: the hall floor is the rectangle x in [0, 20] m, y in [0, 12] m, z = 0 is the floor.
Main aisle runs along x, between y = 4.75 and y = 7.25.
Top row (y > 7.75): Magasin, Coupe/dénudage, Sertissage.
Bottom row (y < 4.25): Expédition, Test électrique (+ reject conveyor), Assemblage.
"""

import math
from dataclasses import dataclass, field

HALL_X, HALL_Y = 20.0, 12.0
WALL_H = 2.4                         # library "240cm high walls"
AISLE_Y0, AISLE_Y1 = 4.75, 7.25

TOP_Y0, TOP_Y1 = 7.75, 11.85        # top row of stations
BOT_Y0, BOT_Y1 = 0.15, 4.25         # bottom row of stations
COL_X = [(0.3, 6.0), (7.0, 13.0), (14.0, 19.7)]   # the three station columns
COL_CX = [3.15, 10.0, 16.85]                        # column centres (drop tables)

DROP_TOP_Y, DROP_BOT_Y = 8.10, 3.90  # centre of the drop tables ("postes de dépose"); front faces 7.85 / 4.15
ROBOT_TOP_Y, ROBOT_BOT_Y = 7.0, 5.0  # where the robot stops in front of them
# Height of the QC cell's conveyor and UR5 base.
PICK_H = 0.45
# AMR transfer tables ("postes de dépose"): the youBot arm (shoulder 0.25 m high, links 0.155 +
# 0.135 m) cannot reach higher than ~0.4 m at the table's depth (measured in Part 7). Real AMR
# transfer stations are built at the robot's height too.
TABLE_H = 0.25
TABLE_SIZE = (0.80, 0.50)            # along the aisle, depth
# Transport unit: small tote ("bac") with a handle fin on top that the gripper (opens ~7 cm) holds.
TOTE = (0.20, 0.14, 0.08)            # along the aisle, depth (= robot approach direction), height
# Handle: short square post (thickness, length, height). The youBot finger pads are only 4.5 cm long
# with the palm right behind them: a longer fin hit the palm before the fingers closed (Part 7).
FIN = (0.03, 0.03, 0.05)
TOTE_MASS = 0.3                      # kg (youBot arm payload: 0.5 kg)
TOTE_INSET = 0.10                    # tote centre behind the table's front edge
# Test électrique = quality-control cell: test benches -> inspection conveyor (along +x) under a
# camera -> UR5 arm with suction pad sorts each harness box: OK table (-> Expédition) or REBUTS
# table (-> Assemblage, rework). Both tables are at the aisle edge for the AMR.
# Regrouped around the UR5 (Part 9): the conveyor end and the two tote spots on the tables (where
# the youBot picks them) are all ~0.73 m from the arm base (UR5 reach 0.85 m).
TEST_DROP_X, REBUTS_X = 10.08, 10.92            # OK table / REBUTS table centres (side by side)
QC_Y, QC_X0, QC_X1 = 2.75, 7.3, 10.5            # inspection conveyor: axis y, start x, end x
QC_CAM_X = 9.0                                   # camera gantry above the conveyor
ARM_X, ARM_Y = 10.5, 3.45                        # UR5 base, right behind the two tables
QC_SPEED = 0.10                                  # m/s, conveyor belt
QC_TOTES = [(7.5, True), (8.1, False), (8.7, True)]   # (x, QC tag OK?) at start: all pass under the camera
# where the UR5 puts sorted totes on a table: front spot (the youBot picks it there), then a buffer
# spot behind it, closer to the arm. y offsets from the table centre (bottom row: front = +y)
QC_SLOTS_DY = (TABLE_SIZE[1] / 2 - TOTE_INSET, -0.10)

S, N, W, E = -math.pi / 2, math.pi / 2, math.pi, 0.0   # headings (direction an object faces)

# Stations: label (floor text, no accents: the 3D alphabet has none), ArUco id,
# and the robot's target pose (x, y, heading in rad) to load/unload there.
STATIONS = {
    "magasin":     dict(label="MAGASIN",          marker_id=1, pose=(COL_CX[0], ROBOT_TOP_Y, N)),
    "coupe":       dict(label="COUPE DENUDAGE",   marker_id=2, pose=(COL_CX[1], ROBOT_TOP_Y, N)),
    "sertissage":  dict(label="SERTISSAGE",       marker_id=3, pose=(COL_CX[2], ROBOT_TOP_Y, N)),
    "assemblage":  dict(label="ASSEMBLAGE",       marker_id=4, pose=(COL_CX[2], ROBOT_BOT_Y, S)),
    "test":        dict(label="TEST ELECTRIQUE",  marker_id=5, pose=(TEST_DROP_X, ROBOT_BOT_Y, S)),
    "expedition":  dict(label="EXPEDITION",       marker_id=6, pose=(COL_CX[0], ROBOT_BOT_Y, S)),
    "dock":        dict(label="DOCK",             marker_id=0, pose=(1.1, 6.0, W)),
    # REBUTS table of the QC cell: rejected harnesses to take back to Assemblage for rework
    "rebuts":      dict(label="REBUTS",           marker_id=7, pose=(REBUTS_X, ROBOT_BOT_Y, S)),
}
ARUCO_DICT = "4X4_50"    # dictionary used by the library marker model (choice 1)
ARUCO_SIZE = 0.15        # marker side (m)

C = dict(
    aisle=(0.33, 0.45, 0.38), yellow=(0.95, 0.80, 0.10), white=(0.92, 0.92, 0.92),
    black=(0.05, 0.05, 0.05), dock=(0.10, 0.70, 0.30), steel=(0.50, 0.52, 0.55),
    wood=(0.62, 0.45, 0.27), wood_light=(0.80, 0.66, 0.46), cardboard=(0.72, 0.55, 0.35),
    machine=(0.86, 0.87, 0.89), hood=(0.30, 0.45, 0.65), dark=(0.20, 0.22, 0.25),
    screen=(0.05, 0.05, 0.08), press=(0.95, 0.75, 0.10), board=(0.85, 0.75, 0.55),
    bench_top=(0.75, 0.62, 0.45), bench_legs=(0.35, 0.40, 0.48),
    drop_top=(0.95, 0.50, 0.10), drop_base=(0.30, 0.33, 0.38), bin=(0.15, 0.35, 0.75),
    test_plate=(0.10, 0.45, 0.20), copper=(0.80, 0.50, 0.20),
    red=(0.80, 0.10, 0.10), orange=(0.95, 0.50, 0.10), green=(0.10, 0.65, 0.20),
    blue=(0.10, 0.30, 0.80), cable_yellow=(0.95, 0.85, 0.10), cable_black=(0.08, 0.08, 0.08),
)
CABLES = [C["red"], C["blue"], C["cable_black"], C["cable_yellow"], C["green"], C["orange"]]

# Library models used, with their measured size (x, y, z) in the model's own frame.
LIB = {
    "floor":      ("infrastructure/floors/5mX5m concrete floor.ttm", (5.0, 5.0, 0.202)),
    "wall400":    ("infrastructure/walls/240cm high walls/wall section 400cm.ttm", (0.15, 4.0, 2.4)),
    "wall200":    ("infrastructure/walls/240cm high walls/wall section 200cm.ttm", (0.15, 2.0, 2.4)),
    "wall100":    ("infrastructure/walls/240cm high walls/wall section 100cm.ttm", (0.15, 1.0, 2.4)),
    "wall50":     ("infrastructure/walls/240cm high walls/wall section 50cm.ttm", (0.15, 0.5, 2.4)),
    "wall25":     ("infrastructure/walls/240cm high walls/wall section 25cm.ttm", (0.15, 0.25, 2.4)),
    "wall5":      ("infrastructure/walls/240cm high walls/wall section 5cm.ttm", (0.15, 0.05, 2.4)),
    "pillar":     ("infrastructure/walls/240cm high walls/pillar 50cm.ttm", (0.5, 0.5, 2.4)),
    "window":     ("infrastructure/windows/140cm window.ttm", (0.18, 1.4, 2.4)),
    "door2":      ("infrastructure/doors/automatic double sliding door.ttm", (0.19, 3.3, 2.4)),
    "rack":       ("furniture/shelves-cupboards-racks/rack.ttm", (0.3, 1.2, 1.97)),
    "cupboard":   ("furniture/shelves-cupboards-racks/deep cupboard.ttm", (0.6, 1.22, 1.97)),
    "table":      ("furniture/tables/customizable table.ttm", None),       # size set per table
    "conveyor":   ("equipment/conveyors/generic conveyor (belt).ttm", None),  # size set per conveyor
    "fence2":     ("equipment/panes/pane (grid) 2.0 x 2.0.ttm", (2.0, 0.036, 2.0)),
    "basket":     ("household/largeBasket.ttm", (0.805, 0.805, 0.185)),
    "laptop":     ("office items/laptop.ttm", (0.18, 0.35, 0.41)),
    "aruco":      ("other/aruco marker.ttm", (ARUCO_SIZE, ARUCO_SIZE, 0.005)),
    "bill_stand": ("people/Standing Bill.ttm", (0.34, 0.52, 1.68)),
    "bill_work":  ("people/Working Bill.ttm", (0.38, 0.56, 1.68)),
    "ur5":        ("robots/non-mobile/UR5.ttm", (0.28, 0.15, 1.05)),        # + suction pad, see build_scene
    "camera_qc":  ("components/sensors/Blob detection camera.ttm", (0.045, 0.112, 0.045)),  # looks along its +y
}
RACK_SHELVES = (0.08, 0.45, 0.75, 1.05, 1.35, 1.65)   # shelf top heights of the library rack (measured)


@dataclass
class Part:
    name: str
    shape: str              # "box" or "cyl" (cylinder axis = local z)
    size: tuple             # box: (sx, sy, sz); cyl: (diameter, diameter, length)
    pos: tuple              # centre (x, y, z), in world frame or in the parent's frame
    color: tuple
    group: str              # station / zone it belongs to (scene hierarchy)
    rot: tuple = (0.0, 0.0, 0.0)  # Euler angles (rad), same frame as pos
    parent: str | None = None     # name of another Part: pos/rot are then relative to it
    kind: str = "solid"     # "solid": obstacle, physical, static, seen by sensors
                            # "dynamic": like solid, but moved by physics (e.g. bins on the conveyor)
                            # "paint": floor marking / decoration, no physics, no sensing
                            # "merge": merged into its parent (one rigid body, e.g. a tote's handle)
    mass: float = 0.5       # kg, "dynamic" parts only


@dataclass
class Model:
    """A model loaded from CoppeliaSim's library."""
    name: str
    key: str                # key in LIB
    pos: tuple              # (x, y, z): where the model's ORIGIN goes (z=None: put its bottom on the floor)
    group: str
    rot: tuple = (0.0, 0.0, 0.0)  # Euler angles (rad); a heading alone = (0, 0, heading)
    setup: dict = field(default_factory=dict)  # model settings, e.g. conveyor length, ArUco id, table size
    scale: tuple | None = None    # non-uniform scaling (x, y, z) of the model, e.g. floor tiles


@dataclass
class Walker:
    """A worker who walks back and forth along a route (library 'Bill on path').
    NOT part of the robot's map: the robot discovers him with its lidar."""
    name: str
    route: list             # [(x, y), ...] waypoints, walked start -> end -> back, forever
    speed: float = 0.8      # m/s (library default 1.0 m/s, a bit fast for a workshop)
    pause_start: float = 3.0    # s waiting at the first point
    pause_end: float = 3.0      # s waiting at the last point
    role: str = ""


# Workers give way to the robot (like real operators trained to work next to AMRs): a worker stops
# while the robot's centre is in front of him, closer than YIELD_DIST and less than YIELD_WIDTH to
# the side; after YIELD_MAX s he walks on anyway (the robot must then move out of his way).
YIELD_DIST, YIELD_WIDTH, YIELD_MAX = 1.3, 0.7, 20.0

# Part 6: walking workers, each one testing a different robot behaviour
WALKERS = [
    Walker("operateur_allee", [(2.6, 6.1), (18.6, 6.1)], 0.8, 4.0, 4.0,
           "walks the whole main aisle: head-on encounters / overtaking -> slow down, sideways dodge"),
    Walker("operateur_traversee", [(15.0, 8.6), (15.0, 3.4)], 0.8, 5.0, 5.0,
           "crosses the aisle between Sertissage and Assemblage -> stop and wait until he has passed"),
    Walker("operateur_arret", [(7.6, 8.8), (9.4, 6.4)], 0.7, 6.0, 12.0,
           "walks into the aisle in front of Coupe and stands there 12 s -> wait, then re-plan around him"),
]


@dataclass
class Label:
    text: str
    x: float
    y: float
    height: float
    group: str
    color: tuple = (0.92, 0.92, 0.92)


PARTS: list[Part] = []
MODELS: list[Model] = []
LABELS: list[Label] = []
_group = "batiment"
_names: dict[str, int] = {}


def _unique(name):
    n = _names.get(name, 0)
    _names[name] = n + 1
    return name if n == 0 else f"{name}_{n}"


def group(name):
    global _group
    _group = name


def box(name, size, pos, color, **kw):
    PARTS.append(Part(_unique(name), "box", tuple(size), tuple(pos), color, _group, **kw))
    return PARTS[-1].name


def cyl(name, diameter, length, pos, color, **kw):
    PARTS.append(Part(_unique(name), "cyl", (diameter, diameter, length), tuple(pos), color, _group, **kw))
    return PARTS[-1].name


def model(name, key, x, y, z=None, heading=0.0, rot=None, **kw):
    MODELS.append(Model(_unique(name), key, (x, y, z), _group, rot or (0.0, 0.0, heading), **kw))
    return MODELS[-1].name


def label(text, x, y, height=0.3, color=C["white"]):
    LABELS.append(Label(text, x, y, height, _group, color))


def paint_line(name, x0, y0, x1, y1, width, color, z=0.0045):
    """Floor marking from (x0, y0) to (x1, y1) (horizontal or vertical only)."""
    sx = abs(x1 - x0) or width
    sy = abs(y1 - y0) or width
    box(name, (sx, sy, 0.005), ((x0 + x1) / 2, (y0 + y1) / 2, z), color, kind="paint")


def rect_outline(name, x0, y0, x1, y1, color=C["white"], width=0.06, z=0.0045):
    for a, b, c, d in ((x0, y0, x1, y0), (x0, y1, x1, y1), (x0, y0, x0, y1), (x1, y0, x1, y1)):
        paint_line(name, a, b, c, d, width, color, z)


# ---------------------------------------------------------------- building (library)
WALL_PIECES = [(4.0, "wall400"), (2.0, "wall200"), (1.0, "wall100"), (0.5, "wall50"), (0.25, "wall25"), (0.05, "wall5")]


def wall_run(side, start, end, openings=()):
    """One side of the hall ("N", "S", "W", "E") from `start` to `end` along it, filled with
    library wall sections, leaving room for openings = [(centre, key)] (windows, doors)."""
    along_x = side in ("N", "S")
    fixed = {"S": -0.075, "N": HALL_Y + 0.075, "W": -0.075, "E": HALL_X + 0.075}[side]
    heading = N if along_x else 0.0          # wall models are 'long' along their local y

    def put(key, a, b):
        c = (a + b) / 2
        x, y = (c, fixed) if along_x else (fixed, c)
        model(f"mur_{side}", key, x, y, heading=heading)

    cursor = start
    for centre, key in sorted(openings) + [(None, None)]:
        stop = end if centre is None else centre - LIB[key][1][1] / 2
        gap = round(stop - cursor, 3)
        for length, piece in WALL_PIECES:      # greedy fill with 4 / 2 / 1 / 0.5 / 0.25 / 0.05 m sections
            while gap >= length - 1e-6:
                put(piece, cursor, cursor + length)
                cursor += length
                gap = round(gap - length, 3)
        if centre is not None:
            put(key, centre - LIB[key][1][1] / 2, centre + LIB[key][1][1] / 2)
            cursor = centre + LIB[key][1][1] / 2


def aruco(station, x, y, z, facing):
    """Real ArUco marker (library model, generated with the station's id) on a vertical face.
    facing: N / S / E / W, the direction the printed side looks to (towards the aisle).
    (x, y) = position on the face. The marker is printed on a white plate: ArUco detection needs a
    light margin ("quiet zone") around the marker's black border."""
    # the model's printed side is its -z face (checked with the robot camera: the other way round,
    # OpenCV only decoded the mirrored image)
    rot = {S: (-math.pi / 2, 0.0, 0.0), N: (math.pi / 2, 0.0, 0.0), E: (0.0, -math.pi / 2, 0.0)}[facing]
    dx, dy = {S: (0.0, -1.0), N: (0.0, 1.0), E: (1.0, 0.0)}[facing]
    m = ARUCO_SIZE + 0.06
    box("aruco_fond", (0.002, m, m) if dx else (m, 0.002, m), (x + dx * 0.001, y + dy * 0.001, z), C["white"], kind="paint")
    x, y = x + dx * 0.0045, y + dy * 0.0045
    model(f"aruco_{STATIONS[station]['marker_id']}", "aruco", x, y, z, rot=rot,
          setup=dict(aruco=dict(id=STATIONS[station]["marker_id"], size=ARUCO_SIZE)))


def table(name, cx, cy, length, width, height, heading=0.0, top=C["bench_top"], legs=C["bench_legs"]):
    """Library 'customizable table', resized (length along its local x)."""
    model(name, "table", cx, cy, heading=heading,
          setup=dict(table=dict(length=length, width=width, height=height, color1=top, color2=legs)))


def rack_with_spools(name, cx, cy):
    """Library rack along x (against the north wall), with small cable spools on 3 shelves."""
    model(name, "rack", cx, cy, heading=N)
    for i, z in enumerate(RACK_SHELVES[1:4]):
        for k, dx in enumerate((-0.4, 0.0, 0.4)):
            reel("bobine", cx + dx, cy, z, 0.22, 0.18, CABLES[(i * 3 + k) % len(CABLES)], axis="y")


# ---------------------------------------------------------------- own shapes (not in the library)
def reel(name, x, y, z0, d, w, color, axis="y"):
    """Cable reel ("bobine"): wooden flanges + wound cable. z0 = lowest point.
    axis = "x", "y" (lying on its flanges' edge) or "z" (standing on a flange)."""
    rot = {"x": (0.0, math.pi / 2, 0.0), "y": (math.pi / 2, 0.0, 0.0), "z": (0.0, 0.0, 0.0)}[axis]
    zc = z0 + (w / 2 if axis == "z" else d / 2)
    core = cyl(name, d * 0.86, w - 0.04, (x, y, zc), color, rot=rot)
    for side in (-1, 1):  # flanges, placed in the core's own frame (its axis = local z)
        cyl(name + "_flasque", d, 0.025, (0, 0, side * (w / 2 - 0.0125)), C["wood"], parent=core)
    return core


def drop_table(station, cx, cy, facing, top=C["drop_top"]):
    """AMR transfer table ("poste de dépose"): low cabinet (TABLE_H high, closed down to the floor,
    so the lidar sees its front face) with the station's ArUco marker on the face towards the aisle."""
    sx, sy = TABLE_SIZE
    box("poste_depose_base", (sx, sy, TABLE_H - 0.02), (cx, cy, (TABLE_H - 0.02) / 2), C["drop_base"])
    box("poste_depose_plateau", (sx, sy, 0.02), (cx, cy, TABLE_H - 0.01), top)
    sign = -1 if facing == S else 1
    aruco(station, cx, cy + sign * sy / 2, (TABLE_H - 0.02) / 2, facing)


def tote_on_table(station, content=None, tag=None):
    """Tote waiting on a station's table, centred in front of the robot's docking position."""
    x, _, heading = STATIONS[station]["pose"]
    if heading == N:        # top row: the robot looks north at the table
        y = DROP_TOP_Y - TABLE_SIZE[1] / 2 + TOTE_INSET
    else:                   # bottom row
        y = DROP_BOT_Y + TABLE_SIZE[1] / 2 - TOTE_INSET
    return tote(f"bac_{station}", x, y, TABLE_H + 0.002, content, tag)


def tote(name, x, y, z0, content=None, tag=None):
    """Transport tote with a handle fin on top (one rigid body), dynamic: the conveyor carries it,
    the arms pick it. content: 'faisceau' (harness) or 'bobines' (reels). tag: True = green QC tag
    (OK), False = red (defect), None = no tag. The QC camera reads the tag colour."""
    sx, sy, sz = TOTE
    b = box(name, TOTE, (x, y, z0 + sz / 2), C["bin"], kind="dynamic", mass=TOTE_MASS)
    box(name + "_poignee", FIN, (0.0, 0.0, sz / 2 + FIN[2] / 2), C["dark"], parent=b, kind="merge")
    if content == "faisceau":
        cyl(name + "_faisceau", 0.10, 0.02, (-0.05, 0.0, sz / 2 + 0.01), C["cable_black"], parent=b, kind="paint")
    elif content == "bobines":
        for dy in (-0.035, 0.035):
            cyl(name + "_bobine", 0.06, 0.03, (-0.05, dy, sz / 2 + 0.015), CABLES[0], parent=b, kind="paint")
    if tag is not None:
        box(name + "_etiquette", (0.05, 0.05, 0.004), (0.06, 0.0, sz / 2 + 0.002),
            C["green"] if tag else C["red"], parent=b, kind="paint")
    return b


def pallet(name, cx, cy):
    """Euro pallet 1.2 x 0.8 m."""
    for dy in (-0.33, 0.0, 0.33):
        box(name + "_semelle", (1.2, 0.10, 0.10), (cx, cy + dy, 0.05), C["wood_light"])
    box(name + "_plateau", (1.2, 0.8, 0.044), (cx, cy, 0.122), C["wood_light"])
    return 0.144  # top height


def box_stack(name, cx, cy, z0, layers):
    """Cardboard boxes (0.6 x 0.4 x 0.3) stacked 2 x 2 on a pallet."""
    for k in range(layers):
        for dx in (-0.3, 0.3):
            for dy in (-0.2, 0.2):
                box(name, (0.58, 0.38, 0.29), (cx + dx, cy + dy, z0 + 0.15 + 0.3 * k), C["cardboard"])


def light_tower(name, x, y, z0):
    """Signal tower (red / orange / green), typical on test benches and machines."""
    cyl(name + "_mat", 0.03, 0.25, (x, y, z0 + 0.125), C["steel"])
    for i, col in enumerate((C["green"], C["orange"], C["red"])):
        cyl(name + "_feu", 0.07, 0.07, (x, y, z0 + 0.285 + 0.07 * i), col)


def cutting_machine(name, cx, cy):
    """Wire cutting/stripping machine (Komax-like): cabinet, hood, screen, feeding reel."""
    box(name + "_bati", (1.8, 0.8, 0.9), (cx, cy, 0.45), C["machine"])
    box(name + "_capot", (1.6, 0.7, 0.45), (cx, cy, 1.125), C["hood"])
    box(name + "_vitre", (1.2, 0.01, 0.25), (cx, cy - 0.355, 1.125), C["dark"], kind="paint")
    box(name + "_bac_sortie", (0.6, 0.35, 0.10), (cx - 0.5, cy - 0.55, 0.80), C["steel"])
    box(name + "_ecran_pied", (0.05, 0.05, 0.45), (cx + 0.7, cy - 0.30, 1.575), C["steel"])
    box(name + "_ecran", (0.45, 0.04, 0.30), (cx + 0.7, cy - 0.34, 1.90), C["screen"])
    light_tower(name, cx - 0.8, cy + 0.3, 1.35)
    box(name + "_support_bobine", (0.5, 0.8, 0.08), (cx + 1.4, cy, 0.04), C["dark"])
    reel(name + "_bobine", cx + 1.4, cy, 0.08, 0.7, 0.4, C["red"], axis="x")


def crimping_press(name, cx, cy, z0):
    """Bench-top crimping press ("presse de sertissage") with its small terminal reel."""
    box(name + "_socle", (0.35, 0.40, 0.12), (cx, cy, z0 + 0.06), C["press"])
    box(name + "_colonne", (0.14, 0.14, 0.55), (cx, cy + 0.12, z0 + 0.395), C["press"])
    box(name + "_tete", (0.30, 0.30, 0.20), (cx, cy - 0.02, z0 + 0.57), C["dark"])
    box(name + "_applicateur", (0.08, 0.08, 0.20), (cx, cy - 0.06, z0 + 0.37), C["steel"])
    reel(name + "_bobine_contacts", cx, cy + 0.15, z0 + 0.67, 0.30, 0.08, C["copper"], axis="x")


def wiring_board(name, cx, cy, tilt=0.26):
    """Harness assembly board ("planche de câblage"): tilted plywood board on two legs,
    with a harness (trunk + branches + connectors) drawn on its front face (+y side)."""
    zc = 1.15
    board = box(name, (2.4, 0.05, 1.2), (cx, cy, zc), C["board"], rot=(tilt, 0.0, 0.0))
    leg_bottom = -zc / math.cos(tilt)       # local z where the leg reaches the floor
    leg_len = 0.5 - leg_bottom
    for sx in (-1.0, 1.0):
        box(name + "_pied", (0.06, 0.06, leg_len), (sx, -0.06, leg_bottom + leg_len / 2), C["steel"], parent=board)
        yb = cy + (-0.06 * math.cos(tilt) - leg_bottom * math.sin(tilt))
        box(name + "_patin", (0.08, 0.8, 0.05), (cx + sx, yb, 0.025), C["steel"])
    box(name + "_tronc", (2.0, 0.012, 0.03), (0.0, 0.035, 0.05), C["cable_black"], parent=board, kind="paint")
    branches = [(-0.85, 0.35, C["orange"]), (-0.45, -0.40, C["red"]), (0.0, 0.30, C["blue"]),
                (0.40, -0.35, C["cable_yellow"]), (0.80, 0.40, C["green"])]
    for x, length, col in branches:
        box(name + "_branche", (0.025, 0.012, abs(length)), (x, 0.035, 0.05 + length / 2), col, parent=board, kind="paint")
        box(name + "_connecteur", (0.07, 0.03, 0.05), (x, 0.045, 0.05 + length), C["white"], parent=board, kind="paint")
    for x in (-1.0, 1.0):
        box(name + "_connecteur", (0.07, 0.03, 0.05), (x, 0.045, 0.05), C["dark"], parent=board, kind="paint")


def test_bench(name, cx, cy):
    """Electrical test bench: library table + test cabinet, laptop, test plate with a harness, signal tower."""
    table(name, cx, cy, 2.2, 0.8, 0.9)
    box(name + "_armoire_test", (0.6, 0.45, 0.55), (cx - 0.75, cy + 0.1, 0.9 + 0.275), C["machine"])
    box(name + "_plaque_test", (1.1, 0.6, 0.02), (cx + 0.35, cy, 0.91), C["test_plate"], kind="paint")
    for i, col in enumerate(CABLES[:4]):
        cyl(name + "_cable", 0.015, 0.9, (cx + 0.35, cy - 0.2 + 0.13 * i, 0.93), col, rot=(0.0, math.pi / 2, 0.0), kind="paint")
    model(name + "_laptop", "laptop", cx - 0.3, cy + 0.2, 0.9 + 0.205, heading=S)
    light_tower(name, cx + 1.0, cy + 0.3, 0.9)


def inspection_conveyor(name, x0, x1, y, width=0.5, h=PICK_H):
    """Library belt conveyor (really moves the boxes during simulation) on our own stand,
    running along +x from x0 to x1, belt top at height h, red end stop at x1."""
    radius = 0.1
    length = (x1 - x0) - 2 * radius - 0.04     # distance between the end drums
    model(name, "conveyor", (x0 + x1) / 2, y, h - 0.02, heading=E,
          setup=dict(conveyor=dict(length=length, width=width, radius=radius, targetVel=QC_SPEED,
                                   color=[0.08, 0.08, 0.08], frameColor=list(C["steel"]))))
    leg_h = h - 0.23
    for xx in (x0 + 0.3, (x0 + x1) / 2, x1 - 0.3):
        for side in (-1, 1):
            box(name + "_pied", (0.05, 0.05, leg_h), (xx, y + side * (width / 2 - 0.03), leg_h / 2), C["dark"])
        box(name + "_traverse", (0.05, width, 0.05), (xx, y, 0.12), C["dark"])
    box(name + "_butee", (0.04, width + 0.06, 0.12), (x1 + 0.02, y, h + 0.04), C["red"])
    box(name + "_moteur", (0.25, 0.18, 0.18), (x0 + 0.3, y - width / 2 - 0.09, h - 0.25), C["bin"])


def camera_gantry(name, x, y, span=1.1, h=1.35):
    """Portal over the conveyor carrying the quality-control camera, looking straight down."""
    for side in (-1, 1):
        box(name + "_montant", (0.06, 0.06, h), (x, y + side * span / 2, h / 2), C["press"])
    box(name + "_traverse", (0.08, span + 0.06, 0.08), (x, y, h + 0.04), C["press"])
    # the camera looks along its own +y: rotating -90 deg about x makes it look down (-z)
    model("camera_controle_qualite", "camera_qc", x, y, h - 0.03, rot=(-math.pi / 2, 0.0, 0.0),
          setup=dict(camera=dict(resolution=256)))


def ur5_cell(name, x, y):
    """UR5 collaborative arm with a suction pad, on a pedestal at pick height, with a yellow
    safety outline on the floor around it. Its library demo script is removed in build_scene."""
    box(name + "_socle", (0.40, 0.40, PICK_H), (x, y, PICK_H / 2), C["dark"])
    model(name, "ur5", x, y, PICK_H + 0.015, heading=W, setup=dict(ur5=dict(suction_pad=True)))
    rect_outline(name + "_zone", x - 0.55, y - 0.45, x + 0.55, y + 0.15, C["yellow"], 0.06, 0.005)


def bill(name, key, x, y, heading):
    model(name, key, x, y, heading=heading)


# ---------------------------------------------------------------- the workshop
def _build():
    group("batiment")
    for ix in range(4):                       # 5 x 5 m concrete tiles, scaled to 5 x 4 m: 4 x 3 tiles = 20 x 12 m
        for iy in range(3):
            model("sol", "floor", 2.5 + 5 * ix, 2.0 + 4 * iy, -0.102, scale=(1.0, 0.8, 1.0))
    for x in (0.0, HALL_X):                   # corner pillars
        for y in (0.0, HALL_Y):
            model("poteau_angle", "pillar", x, y)
    wall_run("S", 0.25, HALL_X - 0.25, [(2.95, "door2"), (10.0, "window"), (16.85, "window")])
    wall_run("N", 0.25, HALL_X - 0.25, [(10.0, "window"), (16.85, "window")])
    wall_run("W", 0.25, HALL_Y - 0.25, [(2.5, "window"), (9.9, "door2")])
    wall_run("E", 0.25, HALL_Y - 0.25, [(2.6, "window"), (6.0, "window"), (9.4, "window")])
    for x in (6.5, 13.5):                     # building pillars along the aisle + grid fences between zones
        model("poteau", "pillar", x, AISLE_Y0 - 0.3)
        model("poteau", "pillar", x, AISLE_Y1 + 0.3)
        for y in (BOT_Y0 + 0.05, BOT_Y0 + 2.05):
            model("grillage", "fence2", x, y, heading=N)
        for y in (TOP_Y0 + 0.25, TOP_Y0 + 2.25):     # panes run +2 m along y from their origin
            model("grillage", "fence2", x, y, heading=N)

    group("allee")
    box("allee_principale", (HALL_X - 0.6, AISLE_Y1 - AISLE_Y0, 0.004), (HALL_X / 2, (AISLE_Y0 + AISLE_Y1) / 2, 0.002), C["aisle"], kind="paint")
    for y in (AISLE_Y0, AISLE_Y1):
        paint_line("ligne_jaune", 0.3, y, HALL_X - 0.3, y, 0.10, C["yellow"])
    for (x0, x1) in COL_X:
        rect_outline("zone", x0, TOP_Y0, x1, TOP_Y1)
        rect_outline("zone", x0, BOT_Y0, x1, BOT_Y1)
    rect_outline("quai_expedition", 1.3, 0.15, 4.6, 0.6, C["yellow"], 0.08, 0.005)
    rect_outline("quai_reception", 0.15, 8.25, 0.6, 11.55, C["yellow"], 0.08, 0.005)

    group("dock")
    box("dock_sol", (1.2, 1.2, 0.005), (0.75, 6.0, 0.007), C["dock"], kind="paint")
    box("borne_recharge", (0.35, 0.8, 0.6), (0.175, 6.0, 0.30), C["dock"])
    for dy in (-0.17, 0.17):                # charging contacts, on both sides of the marker
        box("contact", (0.02, 0.08, 0.04), (0.36, 6.0 + dy, 0.12), C["copper"], kind="paint")
    aruco("dock", 0.35, 6.0, 0.15, E)       # low, like the stations' markers: in the camera's view
    label("DOCK", 1.6, 6.0, 0.25)

    group("magasin")
    for cx in (1.25, 2.55, 3.85, 5.15):
        rack_with_spools("rayonnage", cx, 11.6)
    for i, x in enumerate((2.0, 3.3, 4.6)):
        reel("touret", x, 9.9, 0.0, 1.0, 0.6, CABLES[i], axis="y")
    top = pallet("palette", 5.1, 8.75)
    for dx in (-0.3, 0.3):
        for dy in (-0.2, 0.2):
            reel("bobine", 5.1 + dx, 8.75 + dy, top, 0.35, 0.30, CABLES[3], axis="z")
    drop_table("magasin", COL_CX[0], DROP_TOP_Y, S)
    tote_on_table("magasin", content="bobines")            # reels ready to go to Coupe

    group("coupe")
    cutting_machine("machine_coupe", 8.4, 10.9)
    cutting_machine("machine_coupe", 11.4, 10.9)
    table("etabli", 11.6, 9.3, 1.8, 0.7, 0.85)
    for i, col in enumerate(CABLES[:5]):
        cyl("faisceau_fils", 0.06, 0.8, (11.6, 9.08 + 0.11 * i, 0.88), col, rot=(0.0, math.pi / 2, 0.0))
    model("bac_fils_coupes", "basket", 9.4, 9.3)
    drop_table("coupe", COL_CX[1], DROP_TOP_Y, S)
    bill("operateur", "bill_work", 8.4, 10.05, N)

    group("sertissage")
    table("etabli_presses", COL_CX[2], 11.3, 4.4, 0.8, 0.9)
    for x in (15.2, 16.85, 18.5):
        crimping_press("presse", x, 11.3, 0.9)
    table("etabli_bacs", 16.3, 9.6, 3.0, 0.7, 0.9)
    for k in range(6):
        box("bac", (0.4, 0.3, 0.15), (15.05 + 0.5 * k, 9.6, 0.975), C["bin"])
    model("armoire", "cupboard", 19.38, 9.4, heading=W)
    drop_table("sertissage", COL_CX[2], DROP_TOP_Y, S)
    bill("operateur", "bill_stand", COL_CX[2], 10.4, N)

    group("assemblage")
    wiring_board("planche_cablage", 15.4, 1.6)
    wiring_board("planche_cablage", 18.3, 1.6)
    model("armoire", "cupboard", 19.38, 3.4, heading=W)
    drop_table("assemblage", COL_CX[2], DROP_BOT_Y, N)
    bill("operateur", "bill_stand", 15.4, 2.9, S)

    group("test")     # quality-control cell (see constants at the top)
    test_bench("banc_test", 8.2, 0.95)
    test_bench("banc_test", 11.8, 0.95)
    bill("operateur", "bill_stand", 8.2, 1.75, S)
    bill("operateur", "bill_stand", 11.8, 1.75, S)
    inspection_conveyor("convoyeur_controle", QC_X0, QC_X1, QC_Y)
    camera_gantry("portique_camera", QC_CAM_X, QC_Y)
    light_tower("convoyeur_controle", QC_X0 + 0.1, QC_Y + 0.4, 0.0)
    for x, ok in QC_TOTES:                                               # totes travelling to the arm
        tote("bac_faisceau", x, QC_Y, PICK_H + 0.005, "faisceau", ok)
    ur5_cell("ur5", ARM_X, ARM_Y)
    drop_table("test", TEST_DROP_X, DROP_BOT_Y, N)                       # OK -> Expédition (empty: the UR5 fills it)

    group("rebuts")   # REBUTS table of the QC cell: rejected totes -> AMR -> Assemblage (rework)
    drop_table("rebuts", REBUTS_X, DROP_BOT_Y, N, top=C["red"])
    label("REBUTS", 11.85, 3.3, 0.2, C["red"])

    group("expedition")
    for x, layers in ((1.4, 2), (3.0, 3), (4.6, 1)):
        top = pallet("palette", x, 1.2)
        box_stack("carton", x, 1.2, top, layers)
    table("table_emballage", 1.3, 2.9, 1.6, 0.8, 0.85)
    box("carton", (0.58, 0.38, 0.29), (1.1, 2.9, 0.85 + 0.145), C["cardboard"])
    drop_table("expedition", COL_CX[0], DROP_BOT_Y, N)
    bill("operateur", "bill_stand", 4.6, 2.4, S)

    for key in ("magasin", "coupe", "sertissage", "assemblage", "test", "expedition"):
        group(key)
        x, y, _ = STATIONS[key]["pose"]
        label(STATIONS[key]["label"], x, 6.85 if y > 6 else 5.15)


_build()
