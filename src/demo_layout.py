r"""youBot demo arena ("showroom"): the test scene scenes\test_youbot.ttt, as plain data.

Same idea as src\layout.py (single source of truth for the scene AND the robot's map), with the
same helpers, in a 10 x 8 m arena enclosed by low walls. One zone per capability of the robot:

    north   POSTE B (table, id 2)   <- worker crossing + stray box ->   POSTE A (table, id 1, tote)
            DOCK (charging, id 0)
    ------- divider wall --------------------------------------------   CHICANE (S-bend)
    south   CAPTEURS (start, 3 test obstacles)   MECANUM (1 m square)

The robot starts in the sensor zone (the 3 obstacles of Part 3: tests\test_sensors.py still uses
them), shows its mecanum moves on the painted square, plans its way through the chicane, docks at
POSTE A and loads the tote with its arm, carries it to POSTE B past a walking worker and a box that
is NOT on its map (it must re-plan), unloads it, and goes back to its charging dock.
Run src\demo_youbot.py for the whole show.
"""

import layout
from layout import C, N, S, E, W, TABLE_H, TABLE_SIZE, TOTE_INSET, Part, Walker, box, cyl, model, label, paint_line, rect_outline

ARENA_X, ARENA_Y = 10.0, 8.0
WALL_H, WALL_T = 0.6, 0.10          # low walls: the GUI camera sees over them, the lidar (13 cm) sees them
DIVIDER_Y, DIVIDER_X1 = 4.0, 6.8    # wall between the south and north halves, from x = 0 to DIVIDER_X1
CHICANE_Y = (3.2, 4.8)              # the two baffles of the S-bend in the east passage
TABLE_Y = 7.35                      # transfer tables against the north wall (front face 7.10)
ROBOT_Y = 6.25                      # robot pose in front of them (same 0.85 m as in the workshop)

START = (1.5, 2.0, N)               # robot start pose, in the sensor zone
MECANUM_SQUARE = (3.3, 1.5, 1.0)    # corner (x, y) and side of the painted square

STATIONS = {
    "poste_a": dict(label="POSTE A", marker_id=1, pose=(8.6, ROBOT_Y, N)),
    "poste_b": dict(label="POSTE B", marker_id=2, pose=(2.2, ROBOT_Y, N)),
    "dock":    dict(label="DOCK",    marker_id=0, pose=(1.1, 5.0, W)),
}
TOTE_NAME = "bac_demo"

# Pedestrian crossing (worker's lane). Far enough from the stray box: the robot stops ~1 m before the
# box, and that stop must be OUT of the lane (box 1.3 m after the lane: the robot waited ON it, re-planned
# into it and the worker walked into the robot at the end of his pause)
CROSSING_X = 6.2
WALKERS = [
    # He does NOT give way to the robot: the robot must cope on its own. He walks the 3.1 m crossing in
    # ~4.4 s, then stands 15 s at each end: the robot sees him moving only in short windows and has a
    # long gap to cross safely once he is out of the way.
    Walker("operateur_demo", [(CROSSING_X, 4.5), (CROSSING_X, 7.6)], 0.7, 15.0, 15.0,
           "crosses the route between POSTE A and POSTE B -> slow down / stop / pass while he waits",
           yields=False),
]
STRAY_BOX = ((0.5, 0.5, 0.4), (3.9, 6.2))   # size, position: in the scene but NOT on the robot's map
# tests\test_tracking.py: robot standing between the box and the crossing, looking north-east (60 deg):
# it sees the worker cross (moving) and the stray box (static), both off the map
TRACK_VIEW = (5.0, 5.0, 1.05)


def _wall(x0, y0, x1, y1):
    """Low wall from (x0, y0) to (x1, y1) (horizontal or vertical), centred on that line."""
    sx = abs(x1 - x0) or WALL_T
    sy = abs(y1 - y0) or WALL_T
    box("barriere", (sx, sy, WALL_H), ((x0 + x1) / 2, (y0 + y1) / 2, WALL_H / 2), C["steel"])


def _table(key):
    x, _, _ = STATIONS[key]["pose"]
    layout.drop_table(key, x, TABLE_Y, S, marker_id=STATIONS[key]["marker_id"])
    label(STATIONS[key]["label"], x, ROBOT_Y - 0.75, 0.25)


def _build():
    layout.group("sol")
    for ix in range(2):                     # 2 x 2 library concrete tiles (5 x 5 m, scaled to 5 x 4 m)
        for iy in range(2):
            model("sol", "floor", 2.5 + 5 * ix, 2.0 + 4 * iy, -0.102, scale=(1.0, 0.8, 1.0))

    layout.group("murs")
    h = WALL_T / 2
    _wall(-h, -h, ARENA_X + h, -h)
    _wall(-h, ARENA_Y + h, ARENA_X + h, ARENA_Y + h)
    _wall(-h, -h, -h, ARENA_Y + h)
    _wall(ARENA_X + h, -h, ARENA_X + h, ARENA_Y + h)
    _wall(0.0, DIVIDER_Y, DIVIDER_X1, DIVIDER_Y)

    # Part 3 test obstacles (same places relative to the start pose as before): box 1.2 m ahead,
    # post 0.7 m to the left, wall 1.0 m behind
    layout.group("zone_capteurs")
    x, y, _ = START
    box("obstacle_avant", (0.5, 0.5, 0.5), (x, y + 1.2, 0.25), C["orange"])
    cyl("poteau_gauche", 0.12, 1.0, (x - 0.7, y, 0.5), C["orange"])
    box("mur_arriere", (2.0, 0.1, 0.5), (x, y - 1.0, 0.25), C["orange"])
    rect_outline("zone", 0.3, 0.6, 2.7, 3.7, C["orange"])
    label("CAPTEURS", x, 0.35, 0.2, C["orange"])

    layout.group("zone_mecanum")
    cx, cy, side = MECANUM_SQUARE
    rect_outline("carre", cx, cy, cx + side, cy + side, C["yellow"], 0.05)
    for dx, dy in ((0, 0), (side, 0), (side, side), (0, side)):           # corner pads
        box("coin", (0.12, 0.12, 0.005), (cx + dx, cy + dy, 0.006), C["yellow"], kind="paint")
    label("MECANUM", cx + side / 2, cy - 0.5, 0.2, C["yellow"])

    # S-bend in the east passage: in from the south on the west side, out to the north on the east side
    layout.group("chicane")
    _wall(DIVIDER_X1 + 1.2, CHICANE_Y[0], ARENA_X, CHICANE_Y[0])          # from the east wall
    _wall(DIVIDER_X1, CHICANE_Y[1], ARENA_X - 1.4, CHICANE_Y[1])          # from the divider's end
    _wall(DIVIDER_X1, DIVIDER_Y, DIVIDER_X1, CHICANE_Y[1])
    for x0, y0 in ((DIVIDER_X1 + 1.2, CHICANE_Y[0]), (ARENA_X - 1.4, CHICANE_Y[1])):
        cyl("balise", 0.12, 0.8, (x0, y0, 0.4), C["yellow"])                 # yellow beacons at the baffle ends
    label("CHICANE", 8.4, 2.4, 0.2, C["yellow"])
    paint_line("fleche", 7.4, 2.6, 7.4, 3.1, 0.06, C["white"])              # painted "way in / way out"
    paint_line("fleche", 9.4, 4.9, 9.4, 5.4, 0.06, C["white"])

    layout.group("postes")
    _table("poste_a")
    _table("poste_b")
    x, _, _ = STATIONS["poste_a"]["pose"]
    layout.tote(TOTE_NAME, x, TABLE_Y - TABLE_SIZE[1] / 2 + TOTE_INSET, TABLE_H + 0.002, content="bobines")
    rect_outline("passage", CROSSING_X - 0.5, 4.1, CROSSING_X + 0.5, ARENA_Y - 0.05, C["white"], 0.05)  # pedestrian crossing
    for k in range(7):
        box("zebra", (1.0, 0.18, 0.004), (CROSSING_X, 4.4 + 0.5 * k, 0.005), C["white"], kind="paint")

    layout.group("dock")
    x, y, _ = STATIONS["dock"]["pose"]
    box("dock_sol", (1.2, 1.2, 0.005), (0.75, y, 0.007), C["dock"], kind="paint")
    box("borne_recharge", (0.35, 0.8, 0.6), (0.175, y, 0.30), C["dock"])
    for dy in (-0.17, 0.17):
        box("contact", (0.02, 0.08, 0.04), (0.36, y + dy, 0.12), C["copper"], kind="paint")
    layout.aruco("dock", 0.35, y, 0.15, E, marker_id=STATIONS["dock"]["marker_id"])
    label("DOCK", 1.6, y - 0.75, 0.2)


with layout.collecting() as (PARTS, MODELS, LABELS):
    _build()

# The stray box is built in the scene but kept OUT of PARTS, so it is not on the robot's map
UNMAPPED = [Part("carton_egare", "box", STRAY_BOX[0], (*STRAY_BOX[1], STRAY_BOX[0][2] / 2), C["cardboard"], "imprevu")]
