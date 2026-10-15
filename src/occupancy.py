r"""Occupancy grid (the robot's fixed map) generated from src\layout.py.

The workshop floor is cut into square cells of RES metres. A cell is OCCUPIED if some object
of the layout covers it between the floor and the robot's height (things higher up, e.g. a
screen on a post, do not block the robot; the post does). People are NOT in the map: they move,
the lidar handles them at runtime.

Three layers:
  - occupied : the objects themselves
  - lethal   : occupied cells grown by the robot's radius (the robot's CENTRE must stay out)
  - cost     : soft extra cost near obstacles, so planned routes keep a comfortable distance
               (in the aisle: they stay near the middle)

    grid = OccupancyGrid.from_layout()
    i, j = grid.to_cell(x, y);  x, y = grid.to_world(i, j)
"""

import json
import math
from pathlib import Path

import cv2
import numpy as np

import layout

RES = 0.05                    # cell size (m)
X0, Y0, X1, Y1 = -0.5, -0.5, layout.HALL_X + 0.5, layout.HALL_Y + 0.5   # map extent (walls included)
ROBOT_HEIGHT = 0.5            # objects lower than this block the robot (youBot + sensors: 0.44 m)
ROBOT_RADIUS = 0.40           # circle containing the youBot + front sensors (0.36 fwd x 0.21 side -> 0.42)
COMFORT = 0.45                # beyond the lethal zone, extra cost decreases to 0 over this distance
NOT_OBSTACLES = {"floor", "aruco", "bill_stand", "bill_work", "laptop", "camera_qc"}  # floor / people / high or flat items

MODELS_JSON = Path(__file__).resolve().parent.parent / "media" / "library" / "models.json"


# ---------------------------------------------------------------- geometry helpers
def rot_matrix(a, b, g):
    """CoppeliaSim Euler angles: R = Rx(a) Ry(b) Rz(g)."""
    ca, sa, cb, sb, cg, sg = math.cos(a), math.sin(a), math.cos(b), math.sin(b), math.cos(g), math.sin(g)
    rx = np.array([[1, 0, 0], [0, ca, -sa], [0, sa, ca]])
    ry = np.array([[cb, 0, sb], [0, 1, 0], [-sb, 0, cb]])
    rz = np.array([[cg, -sg, 0], [sg, cg, 0], [0, 0, 1]])
    return rx @ ry @ rz


def box_corners(lo, hi):
    """8 corners of the box [lo, hi] (3-vectors)."""
    return np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])


def _model_origins():
    """Where each library model's origin sits inside its bounding box (measured by inspect_models.py)."""
    data = json.loads(MODELS_JSON.read_text(encoding="utf-8")) if MODELS_JSON.exists() else {}
    return {rel: v["origin_from_min_corner"] for rel, v in data.items()}


# ---------------------------------------------------------------- footprints of the layout
def part_boxes():
    """World corners (8 x 3) of every solid Part of the layout (parent chains resolved)."""
    world = {}                                       # name -> (R, t)
    out = []
    for p in layout.PARTS:
        r_loc, t_loc = rot_matrix(*p.rot), np.array(p.pos, float)
        if p.parent:
            r_par, t_par = world[p.parent]
            r, t = r_par @ r_loc, r_par @ t_loc + t_par
        else:
            r, t = r_loc, t_loc
        world[p.name] = (r, t)
        if p.kind == "paint":
            continue
        half = np.array(p.size, float) / 2
        out.append((p.name, box_corners(-half, half) @ r.T + t))
    return out


def model_boxes():
    """World corners of every obstacle-like library Model of the layout."""
    origins = _model_origins()
    out = []
    for m in layout.MODELS:
        if m.key in NOT_OBSTACLES:
            continue
        rel, size = layout.LIB[m.key]
        if m.key == "table":                              # size set per table
            v = m.setup["table"]
            size, origin = (v["length"], v["width"], v["height"]), (v["length"] / 2, v["width"] / 2, v["height"])
        elif m.key == "conveyor":                         # size set per conveyor
            v = m.setup["conveyor"]
            size = (v["length"] + 2 * v["radius"] + 0.04, v["width"], 0.25)
            origin = (size[0] / 2, size[1] / 2, 0.23)
        else:
            origin = origins.get(rel, [s / 2 for s in size])
        lo, hi = -np.array(origin, float), np.array(size, float) - np.array(origin, float)
        r = rot_matrix(*m.rot)
        corners = box_corners(lo, hi) @ r.T
        x, y, z = m.pos
        z = -corners[:, 2].min() if z is None else z          # same rule as build_scene: bottom on the floor
        out.append((m.name, corners + np.array([x, y, z])))
    return out


# ---------------------------------------------------------------- the grid
class OccupancyGrid:
    def __init__(self, occupied):
        self.occupied = occupied                      # bool array [row = y, col = x]
        self.h, self.w = occupied.shape
        r = int(math.ceil(ROBOT_RADIUS / RES))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
        self.lethal = cv2.dilate(occupied.astype(np.uint8), kernel).astype(bool)
        # distance (m) of every cell to the nearest occupied cell -> soft cost
        dist = cv2.distanceTransform((~occupied).astype(np.uint8), cv2.DIST_L2, 5) * RES
        self.cost = np.clip((ROBOT_RADIUS + COMFORT - dist) / COMFORT, 0.0, 1.0)
        self.cost[self.lethal] = 1.0

    @classmethod
    def from_layout(cls):
        w, h = int(round((X1 - X0) / RES)), int(round((Y1 - Y0) / RES))
        occ = np.zeros((h, w), bool)
        for _, corners in part_boxes() + model_boxes():
            if corners[:, 2].max() < 0.02 or corners[:, 2].min() > ROBOT_HEIGHT:
                continue                              # below the floor surface or above the robot
            i0, j0 = cls.to_cell_static(corners[:, 0].min(), corners[:, 1].min())
            i1, j1 = cls.to_cell_static(corners[:, 0].max(), corners[:, 1].max())
            occ[max(j0, 0):min(j1, h - 1) + 1, max(i0, 0):min(i1, w - 1) + 1] = True
        return cls(occ)

    @staticmethod
    def to_cell_static(x, y):
        return int(math.floor((x - X0) / RES)), int(math.floor((y - Y0) / RES))

    def to_cell(self, x, y):
        """World (m) -> cell (column i, row j)."""
        return self.to_cell_static(x, y)

    def to_world(self, i, j):
        """Cell -> world coordinates of its centre."""
        return X0 + (i + 0.5) * RES, Y0 + (j + 0.5) * RES

    def is_free(self, x, y):
        i, j = self.to_cell(x, y)
        return 0 <= i < self.w and 0 <= j < self.h and not self.lethal[j, i]

    def image(self, scale=3):
        """BGR picture of the map (north up): black = obstacle, grey = forbidden for the robot's
        centre, light shading = comfort cost, white = free."""
        img = np.full((self.h, self.w, 3), 255, np.uint8)
        shade = (255 - 60 * self.cost).astype(np.uint8)
        img[:] = shade[:, :, None]
        img[self.lethal] = (175, 175, 175)
        img[self.occupied] = (30, 30, 30)
        img = img[::-1]                               # row 0 = south -> put north at the top
        return cv2.resize(img, (self.w * scale, self.h * scale), interpolation=cv2.INTER_NEAREST)

    def world_to_image(self, x, y, scale=3):
        return int((x - X0) / RES * scale), int((self.h - (y - Y0) / RES) * scale)
