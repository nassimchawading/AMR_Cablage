r"""Measure CoppeliaSim library models before using them in the layout.

For each model: load it far away from the workshop (x = 60 m), print its bounding box
(size, and where its origin sits inside the box), list its scripts and custom data,
render a thumbnail into media\library\, then delete it. The open scene is NOT saved.

Run (PowerShell, from the project folder, CoppeliaSim open):
  .\.venv\Scripts\python.exe src\inspect_models.py [filter ...]   (keeps models whose path contains any filter)
"""

import json
import sys
from pathlib import Path

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from build_scene import look_at_matrix
from snapshot import save_png

OUT = Path(__file__).resolve().parent.parent / "media" / "library"
FAR = (60.0, 0.0, 0.0)

CANDIDATES = [
    "infrastructure/floors/5mX5m concrete floor.ttm",
    "infrastructure/walls/240cm high walls/wall section 400cm.ttm",
    "infrastructure/walls/240cm high walls/wall section 200cm.ttm",
    "infrastructure/walls/240cm high walls/wall section 100cm.ttm",
    "infrastructure/walls/240cm high walls/L wall.ttm",
    "infrastructure/walls/240cm high walls/pillar 50cm.ttm",
    "infrastructure/windows/140cm window.ttm",
    "infrastructure/doors/automatic double sliding door.ttm",
    "infrastructure/doors/automatic sliding door.ttm",
    "furniture/shelves-cupboards-racks/rack.ttm",
    "furniture/shelves-cupboards-racks/deep cupboard.ttm",
    "furniture/tables/customizable table.ttm",
    "furniture/tables/high table.ttm",
    "equipment/conveyors/generic conveyor (belt).ttm",
    "equipment/conveyors/generic conveyor (rollers).ttm",
    "equipment/panes/pane (grid) 2.0 x 2.0.ttm",
    "equipment/panes/pane (grid) 1.0 x 2.0.ttm",
    "equipment/generic tray.ttm",
    "household/largeBasket.ttm",
    "office items/laptop.ttm",
    "other/aruco marker.ttm",
    "people/Standing Bill.ttm",
    "people/Working Bill.ttm",
    "people/Walking Bill.ttm",
    "people/Bill on path.ttm",
    "robots/non-mobile/Dobot Magician.ttm",
    "robots/non-mobile/UR3.ttm",
    "robots/non-mobile/UR5.ttm",
    "robots/non-mobile/ABB IRB 360.ttm",
    "components/grippers/suction pad.ttm",
    "components/sensors/Blob detection camera.ttm",
]


def bbox(sim, h):
    keys = ["min_x", "max_x", "min_y", "max_y", "min_z", "max_z"]
    v = [sim.getObjectFloatParam(h, getattr(sim, "objfloatparam_modelbbox_" + k)) for k in keys]
    return dict(zip(keys, v))


def describe(sim, h):
    """Scripts and custom data attached anywhere in the model."""
    info = []
    for o in sim.getObjectsInTree(h, sim.handle_all, 0):
        if sim.getObjectType(o) == sim.sceneobject_script:
            txt = sim.getObjectStringParam(o, sim.scriptstringparam_text) or ""
            info.append(f"script {sim.getObjectAlias(o, 1)} ({len(txt)} chars)")
        tags = sim.readCustomDataTags(o) or []
        if tags:
            info.append(f"custom data on {sim.getObjectAlias(o, 1)}: {tags}")
    return info


def thumbnail(sim, h, path, bb):
    cx, cy = (bb["min_x"] + bb["max_x"]) / 2 + FAR[0], (bb["min_y"] + bb["max_y"]) / 2
    cz = (bb["min_z"] + bb["max_z"]) / 2
    r = max(bb["max_x"] - bb["min_x"], bb["max_y"] - bb["min_y"], bb["max_z"] - bb["min_z"], 0.3)
    eye = (cx + 1.2 * r, cy - 1.6 * r, cz + 1.0 * r)
    s = sim.createVisionSensor(1 | 2 | 4, [480, 360, 0, 0], [0.01, 100.0, 0.9, 0.1, 0, 0, 0.85, 0.85, 0.85, 0, 0])
    try:
        sim.setObjectMatrix(s, look_at_matrix(eye, (cx, cy, cz)), sim.handle_world)
        sim.handleVisionSensor(s)
        img, res = sim.getVisionSensorImg(s)
        save_png(path, img, res[0], res[1])
    finally:
        sim.removeObjects([s])


def main():
    filters = [a.lower() for a in sys.argv[1:]] or [""]
    sim = RemoteAPIClient().require("sim")
    models_dir = sim.getStringParam(sim.stringparam_application_path) + "/models/"
    OUT.mkdir(parents=True, exist_ok=True)
    report = {}
    for rel in CANDIDATES:
        if not any(f in rel.lower() for f in filters):
            continue
        h = sim.loadModel(models_dir + rel)
        try:
            sim.setObjectPosition(h, list(FAR), sim.handle_world)
            bb = bbox(sim, h)
            size = [round(bb["max_" + a] - bb["min_" + a], 3) for a in "xyz"]
            origin = [round(-bb["min_" + a], 3) for a in "xyz"]   # origin position inside the box, from its min corner
            name = Path(rel).stem
            thumbnail(sim, h, OUT / f"{name}.png", bb)
            try:
                extras = describe(sim, h)
            except Exception as e:
                extras = [f"(could not list scripts: {e})"]
            report[rel] = dict(alias=sim.getObjectAlias(h), size=size, origin_from_min_corner=origin, extras=extras)
            print(f"{rel}\n   size {size}  origin-from-min-corner {origin}")
            for line in report[rel]["extras"]:
                print("   ", line)
        finally:
            sim.removeModel(h)
    path = OUT / "models.json"          # merge with previous runs
    old = json.loads(path.read_text(encoding="utf-8")) if path.exists() else {}
    path.write_text(json.dumps({**old, **report}, indent=2), encoding="utf-8")
    print("Report:", OUT / "models.json")


if __name__ == "__main__":
    main()
