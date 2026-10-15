r"""Part 9 test: the quality-control cell sorts the harness totes by itself.

The conveyor brings the 3 totes (tags: green, red, green) under the QC camera; the Python cell
(src\qc_cell.py) decides OK / defect from the image and the UR5 puts each tote on the OK table or
the REBUTS table. Check (true positions from the simulator): every tote on the right table,
standing upright. Saves media\qc_camera.png (camera view with the detected tag).

Needs scenes\atelier_cablage.ttt open. Run (PowerShell, from the project folder):
  .\.venv\Scripts\python.exe tests\test_qc_cell.py
"""

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))
import cv2
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import layout
from qc_cell import QCCell, TABLES, find_tags

TIMEOUT = 150.0


def main():
    client = RemoteAPIClient()
    sim, simIK = client.require("sim"), client.require("simIK")
    totes = []
    for i, (_, ok) in enumerate(layout.QC_TOTES):
        totes.append((sim.getObject(f"/Atelier/bac_faisceau" + (f"_{i}" if i else "")), ok))
    sim.setStepping(True)
    sim.startSimulation()
    saved = False
    try:
        cell = QCCell(sim, simIK)
        while sim.getSimulationTime() < TIMEOUT and cell.stats["sorted"] < len(totes):
            cell.update()
            if not saved and cell.last_image is not None:
                tags = find_tags(cell.last_image)
                if tags and abs(tags[0][1] - 128) < 10:
                    img = cell.last_image.copy()
                    for ok, u, v, _ in tags:
                        cv2.circle(img, (int(u), int(v)), 14, (0, 255, 0) if ok else (0, 0, 255), 2)
                        cv2.putText(img, "OK" if ok else "DEFAUT", (int(u) - 20, int(v) - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    cv2.imwrite(str(ROOT / "media" / "qc_camera.png"), cv2.resize(img, (512, 512), interpolation=cv2.INTER_NEAREST))
                    saved = True
            sim.step()
        for _ in range(20):
            sim.step()
        good = 0
        for h, ok in totes:
            x, y, z = sim.getObjectPosition(h, sim.handle_world)
            m = sim.getObjectMatrix(h, sim.handle_world)
            upright = m[10] > 0.98
            on = abs(x - TABLES[ok]) < layout.TABLE_SIZE[0] / 2 and abs(y - layout.DROP_BOT_Y) < layout.TABLE_SIZE[1] / 2 \
                and abs(z - (layout.TABLE_H + layout.TOTE[2] + layout.FIN[2] / 2)) < 0.02
            good += on and upright
            print(f"{sim.getObjectAlias(h)} tag {'green' if ok else 'red  '} -> at ({x:.3f}, {y:.3f}, {z:.3f}) "
                  f"{'on the ' + ('OK' if ok else 'REBUTS') + ' table' if on else 'NOT on its table'}{'' if upright else ', TIPPED OVER'}")
        print(f"inspected {cell.stats['inspected']}, sorted {cell.stats['sorted']}, suction retries {cell.stats['failed']}, "
              f"time {sim.getSimulationTime():.1f} s")
    finally:
        sim.stopSimulation()
    print("RESULT:", "PASSED" if good == len(totes) else "FAILED")


if __name__ == "__main__":
    main()
