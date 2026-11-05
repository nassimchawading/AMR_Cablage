r"""Part 9 test: the quality-control cell sorts the harness totes by itself.

The conveyor brings the 3 totes (tags: green, red, green) under the QC camera; the Python cell
(src\qc_cell.py) decides OK / defect from the image and the UR5 puts each tote on the OK table or
the REBUTS table (one spot per table: the second OK tote must WAIT at the end stop, nobody takes
the first one away here). Check (true positions from the simulator, qc_cell.check_totes): each
tote where it should be, standing upright, no cell fault. Saves media\qc_camera.png.

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
from qc_cell import QCCell, check_totes, find_tags
from tote_flow import restore_scene

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
        restore_scene(sim)             # totes as built (a tote-stream run repaints / parks some)
        cell = QCCell(sim, simIK)
        n_ok = sum(ok for _, ok in totes)
        expected = min(n_ok, len(layout.QC_SLOTS_DY)) + min(len(totes) - n_ok, len(layout.QC_SLOTS_DY))
        while sim.getSimulationTime() < TIMEOUT and not cell.fault and not (
                cell.stats["sorted"] >= expected and (expected == len(totes) or cell.full_reported)):
            cell.tick()
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
        good, total, lines = check_totes(sim)
        print("\n".join(lines))
        print(f"inspected {cell.stats['inspected']}, sorted {cell.stats['sorted']}, suction retries {cell.stats['failed']}, "
              f"fault {cell.fault}, time {sim.getSimulationTime():.1f} s")
    finally:
        sim.stopSimulation()
    print("RESULT:", "PASSED" if good == total and not cell.fault else "FAILED")


if __name__ == "__main__":
    main()
