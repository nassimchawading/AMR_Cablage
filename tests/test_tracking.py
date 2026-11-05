r"""Plan B1-B3 test: does the tracker follow the walking workers correctly?

The robot stands still at a viewpoint and watches the workers walk for DURATION s of simulation.
Every step the navigator's perception gives the unexpected lidar points, the tracker turns them into
objects with a position and a velocity (src\tracker.py). They are compared with the TRUE positions
and velocities of the workers, read from the simulator (only possible in simulation: that is what
makes this a measurement):
  - detection rate: share of the steps where a worker the lidar really sees (in its field and hit by
    at least 2 rays, i.e. not hidden behind a machine) has a track near him,
  - position error and velocity error of that track,
  - moving tracks that match no worker (false "moving" objects).
Saves media\suivi_<scene>.png: map with the tracks, velocity arrows, predicted positions 1.5 s ahead
and the true workers. --live also shows it in a window (q to stop).

Works in both scenes (front tab): scenes\atelier_cablage.ttt or scenes\test_youbot.ttt. Run:
  .\.venv\Scripts\python.exe -u tests\test_tracking.py [--live]
"""

import math
import sys
from collections import deque
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))
import cv2
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from equip_youbot import LIDAR_POS, place_youbot
from navigator import Navigator
from occupancy import OccupancyGrid
from sensors import Sensors
from tracker import draw_tracks
from youbot import YouBot

DURATION = 60.0
MATCH = 0.7             # m: a track this close to a worker is "his" track
AHEAD = 1.5             # s: predicted position drawn on the map
SCALE = 3               # map pixels per 5 cm cell (60 px/m)


def scene_setup(sim):
    """Walkers, map, viewpoint and map window of the scene in the front tab."""
    scene = Path(sim.getStringParam(sim.stringparam_scene_path_and_name)).stem
    if scene == "atelier_cablage":
        import layout
        # in the aisle in front of Coupe, looking north: the lidar (240 deg) sees BOTH crossings, x = 5.75
        # (west) and x = 15 (east), ~4.6 m away, and operateur_arret walking in and standing (static)
        return (scene, "/Atelier", layout.WALKERS, OccupancyGrid.from_layout(), (10.4, 5.6, math.pi / 2),
                ((4.5, 16.5), (2.6, 9.6)))
    if scene == "test_youbot":
        import demo_layout as demo
        return (scene, "/Demo", demo.WALKERS, OccupancyGrid.from_layout(demo.PARTS, demo.MODELS), demo.TRACK_VIEW,
                ((2.0, 9.0), (2.5, 8.5)))
    raise SystemExit(f"STOP: unknown scene '{scene}' in the front tab.")


def in_lidar_field(pose, p):
    x, y, h = pose
    lx, ly = x + LIDAR_POS[0] * math.cos(h), y + LIDAR_POS[0] * math.sin(h)
    d = math.hypot(p[0] - lx, p[1] - ly)
    bearing = math.atan2(p[1] - ly, p[0] - lx) - h
    return 0.3 < d < 5.0 and abs(math.atan2(math.sin(bearing), math.cos(bearing))) < math.radians(118)


class MapView:
    def __init__(self, grid, window):
        (x0, x1), (y0, y1) = window
        full = grid.image(SCALE)
        u0, v0 = grid.world_to_image(x0, y1, SCALE)
        u1, v1 = grid.world_to_image(x1, y0, SCALE)
        self.base, self.off, self.grid = full[v0:v1, u0:u1].copy(), (u0, v0), grid

    def px(self, x, y):
        u, v = self.grid.world_to_image(x, y, SCALE)
        return u - self.off[0], v - self.off[1]

    def draw(self, pose, points, tracks, truth, t):
        img = self.base.copy()
        for p in points:
            cv2.circle(img, self.px(*p), 2, (60, 60, 230), -1)
        for name, p in truth.items():                    # true worker position (simulator)
            u, v = self.px(*p)
            cv2.drawMarker(img, (u, v), (200, 120, 0), cv2.MARKER_CROSS, 14, 2)
        draw_tracks(img, tracks, self.px, ahead=AHEAD, px_per_m=SCALE * 20)
        x, y, h = pose
        c = self.px(x, y)
        cv2.circle(img, c, int(0.3 * SCALE * 20), (40, 40, 40), 2)
        cv2.line(img, c, self.px(x + 0.4 * math.cos(h), y + 0.4 * math.sin(h)), (40, 40, 40), 2)
        cv2.putText(img, f"t = {t:5.1f} s   croix bleue = operateur (verite simulateur)", (8, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (60, 60, 60), 1, cv2.LINE_AA)
        cv2.putText(img, f"orange = suivi en mouvement, fleche = position prevue a {AHEAD} s, gris = fixe",
                    (8, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (60, 60, 60), 1, cv2.LINE_AA)
        return img


def main():
    sim = RemoteAPIClient().require("sim")
    if sim.getSimulationState() != sim.simulation_stopped:
        print("STOP: stop the simulation first (square Stop button), then run again.")
        return 1
    scene, root, walker_defs, grid, view_pose, window = scene_setup(sim)
    walkers = {w.name: sim.getObject(f"{root}/operateurs_mobiles/{w.name}/Bill") for w in walker_defs}
    robot, sensors = YouBot(sim), Sensors(sim)
    live = "--live" in sys.argv
    sim.setStepping(True)
    sim.startSimulation()
    view = MapView(grid, window)
    hist = {n: deque(maxlen=11) for n in walkers}           # last 0.5 s of true positions
    stats = {n: dict(visible=0, tracked=0, pos_err=[], vel_err=[]) for n in walkers}
    false_moving, steps = 0, 0
    best = (-1, None)
    try:
        place_youbot(sim, robot.model, *view_pose)
        for _ in range(5):
            sim.step()
        nav = Navigator(sim, robot, sensors, grid)
        t0 = sim.getSimulationTime()
        while sim.getSimulationTime() - t0 < DURATION:
            now = sim.getSimulationTime()
            pose = robot.pose()
            nav.perceive(pose, now)
            tracks = nav.tracker.confirmed()
            truth = {}
            for n, h in walkers.items():
                p = sim.getObjectPosition(h, sim.handle_world)[:2]
                hist[n].append(p)
                truth[n] = p
            steps += 1
            tpose = robot.true_pose()
            seen = nav.lidar_new
            for n, p in truth.items():
                # "visible" = in the lidar field AND really hit by the lidar (not hidden behind a machine)
                hit = np.sum(np.hypot(seen[:, 0] - p[0], seen[:, 1] - p[1]) < 0.5) if len(seen) else 0
                if not in_lidar_field(tpose, p) or hit < 2 or len(hist[n]) < hist[n].maxlen:
                    continue
                s = stats[n]
                s["visible"] += 1
                near = min(tracks, key=lambda t: math.dist(t.pos, p), default=None)
                if near is None or math.dist(near.pos, p) > MATCH:
                    continue
                s["tracked"] += 1
                s["pos_err"].append(math.dist(near.pos, p))
                v_true = (np.array(p) - np.array(hist[n][0])) / ((len(hist[n]) - 1) * 0.05)
                s["vel_err"].append(float(np.hypot(*(near.vel - v_true))))
            for t in tracks:
                if t.status == "moving" and all(math.dist(t.pos, p) > MATCH for p in truth.values()):
                    false_moving += 1
            moving = sum(t.status == "moving" for t in tracks)
            if moving > best[0] or live:
                img = view.draw(pose, nav.memory[-1][1] if nav.memory else [], tracks, truth, now - t0)
                if moving > best[0]:
                    best = (moving, img)
                if live:
                    cv2.imshow("suivi des operateurs", img)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
            sim.step()
    except KeyboardInterrupt:
        print("Stopped by the user.")
    finally:
        sim.stopSimulation()
        cv2.destroyAllWindows()
    if best[1] is not None:
        out = ROOT / "media" / f"suivi_{scene}.png"
        cv2.imwrite(str(out), best[1])
        print("Saved", out)
    print(f"\n{scene}: {steps} steps ({steps * 0.05:.0f} s simulated), robot standing at {view_pose}")
    ok = True
    for n, s in stats.items():
        if not s["visible"]:
            print(f"  {n:20s} never in the lidar field")
            continue
        rate = s["tracked"] / s["visible"]
        pe = np.mean(s["pos_err"]) if s["pos_err"] else float("nan")
        ve = np.median(s["vel_err"]) if s["vel_err"] else float("nan")
        print(f"  {n:20s} in the lidar field {s['visible'] * 0.05:5.1f} s, tracked {rate * 100:5.1f} %, "
              f"position error mean {pe * 100:4.1f} cm, velocity error median {ve:.2f} m/s")
        ok &= rate > 0.8 and pe < 0.25 and ve < 0.3
    print(f"  moving tracks matching no worker: {false_moving} track-steps ({false_moving * 0.05:.1f} s)")
    ok &= false_moving * 0.05 < 3.0
    print("RESULT:", "PASSED" if ok else "FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
