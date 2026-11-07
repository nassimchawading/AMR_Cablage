r"""Live windows while the robot works (shared by src\demo_youbot.py and the tests' --live option).

    view = LiveView(sim)                # wraps `sim` - or a Factory (QC cell keeps running) - like
                                        # src\factory.py does: use `view` everywhere instead of `sim`
    robot, sensors = YouBot(view), Sensors(view)
    nav = Navigator(view, robot, sensors, grid, log=view.log)
    view.setup(robot, sensors, grid, nav=nav, walkers={name: handle})
    ...                                 # every view.step() = sim.step(); the windows are refreshed
                                        # every VIEW_EVERY steps (0.15 s simulated)

Windows (default):
  "camera"          robot camera + the ArUco markers it detects (id, distance)
  "lidar + sonars"  top view around the robot (forward = up): lidar field, sonar cones, lidar points,
                    tracked objects (orange = moving, arrow = predicted position 1.5 s ahead, grey = static)
  "carte"           map: planned A* route (blue), trail (green), remembered unexpected obstacles (red),
                    tracked objects with predictions, true worker positions (blue rings, simulator)
  "ordres"          (with setup(..., mgr=MissionManager)) what the robot does, battery bar, order queue
                    (with priority class), last orders, QC cell / operators counters (src\shift.py)
mosaic=True: the same tiles + a text panel (chapter, pose, events) in ONE 2 x 2 window, as in the youBot
demo; record=<path.mp4> also records it. Press q in a window: KeyboardInterrupt, so the program stops
cleanly through its usual finally: blocks.
autosave=<prefix>: once, the first time a MOVING tracked object is within AUTOSAVE_DIST of the robot,
every window is saved as media\<prefix>_<window>.png (pictures for the report).
"""

import math
from collections import deque
from pathlib import Path

import cv2
import numpy as np

from markers import MarkerDetector
from sensors import draw_scan
from tracker import draw_tracks

ROOT = Path(__file__).resolve().parent.parent
VIEW_EVERY = 3              # refresh the windows every 3 simulation steps (0.15 s simulated)
TILE_W, TILE_H = 480, 360   # mosaic tiles
SCAN_SIZE, SCAN_SCALE = 520, 70   # separate "lidar + sonars" window: 520 px, 70 px/m (3.7 m around)
AHEAD = 1.5                 # s: predicted positions drawn
AUTOSAVE_DIST = 3.0         # m


def legend(img, cols=1, top=None):
    """Small legend (drawn in place, returned): bottom-left corner of a map image, or from row `top`
    (e.g. the free space under the map in the mosaic tile); cols = number of columns."""
    items = [((220, 120, 0), "route A* prevue", "line"), ((0, 160, 0), "trajet parcouru", "line"),
             ((30, 30, 230), "obstacle imprevu vu", "dot"), ((0, 140, 255), "operateur suivi + prediction", "dot"),
             ((110, 110, 110), "objet suivi fixe", "dot"), ((200, 160, 160), "operateur (position vraie)", "ring"),
             ((40, 40, 40), "robot", "ring")]
    h = img.shape[0]
    rows = -(-len(items) // cols)
    x0, y0 = 6, (h - 8 - 15 * rows) if top is None else top + 14
    overlay = img.copy()
    cv2.rectangle(overlay, (x0 - 4, y0 - 12), (x0 + 215 * cols - 10, y0 + 15 * rows - 8), (255, 255, 255), -1)
    cv2.addWeighted(overlay, 0.8, img, 0.2, 0, img)
    for i, (col, text, kind) in enumerate(items):
        x0 = 6 + 215 * (i // rows)
        y = y0 + 15 * (i % rows)
        if kind == "line":
            cv2.line(img, (x0, y - 4), (x0 + 18, y - 4), col, 2)
        elif kind == "dot":
            cv2.circle(img, (x0 + 9, y - 4), 4, col, -1)
        else:
            cv2.circle(img, (x0 + 9, y - 4), 5, col, 1)
        cv2.putText(img, text, (x0 + 26, y), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (40, 40, 40), 1, cv2.LINE_AA)
    return img


class LiveView:
    def __init__(self, sim, title="AMR", mosaic=False, record=None, show_map=True, map_window=None,
                 map_scale=2, autosave=None):
        self._sim = sim
        self.title, self.mosaic, self.record, self.show_map = title, mosaic, record, show_map
        self.map_window, self.map_scale = map_window, map_scale      # map_window ((x0, x1), (y0, y1)) or None
        self.autosave = autosave
        self.n = 0
        self.chapter = ""
        self.events = deque(maxlen=9)
        self.nav = None
        self.robot = None
        self.mgr = None
        self.writer = None
        self.last = {}

    def __getattr__(self, name):            # everything else: the wrapped sim / Factory
        return getattr(self._sim, name)

    def log(self, msg):
        print(msg)
        self.events.append(f"{self._sim.getSimulationTime():6.1f}s {msg.strip()}")

    def setup(self, robot, sensors, grid, nav=None, walkers=None, mgr=None):
        """Call once the simulation runs (the camera / sensors answer)."""
        self.robot, self.sensors, self.grid, self.nav, self.mgr = robot, sensors, grid, nav, mgr
        self.walkers = walkers or {}
        self.markers = MarkerDetector(self._sim)
        s = self.map_scale
        full = grid.image(s)
        if self.map_window:
            (x0, x1), (y0, y1) = self.map_window
            u0, v0 = grid.world_to_image(x0, y1, s)
            u1, v1 = grid.world_to_image(x1, y0, s)
        else:
            u0, v0, u1, v1 = 0, 0, full.shape[1], full.shape[0]
        self.crop = (u0, v0)
        self.map_base = full[v0:v1, u0:u1].copy()

    def step(self):
        self._sim.step()
        self.n += 1
        if self.robot is not None and self.n % VIEW_EVERY == 0:
            self.refresh()

    # ------------------------------------------------------------ tiles
    def _p(self, x, y):
        u, v = self.grid.world_to_image(x, y, self.map_scale)
        return u - self.crop[0], v - self.crop[1]

    def _tracks(self):
        return self.nav.tracker.confirmed() if self.nav is not None else []

    def camera(self):
        img = self.sensors.camera_image().copy()
        MarkerDetector.draw(img, self.markers.detect(img))
        return img

    def scan(self, size=SCAN_SIZE, scale=SCAN_SCALE):
        img = draw_scan(self.sensors.lidar_points(), self.sensors.sonar_distances(), size=size, scale=scale,
                        geometry=self.sensors.geometry)
        x, y, h = self.robot.pose()
        c, s, mid = math.cos(h), math.sin(h), size // 2

        def to_px(wx, wy):                  # world -> robot frame (forward, left) -> pixel (forward = up)
            f, l = (wx - x) * c + (wy - y) * s, -(wx - x) * s + (wy - y) * c
            return int(round(mid - l * scale)), int(round(mid - f * scale))

        draw_tracks(img, self._tracks(), to_px, ahead=AHEAD, px_per_m=scale)
        return img

    def map(self):
        img = self.map_base.copy()
        nav = self.nav
        if nav is not None:
            if getattr(nav, "route", None) is not None:
                cv2.polylines(img, [np.array([self._p(*p) for p in nav.route.pts], np.int32)], False, (220, 120, 0), 2)
            if len(nav.trail) > 1:
                cv2.polylines(img, [np.array([self._p(*p) for p in nav.trail[-3000:]], np.int32)], False, (0, 160, 0), 1)
            for _, pts in nav.memory:                       # unexpected obstacles the robot remembers
                for p in pts:
                    cv2.circle(img, self._p(*p), 2, (30, 30, 230), -1)
            draw_tracks(img, self._tracks(), self._p, ahead=AHEAD, px_per_m=self.map_scale * 20)
        for h in self.walkers.values():                     # true worker positions (for the viewer only)
            wx, wy, _ = self._sim.getObjectPosition(h, self._sim.handle_world)
            cv2.circle(img, self._p(wx, wy), 8, (200, 160, 160), 1)
        x, y, h = self.robot.true_pose()
        c = self._p(x, y)
        cv2.circle(img, c, 9, (40, 40, 40), 2)
        cv2.line(img, c, self._p(x + 0.35 * math.cos(h), y + 0.35 * math.sin(h)), (40, 40, 40), 2)
        return img

    def panel(self, w=TILE_W, h=TILE_H):
        out = np.full((h, w, 3), 30, np.uint8)
        x, y, hd = self.robot.pose()
        lines = [(self.chapter, 0.65, (0, 220, 255)),
                 (f"temps simule {self._sim.getSimulationTime():6.1f} s", 0.5, (230, 230, 230)),
                 (f"pose (UWB + gyro) x {x:5.2f}  y {y:5.2f}  cap {math.degrees(hd):+6.1f} deg", 0.5, (230, 230, 230)),
                 ("", 0.5, (0, 0, 0))]
        lines += [(e[:62], 0.42, (170, 230, 170)) for e in self.events]
        yy = 30
        for text, size, col in lines:
            cv2.putText(out, text, (10, yy), cv2.FONT_HERSHEY_SIMPLEX, size, col, 1, cv2.LINE_AA)
            yy += 30 if size > 0.45 else 24
        return out

    def orders(self, w=560, h=600):
        """Orders / battery panel (mission manager). Full size: separate "ordres" window; small (h < 500):
        the 4th tile of the mosaic, with fewer lines."""
        mgr, sim = self.mgr, self._sim
        out = np.full((h, w, 3), 30, np.uint8)
        font = cv2.FONT_HERSHEY_SIMPLEX
        small = h < 500
        lh, fs = (19, 0.42) if small else (24, 0.45)

        def put(text, y, size=fs, col=(230, 230, 230), x=10, bold=1):
            cv2.putText(out, text, (x, y), font, size, col, bold, cv2.LINE_AA)

        t = sim.getSimulationTime()
        put(f"POSTE  {int(t // 60):02d}:{int(t % 60):02d}  (temps simule)", 22 if small else 28,
            0.52 if small else 0.6, (0, 220, 255), bold=2)
        cur = mgr.current
        act = {"IDLE": "en attente d'un ordre", "GO_CHARGE": "va a la borne de recharge",
               "CHARGE": "en charge"}.get(mgr.activity, mgr.activity)
        y = 50 if small else 64
        put(act if cur is None else f"#{cur.id} {cur.what}", y, 0.62 if small else 0.75, (255, 255, 255), bold=2)
        if cur is not None:
            p = f"{cur.pickup}[{cur.pickup_slot}]" if cur.pickup_slot is not None else cur.pickup
            d = f"{cur.drop}[{cur.drop_slot}]" if cur.drop_slot is not None else cur.drop
            put(f"{p} -> {d}   {cur.state}" + ("   URGENT" if cur.urgent else ""), y + (22 if small else 28),
                0.48 if small else 0.55, (200, 220, 255))
        bat = getattr(sim, "battery", None)
        y += 34 if small else 46
        if bat is not None:                                   # battery bar
            lvl, bw = bat.level, w - (180 if small else 160)
            col = (60, 200, 60) if lvl >= 50 else (0, 170, 255) if lvl >= 25 else (40, 40, 230)
            cv2.rectangle(out, (10, y), (10 + bw, y + 18), (90, 90, 90), 1)
            cv2.rectangle(out, (11, y + 1), (11 + int((bw - 2) * lvl / 100), y + 17), col, -1)
            put(f"batterie {lvl:5.1f} %" + (" (charge)" if bat.charging else ""), y + 15, fs, (255, 255, 255), x=bw + 18)
        y += 44 if small else 50
        queue = sorted(mgr.waiting, key=lambda o: (mgr.priority(o), o.created, o.id))
        put(f"file d'attente ({len(queue)})", y, fs + 0.05, (0, 220, 255))
        for o in queue[:3 if small else 7]:
            y += lh
            ready = "" if mgr.ready(o) else " (pas pret)"
            put(f"[{mgr.priority(o)}] #{o.id} {o.what}: {o.pickup} -> {o.drop} {t - o.created:4.0f} s{ready}", y)
        y = max(y + lh + 6, 220 if small else 360)
        put("derniers ordres", y, fs + 0.05, (0, 220, 255))
        cols = {"DONE": (120, 230, 120), "REFUSED": (0, 170, 255), "FAILED": (60, 60, 240), "CANCELLED": (160, 160, 160)}
        for o in mgr.finished[-(3 if small else 6):][::-1]:
            y += lh
            put(f"#{o.id} {o.state:8s} {o.what}: {o.pickup_slot} -> {o.drop_slot} {o.done - o.started:4.0f} s",
                y, fs, cols.get(o.state, (230, 230, 230)))
        qc, flow = getattr(sim, "qc", None), getattr(sim, "flow", None)
        if qc is not None and flow is not None:
            put(f"cellule CQ: {qc.stats['inspected']} inspectes ({qc.stats['ok']} OK / {qc.stats['defect']} defaut), "
                f"{qc.stats['sorted']} tries", h - (28 if small else 40), fs - 0.03, (200, 200, 200))
            put(f"operateurs: {flow.stats['spawned']} bacs poses, {flow.stats['taken']} retires, "
                f"{flow.stats['restocked']} reappros", h - (9 if small else 16), fs - 0.03, (200, 200, 200))
        return out

    @staticmethod
    def _fit(img, w, h):
        """img scaled to width w, top of an h-high white tile (cropped if taller)."""
        img = cv2.resize(img, (w, int(img.shape[0] * w / img.shape[1])))
        out = np.full((h, w, 3), 255, np.uint8)
        out[:min(h, img.shape[0])] = img[:h]
        return out

    # ------------------------------------------------------------ windows
    def refresh(self):
        if self.mosaic:
            scan = self.scan(size=TILE_H, scale=60)
            tile = np.full((TILE_H, TILE_W, 3), 255, np.uint8)
            tile[:, (TILE_W - TILE_H) // 2:(TILE_W + TILE_H) // 2] = scan
            full = self.map()
            m = self._fit(full, TILE_W, TILE_H)
            legend(m, cols=2, top=min(TILE_H - 66, int(full.shape[0] * TILE_W / full.shape[1])))
            info = self.orders(TILE_W, TILE_H) if self.mgr is not None else self.panel()
            frame = np.vstack((np.hstack((cv2.resize(self.camera(), (TILE_W, TILE_H)), tile)),
                               np.hstack((m, info))))
            self.last = {"mosaique": frame}
            cv2.imshow(self.title, frame)
            if self.record:
                if self.writer is None:
                    self.writer = cv2.VideoWriter(str(self.record), cv2.VideoWriter_fourcc(*"mp4v"),
                                                  1.0 / (VIEW_EVERY * 0.05), (frame.shape[1], frame.shape[0]))
                self.writer.write(frame)
        else:
            self.last = {"camera": self.camera(), "lidar_sonars": self.scan()}
            if self.show_map:
                self.last["carte"] = legend(self.map())
            if self.mgr is not None:
                self.last["ordres"] = self.orders()
            names = {"camera": "camera", "lidar_sonars": "lidar + sonars", "carte": "carte", "ordres": "ordres"}
            for key, img in self.last.items():
                cv2.imshow(names[key], img)
        self._maybe_autosave()
        if cv2.waitKey(1) & 0xFF == ord("q"):
            raise KeyboardInterrupt

    def _maybe_autosave(self):
        if not self.autosave:
            return
        x, y, _ = self.robot.pose()
        if any(t.status == "moving" and math.dist(t.pos, (x, y)) < AUTOSAVE_DIST for t in self._tracks()):
            self.save(self.autosave)
            self.autosave = None

    def save(self, prefix):
        r"""Save the windows as they are now: media\<prefix>_<window>.png."""
        for key, img in self.last.items():
            path = ROOT / "media" / f"{prefix}_{key}.png"
            cv2.imwrite(str(path), img)
            print("Saved", path)

    def close(self):
        if self.writer is not None:
            self.writer.release()
        cv2.destroyAllWindows()
