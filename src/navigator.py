r"""Navigation: follow an A* route with the youBot while avoiding unexpected obstacles
(walking workers), using the lidar AND the sonar ring, with a short obstacle memory.

    nav = Navigator(sim, robot, sensors, grid)
    result = nav.go_to((x, y, heading))      # drives there, returns statistics

Perception (every step): unexpected obstacles = sensor detections NOT explained by the fixed map:
  - lidar points (front, 240 deg, 5 m),
  - sonar detections (all around, ~1 m): the lidar cannot see behind / beside the robot,
  - remembered for MEMORY seconds, so that a person leaving the field of view is not forgotten.

State machine (one decision every simulation step, 50 ms):
  FOLLOW   follow the route: drive towards a "carrot" point LOOKAHEAD m ahead on the route, facing
           the direction of travel (the lidar looks where the robot goes). Obstacle in the corridor
           ahead: slow down from SLOW_DIST, stop at STOP_DIST. Anybody within NEAR_PERSON: speed
           limited to NEAR_SPEED. Anything closer than TOO_CLOSE (any direction): stop.
  WAIT     stopped. Way clear -> FOLLOW. Somebody keeps coming closer (workers do not stop for the
           robot) -> ESCAPE. Still blocked after WAIT_REPLAN s -> re-plan with A* around the obstacle.
  ESCAPE   move away from the approaching person (mecanum: in any direction, without turning),
           choosing a direction that is free on the map; then wait until nobody is within
           CLEAR_DIST for CLEAR_TIME, and re-plan from there.
  FINAL    last centimetres: precise approach of the target pose (position + heading).
Emergency stop while moving: a sonar looking in the direction of motion reads < SONAR_STOP.
"""

import math
import time
from collections import deque

import cv2
import numpy as np

from equip_youbot import SONARS
from occupancy import RES, X0, Y0, OccupancyGrid
from planner import Planner, path_length

V_MAX = 0.45            # m/s cruise speed
LOOKAHEAD = 0.6         # m, carrot distance on the route
CORRIDOR = 0.50         # m, half width of the corridor checked ahead (robot half width 0.21 + 0.29 margin;
                        # 0.36 let a worker pass 15 cm from the robot's side in the first test)
CHECK_AHEAD = 2.2       # m of route checked for obstacles
SLOW_DIST, STOP_DIST = 1.8, 0.75   # m from the robot centre, along the route
NEAR_PERSON, NEAR_SPEED = 1.2, 0.2   # unexpected obstacle within 1.2 m (anywhere) -> max 0.2 m/s
TOO_CLOSE = 0.55        # m, any direction: stop
WAIT_REPLAN = 3.0       # s blocked before re-planning
CLOSING = 0.10          # m: nearest obstacle got this much closer in 0.5 s while we are stopped -> escape
ESCAPE_SPEED, ESCAPE_DONE, ESCAPE_MAX_T = 0.35, 1.0, 8.0
CLEAR_DIST, CLEAR_TIME = 1.2, 1.0
MEMORY = 1.5            # s an obstacle point is remembered
SONAR_STOP = 0.22       # m
PERSON_R = 0.35         # m, radius added to the map around obstacle points when re-planning
FINAL_DIST = 0.35       # m from the goal: switch to precise approach


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Route:
    """Polyline with arc-length helpers."""

    def __init__(self, pts):
        self.pts = np.asarray(pts, float)
        seg = np.diff(self.pts, axis=0)
        self.seg_len = np.hypot(seg[:, 0], seg[:, 1])
        self.cum = np.concatenate(([0.0], np.cumsum(self.seg_len)))
        self.length = float(self.cum[-1])

    def project(self, p, s_min=0.0):
        """Arc length s of the closest route point to p (not before s_min), and the distance."""
        best = (float("inf"), 0.0)
        for k in range(len(self.seg_len)):
            if self.cum[k + 1] < s_min:
                continue
            a, d = self.pts[k], self.pts[k + 1] - self.pts[k]
            t = 0.0 if self.seg_len[k] == 0 else float(np.clip(np.dot(p - a, d) / self.seg_len[k] ** 2, 0, 1))
            s = max(self.cum[k] + t * self.seg_len[k], s_min)
            dist = float(np.hypot(*(p - self.point(s))))
            if dist < best[0]:
                best = (dist, s)
        return best[1], best[0]

    def point(self, s):
        s = min(max(s, 0.0), self.length)
        k = min(int(np.searchsorted(self.cum, s, side="right") - 1), len(self.seg_len) - 1)
        t = 0.0 if self.seg_len[k] == 0 else (s - self.cum[k]) / self.seg_len[k]
        return self.pts[k] + t * (self.pts[k + 1] - self.pts[k])

    def tangent(self, s):
        k = int(np.searchsorted(self.cum, min(s, self.length - 1e-6), side="right") - 1)
        k = min(max(k, 0), len(self.seg_len) - 1)
        d = self.pts[k + 1] - self.pts[k]
        return math.atan2(d[1], d[0])


class Navigator:
    def __init__(self, sim, robot, sensors, grid, draw=True, walkers=None, log=print):
        self.sim, self.robot, self.sensors, self.grid = sim, robot, sensors, grid
        self.planner = Planner(grid)
        self.log = log
        r = int(round(0.15 / RES))                  # detections within 15 cm of a mapped object = "known"
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
        self.known = cv2.dilate(grid.occupied.astype(np.uint8), k).astype(bool)
        self.memory = deque()                       # (time, N x 2 world points)
        self.near_hist = deque()                    # (time, distance to the nearest obstacle)
        self.moving = False
        self.walkers = walkers or {}                # name -> handle (statistics only: true positions)
        self.drawing = sim.addDrawingObject(sim.drawing_lines, 5, 0, -1, 2000, [0.1, 0.4, 1.0]) if draw else None
        self.trail = []
        self.walker_trails = {n: [] for n in self.walkers}
        self.robot_coll = self.walker_coll = None
        if self.walkers:
            self.robot_coll = sim.createCollection(0)
            sim.addItemToCollection(self.robot_coll, sim.handle_tree, robot.model, 0)
            self.walker_coll = sim.createCollection(0)
            for h in self.walkers.values():
                sim.addItemToCollection(self.walker_coll, sim.handle_tree, h, 0)

    # ------------------------------------------------------------ perception
    def _unknown(self, wx, wy):
        i = np.clip(((wx - X0) / RES).astype(int), 0, self.grid.w - 1)
        j = np.clip(((wy - Y0) / RES).astype(int), 0, self.grid.h - 1)
        return ~self.known[j, i]

    def perceive(self, pose, now):
        """Unexpected obstacles (world points): lidar + sonars not explained by the map, + memory."""
        x, y, h = pose
        c, s = math.cos(h), math.sin(h)
        fl = [self.sensors.lidar_points()]
        sonar = self.sensors.sonar_distances()
        for name, f, l, _, deg in SONARS:
            d = sonar.get(name)
            if d is not None:
                a = math.radians(deg)
                fl.append(np.array([[f + d * math.cos(a), l + d * math.sin(a)]]))
        pts = np.vstack([p for p in fl if len(p)]) if any(len(p) for p in fl) else np.zeros((0, 2))
        if len(pts):
            wx, wy = x + pts[:, 0] * c - pts[:, 1] * s, y + pts[:, 0] * s + pts[:, 1] * c
            keep = self._unknown(wx, wy)
            cur = np.column_stack((wx[keep], wy[keep]))
            cur = np.unique(np.round(cur / 0.1) * 0.1, axis=0) if len(cur) else cur   # 10 cm cells
        else:
            cur = np.zeros((0, 2))
        self.memory.append((now, cur))
        while self.memory and now - self.memory[0][0] > MEMORY:
            self.memory.popleft()
        allp = [p for _, p in self.memory if len(p)]
        obs = np.unique(np.vstack(allp), axis=0) if allp else np.zeros((0, 2))
        near = float(np.min(np.hypot(obs[:, 0] - x, obs[:, 1] - y))) if len(obs) else 9.0
        self.near_hist.append((now, near))
        while self.near_hist and now - self.near_hist[0][0] > 0.6:
            self.near_hist.popleft()
        return obs, near

    def closing(self, near):
        """True if the nearest obstacle came CLOSING metres closer within the last 0.5 s."""
        return bool(self.near_hist) and self.near_hist[0][1] - near > CLOSING and near < NEAR_PERSON

    def blocking_distance(self, route, s_robot, obstacles, pos):
        """Distance along the route from the robot to the first obstacle in the corridor ahead."""
        best = float("inf")
        for p in obstacles:
            if math.dist(p, pos) > CHECK_AHEAD + 0.6:
                continue
            s, lat = route.project(p, s_robot)
            if lat < CORRIDOR and 0 <= s - s_robot < CHECK_AHEAD:
                best = min(best, max(s - s_robot, math.dist(p, pos)))
        return best

    def sonar_blocked(self, move_dir_robot):
        for name, d in self.sensors.sonar_distances().items():
            if d is not None and d < SONAR_STOP:
                deg = next(s[4] for s in SONARS if s[0] == name)
                if abs(wrap(math.radians(deg) - move_dir_robot)) < math.radians(60):
                    return name
        return None

    # ------------------------------------------------------------ planning / motion
    def plan(self, start, goal, obstacles=()):
        planner = self.planner
        if len(obstacles):
            occ = self.grid.occupied.astype(np.uint8)
            r = int(round(PERSON_R / RES))
            for p in obstacles:
                cv2.circle(occ, self.grid.to_cell(*p), r, 1, -1)
            planner = Planner(OccupancyGrid(occ.astype(bool)))
        pts = planner.plan(start, goal)
        if pts and self.drawing is not None:
            self.sim.addDrawingObjectItem(self.drawing, None)        # erase the previous route
            for p, q in zip(pts, pts[1:]):
                self.sim.addDrawingObjectItem(self.drawing, [p[0], p[1], 0.03, q[0], q[1], 0.03])
        return Route(pts) if pts else None

    def drive_world(self, vx, vy, turn, heading):
        """Command a velocity given in the WORLD frame (+ rotation)."""
        c, s = math.cos(heading), math.sin(heading)
        self.robot.move(vx * c + vy * s, -vx * s + vy * c, turn)
        self.moving = abs(vx) + abs(vy) + abs(turn) > 1e-3

    def stop(self):
        self.robot.stop()
        self.moving = False

    def escape_direction(self, pos, obstacles, near):
        """Direction (unit world vector) away from close obstacles that stays free on the map."""
        close = [p for p in obstacles if math.dist(p, pos) < 1.5]
        if not close:
            return None
        away = np.sum([(pos - p) / max(math.dist(p, pos), 0.1) ** 2 for p in close], axis=0)
        base = math.atan2(away[1], away[0])
        for off in (0, 30, -30, 60, -60, 90, -90, 120, -120):
            a = base + math.radians(off)
            d = np.array([math.cos(a), math.sin(a)])
            if not all(self.grid.is_free(*(pos + d * k * 0.15)) for k in range(1, 5)):
                continue
            target = pos + d * 0.6
            if min(math.dist(target, p) for p in close) > near:
                return d
        return None

    def record(self, stats):
        x, y, _ = self.robot.true_pose()
        self.trail.append((x, y))
        for n, h in self.walkers.items():
            p = self.sim.getObjectPosition(h, self.sim.handle_world)
            self.walker_trails[n].append((p[0], p[1]))
            stats["min_dist_worker"] = min(stats["min_dist_worker"], math.dist((x, y), p[:2]))
        if self.robot_coll is not None:
            hit = self.sim.checkCollision(self.robot_coll, self.walker_coll)
            if (hit[0] if isinstance(hit, (list, tuple)) else hit) > 0:
                stats["contacts_moving" if self.moving else "contacts_stopped"] += 1

    # ------------------------------------------------------------ main loop
    def go_to(self, goal, timeout=240.0):
        sim, robot = self.sim, self.robot
        t0, wall0 = sim.getSimulationTime(), time.perf_counter()
        stats = dict(goal=goal, reached=False, time=0.0, distance=0.0, stops=0, escapes=0, replans=0,
                     emergency_stops=0, min_dist_worker=float("inf"), contacts_moving=0, contacts_stopped=0,
                     events=[])
        trail_start = len(self.trail)

        def event(msg):
            stats["events"].append((round(sim.getSimulationTime() - t0, 1), msg))
            self.log(f"  [{sim.getSimulationTime() - t0:6.1f} s] {msg}")

        def to_wait(msg):
            nonlocal state, wait_t, last_try
            self.stop()
            self.near_hist.clear()      # "closing" must only measure people moving, not our own approach
            state, wait_t, last_try = "WAIT", now, now
            stats["stops"] += 1
            event(msg)

        pose = robot.pose()
        route = self.plan(pose[:2], goal[:2])
        if route is None:
            event("no route found")
            return stats
        state, s_robot = "FOLLOW", 0.0
        wait_t = last_try = clear_since = esc_t = 0.0
        limited = False
        while sim.getSimulationTime() - t0 < timeout:
            now = sim.getSimulationTime()
            pose = robot.pose()
            pos = np.array(pose[:2])
            obstacles, near = self.perceive(pose, now)
            s_robot, _ = route.project(pos, max(0.0, s_robot - 0.3))
            remaining = route.length - s_robot
            d_block = self.blocking_distance(route, s_robot, obstacles, pos)

            if state == "FOLLOW":
                if remaining < FINAL_DIST:
                    state = "FINAL"
                    event("final approach")
                elif d_block < STOP_DIST:
                    to_wait(f"obstacle {d_block:.2f} m ahead on the route -> stop")
                elif near < TOO_CLOSE:
                    to_wait(f"somebody {near:.2f} m from the robot -> stop")
                else:
                    carrot = route.point(s_robot + LOOKAHEAD)
                    d = carrot - pos
                    dist = float(np.hypot(*d)) or 1e-6
                    herr = wrap(route.tangent(s_robot + 0.2) - pose[2])
                    v = V_MAX * min(1.0, max(0.0, (d_block - STOP_DIST) / (SLOW_DIST - STOP_DIST)))
                    v = max(v, 0.08) * max(0.25, math.cos(herr)) * min(1.0, remaining / 0.8 + 0.3)
                    if near < NEAR_PERSON:
                        v = min(v, NEAR_SPEED)
                        if not limited:
                            event(f"somebody {near:.2f} m away -> speed limited to {NEAR_SPEED} m/s")
                    limited = near < NEAR_PERSON
                    son = self.sonar_blocked(wrap(math.atan2(d[1], d[0]) - pose[2]))
                    if son:
                        stats["emergency_stops"] += 1
                        to_wait(f"EMERGENCY STOP (sonar {son})")
                    else:
                        self.drive_world(v * d[0] / dist, v * d[1] / dist, max(-1.0, min(1.0, 1.8 * herr)), pose[2])

            elif state == "WAIT":
                self.stop()
                if self.closing(near):
                    state, esc_t, clear_since = "ESCAPE", now, None
                    stats["escapes"] += 1
                    event(f"somebody keeps coming closer ({near:.2f} m) -> move out of the way")
                elif d_block > STOP_DIST + 0.3 and near > TOO_CLOSE + 0.15 and now - wait_t > 0.5:
                    state = "FOLLOW"
                    event("way clear -> go on")
                elif now - wait_t > WAIT_REPLAN and now - last_try > 2.0:
                    last_try = now
                    new = self.plan(pose[:2], goal[:2], obstacles)
                    stats["replans"] += 1
                    if new is not None:
                        route, s_robot, state = new, 0.0, "FOLLOW"
                        event(f"blocked {now - wait_t:.0f} s -> re-planned around the obstacle ({new.length:.1f} m)")
                    else:
                        event("re-plan failed, still waiting")

            elif state == "ESCAPE":
                if near > ESCAPE_DONE or now - esc_t > ESCAPE_MAX_T:
                    self.stop()
                    clear_since = now if near > CLEAR_DIST else None
                    state = "CLEAR"
                    event(f"out of the way ({near:.2f} m) -> waiting until the person has passed")
                else:
                    d = self.escape_direction(pos, obstacles, near)
                    if d is None:
                        self.stop()
                    else:
                        self.drive_world(ESCAPE_SPEED * d[0], ESCAPE_SPEED * d[1], 0.0, pose[2])

            elif state == "CLEAR":
                self.stop()
                if self.closing(near):
                    state, esc_t = "ESCAPE", now
                    stats["escapes"] += 1
                    event(f"coming closer again ({near:.2f} m) -> move out of the way")
                elif near > CLEAR_DIST:
                    clear_since = clear_since or now
                    if now - clear_since >= CLEAR_TIME:
                        new = self.plan(pose[:2], goal[:2])
                        stats["replans"] += 1
                        if new is not None:
                            route, s_robot, state = new, 0.0, "FOLLOW"
                            event(f"nobody around for {CLEAR_TIME:.0f} s -> re-planned from here ({new.length:.1f} m)")
                else:
                    clear_since = None
                    if now - esc_t > 6.0 and now - last_try > 2.0:     # somebody standing close: go around
                        last_try = now
                        new = self.plan(pose[:2], goal[:2], obstacles)
                        stats["replans"] += 1
                        if new is not None:
                            route, s_robot, state = new, 0.0, "FOLLOW"
                            event(f"person not moving away -> re-planned around ({new.length:.1f} m)")

            elif state == "FINAL":
                if near < TOO_CLOSE - 0.1:
                    self.stop()                                 # somebody right next to the goal: wait
                elif robot.go_to_step(goal, pose=pose, v_max=0.2, tol=0.05, tol_h=0.05):
                    self.moving = False
                    stats["reached"] = True
                    event("goal reached")
                    break
                else:
                    self.moving = True

            self.record(stats)
            sim.step()

        self.stop()
        stats["time"] = round(sim.getSimulationTime() - t0, 1)
        stats["wall_time"] = round(time.perf_counter() - wall0, 1)
        stats["distance"] = round(path_length(self.trail[trail_start:]), 2)
        stats["min_dist_worker"] = round(stats["min_dist_worker"], 2)
        if not stats["reached"]:
            event("TIMEOUT")
        return stats
