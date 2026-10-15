"""A* path planner on the occupancy grid.

    planner = Planner(grid)
    path = planner.plan((x0, y0), (x1, y1))   # list of (x, y) waypoints in metres, or None

A* explores grid cells from the start, always expanding first the cell with the lowest
f = g + h, where g = cost of the best known route from the start to that cell and h = straight-
line ("octile") distance from that cell to the goal. Because h never over-estimates the true
remaining cost, the first time the goal is reached the route is the cheapest one.

Cost of a step = its length x (1 + COST_WEIGHT x comfort cost of the cell): routes are short but
keep away from obstacles (in the aisle, they stay near the middle).
The raw route (a staircase of cells) is then smoothed: we keep only the points needed so that
every straight segment between them stays in safe space ("line of sight" shortcut).
"""

import heapq
import math

import numpy as np

COST_WEIGHT = 4.0
SQRT2 = math.sqrt(2.0)
NEIGHBOURS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
              (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2)]


class Planner:
    def __init__(self, grid):
        self.grid = grid

    def _nearest_free(self, i, j, max_r=20):
        """If a start/goal cell is in the forbidden zone, use the nearest allowed cell."""
        lethal = self.grid.lethal
        if not lethal[j, i]:
            return i, j
        for r in range(1, max_r + 1):
            for di in range(-r, r + 1):
                for dj in (-r, r) if abs(di) != r else range(-r, r + 1):
                    a, b = i + di, j + dj
                    if 0 <= a < self.grid.w and 0 <= b < self.grid.h and not lethal[b, a]:
                        return a, b
        return None

    def astar(self, start, goal):
        """A* on cells. start/goal = (i, j). Returns the list of cells, or None."""
        lethal, cost, w, h = self.grid.lethal, self.grid.cost, self.grid.w, self.grid.h
        gi, gj = goal

        def heuristic(i, j):                       # octile distance (8-connected grid)
            dx, dy = abs(i - gi), abs(j - gj)
            return (dx + dy) + (SQRT2 - 2) * min(dx, dy)

        g = np.full((h, w), np.inf)
        parent = {}
        g[start[1], start[0]] = 0.0
        open_heap = [(heuristic(*start), 0.0, start)]
        closed = np.zeros((h, w), bool)
        while open_heap:
            _, gc, (i, j) = heapq.heappop(open_heap)
            if closed[j, i]:
                continue
            if (i, j) == goal:
                cells = [goal]
                while cells[-1] != start:
                    cells.append(parent[cells[-1]])
                return cells[::-1]
            closed[j, i] = True
            for di, dj, step in NEIGHBOURS:
                a, b = i + di, j + dj
                if not (0 <= a < w and 0 <= b < h) or lethal[b, a] or closed[b, a]:
                    continue
                if di and dj and (lethal[j, a] or lethal[b, i]):   # no corner cutting
                    continue
                ng = gc + step * (1.0 + COST_WEIGHT * cost[b, a])
                if ng < g[b, a]:
                    g[b, a] = ng
                    parent[(a, b)] = (i, j)
                    heapq.heappush(open_heap, (ng + heuristic(a, b), ng, (a, b)))
        return None

    def line_is_safe(self, c0, c1, max_cost=0.6):
        """True if the straight segment between two cells stays out of forbidden cells and
        away from obstacles (comfort cost below max_cost)."""
        (i0, j0), (i1, j1) = c0, c1
        n = max(abs(i1 - i0), abs(j1 - j0), 1)
        for k in range(n + 1):
            i = int(round(i0 + (i1 - i0) * k / n))
            j = int(round(j0 + (j1 - j0) * k / n))
            if self.grid.lethal[j, i] or self.grid.cost[j, i] > max_cost:
                return False
        return True

    def smooth(self, cells):
        """Line-of-sight shortcut: from each kept point, jump to the farthest safely visible cell."""
        if len(cells) <= 2:
            return cells
        out, k = [cells[0]], 0
        while k < len(cells) - 1:
            nxt = k + 1
            for m in range(len(cells) - 1, k + 1, -1):
                if self.line_is_safe(cells[k], cells[m]):
                    nxt = m
                    break
            out.append(cells[nxt])
            k = nxt
        return out

    def plan(self, start_xy, goal_xy):
        """Route from start to goal (world metres) as a list of (x, y) waypoints, or None."""
        s = self._nearest_free(*self.grid.to_cell(*start_xy))
        gl = self._nearest_free(*self.grid.to_cell(*goal_xy))
        if s is None or gl is None:
            return None
        cells = self.astar(s, gl)
        if cells is None:
            return None
        self.last_raw = cells
        pts = [self.grid.to_world(i, j) for i, j in self.smooth(cells)]
        pts[0], pts[-1] = tuple(start_xy), tuple(goal_xy)      # exact start / goal
        return pts


def path_length(pts):
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
