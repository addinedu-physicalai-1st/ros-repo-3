"""A* path planner on static occupancy grid map (for path visualization).

Takes a loaded map image (QImage) + YAML metadata and plans
obstacle-avoiding paths between world-coordinate waypoints.
"""

from __future__ import annotations

import heapq
import math
from typing import Optional

import numpy as np
from PyQt6.QtGui import QImage


class PathPlanner:
  """Grid-based A* planner with obstacle inflation."""

  def __init__(
      self,
      map_image: QImage,
      resolution: float,
      origin: tuple[float, float],
      inflation_radius: float = 0.12,
      occupied_thresh: int = 128,
  ):
    self._resolution = resolution
    self._origin_x = origin[0]
    self._origin_y = origin[1]
    self._inflation_cells = max(1, int(inflation_radius / resolution))
    self._occupied_thresh = occupied_thresh

    self._grid = self._build_grid(map_image)
    self._inflated = self._inflate(self._grid)
    self._h = self._inflated.shape[0]
    self._w = self._inflated.shape[1]

  def _build_grid(self, img: QImage) -> np.ndarray:
    """QImage -> binary grid: True=free, False=obstacle."""
    w, h = img.width(), img.height()
    grid = np.zeros((h, w), dtype=bool)
    for row in range(h):
      for col in range(w):
        gray = img.pixelColor(col, row).lightness()
        grid[row, col] = gray > self._occupied_thresh
    return grid

  def _inflate(self, grid: np.ndarray) -> np.ndarray:
    h, w = grid.shape
    inflated = grid.copy()
    r = self._inflation_cells
    oys, oxs = np.where(~grid)
    for oy, ox in zip(oys, oxs):
      inflated[max(0, oy - r):min(h, oy + r + 1),
               max(0, ox - r):min(w, ox + r + 1)] = False
    return inflated

  def _world_to_grid(self, wx: float, wy: float) -> tuple[int, int]:
    col = int((wx - self._origin_x) / self._resolution)
    row = self._h - 1 - int((wy - self._origin_y) / self._resolution)
    return col, row

  def _grid_to_world(self, col: int, row: int) -> tuple[float, float]:
    wx = col * self._resolution + self._origin_x + self._resolution * 0.5
    wy = (self._h - 1 - row) * self._resolution + self._origin_y + self._resolution * 0.5
    return wx, wy

  def _nearest_free(self, col: int, row: int, radius: int = 60) -> Optional[tuple[int, int]]:
    for r in range(1, radius + 1):
      for dr in range(-r, r + 1):
        for dc in range(-r, r + 1):
          if abs(dr) != r and abs(dc) != r:
            continue
          nr, nc = row + dr, col + dc
          if 0 <= nr < self._h and 0 <= nc < self._w and self._inflated[nr, nc]:
            return nc, nr
    return None

  def plan(
      self,
      start: tuple[float, float],
      goal: tuple[float, float],
  ) -> Optional[list[tuple[float, float]]]:
    """A* from start to goal. Returns world-coord path or None."""
    sc, sr = self._world_to_grid(*start)
    gc, gr = self._world_to_grid(*goal)
    sc = max(0, min(self._w - 1, sc))
    sr = max(0, min(self._h - 1, sr))
    gc = max(0, min(self._w - 1, gc))
    gr = max(0, min(self._h - 1, gr))

    if not self._inflated[sr, sc]:
      fix = self._nearest_free(sc, sr)
      if fix is None:
        return None
      sc, sr = fix
    if not self._inflated[gr, gc]:
      fix = self._nearest_free(gc, gr)
      if fix is None:
        return None
      gc, gr = fix

    start_n = (sr, sc)
    goal_n = (gr, gc)
    if start_n == goal_n:
      return [start, goal]

    open_set: list[tuple[float, int, int]] = [(0.0, sr, sc)]
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    g_score: dict[tuple[int, int], float] = {start_n: 0.0}
    sqrt2 = math.sqrt(2.0)
    nb = [(-1, -1, sqrt2), (-1, 0, 1.0), (-1, 1, sqrt2),
          (0, -1, 1.0), (0, 1, 1.0),
          (1, -1, sqrt2), (1, 0, 1.0), (1, 1, sqrt2)]

    visited = set()
    max_iter = self._w * self._h

    for _ in range(max_iter):
      if not open_set:
        break
      _, cr, cc = heapq.heappop(open_set)
      current = (cr, cc)
      if current in visited:
        continue
      visited.add(current)
      if current == goal_n:
        # Reconstruct
        path_grid = [current]
        while current in came_from:
          current = came_from[current]
          path_grid.append(current)
        path_grid.reverse()
        world_path = [start]
        for r, c in path_grid:
          world_path.append(self._grid_to_world(c, r))
        world_path.append(goal)
        return self._simplify(world_path)
      for dr, dc, cost in nb:
        nr, nc = cr + dr, cc + dc
        if nr < 0 or nr >= self._h or nc < 0 or nc >= self._w:
          continue
        if not self._inflated[nr, nc]:
          continue
        neighbor = (nr, nc)
        if neighbor in visited:
          continue
        tent_g = g_score[current] + cost
        if tent_g < g_score.get(neighbor, float('inf')):
          came_from[neighbor] = current
          g_score[neighbor] = tent_g
          f = tent_g + math.sqrt((nr - gr) ** 2 + (nc - gc) ** 2)
          heapq.heappush(open_set, (f, nr, nc))

    return None

  def _simplify(self, path: list[tuple[float, float]],
                spacing: float = 0.15) -> list[tuple[float, float]]:
    """Downsample path to ~spacing meter intervals."""
    if len(path) <= 2:
      return path
    result = [path[0]]
    accum = 0.0
    for i in range(1, len(path)):
      dx = path[i][0] - path[i - 1][0]
      dy = path[i][1] - path[i - 1][1]
      accum += math.sqrt(dx * dx + dy * dy)
      if accum >= spacing:
        result.append(path[i])
        accum = 0.0
    if result[-1] != path[-1]:
      result.append(path[-1])
    return result

  def plan_through(
      self,
      points: list[tuple[float, float]],
  ) -> Optional[list[tuple[float, float]]]:
    """Plan A* path through ordered points. Returns combined path or None."""
    if len(points) < 2:
      return None
    result: list[tuple[float, float]] = []
    for i in range(len(points) - 1):
      seg = self.plan(points[i], points[i + 1])
      if seg is None:
        return None
      if result:
        seg = seg[1:]
      result.extend(seg)
    return result if result else None
