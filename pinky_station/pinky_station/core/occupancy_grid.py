"""Occupancy grid builder using log-odds and Bresenham raycasting.

Builds a 2D occupancy grid map from LiDAR scans + robot pose,
similar to ROS2 SLAM Toolbox / GMapping output.

Map convention (matching ROS2 map_server):
  - 0   = occupied (black in .pgm)
  - 254 = free     (white in .pgm)
  - 205 = unknown  (gray  in .pgm)
"""

from __future__ import annotations

import math
import struct
from pathlib import Path

import numpy as np
from PyQt6.QtGui import QImage


class OccupancyGridBuilder:
    """Incremental occupancy grid using log-odds representation."""

    def __init__(
        self,
        resolution: float = 0.05,
        width: int = 800,
        height: int = 800,
        origin_x: float = -20.0,
        origin_y: float = -20.0,
        max_range: float = 12.0,
        log_odds_hit: float = 0.7,
        log_odds_miss: float = -0.4,
        log_odds_max: float = 5.0,
        log_odds_min: float = -2.0,
        occupied_thresh: float = 0.65,
        free_thresh: float = 0.196,
    ):
        self.resolution = resolution
        self.width = width
        self.height = height
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.max_range = max_range

        self.log_odds_hit = log_odds_hit
        self.log_odds_miss = log_odds_miss
        self.log_odds_max = log_odds_max
        self.log_odds_min = log_odds_min
        self.occupied_thresh = occupied_thresh
        self.free_thresh = free_thresh

        # Log-odds grid: 0.0 = unknown
        self.grid = np.zeros((height, width), dtype=np.float32)
        self._dirty = True

    def reset(self) -> None:
        self.grid[:] = 0.0
        self._dirty = True

    def world_to_grid(self, wx: float, wy: float) -> tuple[int, int]:
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((wy - self.origin_y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> tuple[float, float]:
        wx = gx * self.resolution + self.origin_x
        wy = gy * self.resolution + self.origin_y
        return wx, wy

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width and 0 <= gy < self.height

    def update(self, scan, robot_x: float, robot_y: float, robot_theta: float) -> None:
        """Integrate one LiDAR scan into the occupancy grid.

        Args:
            scan: protobuf LidarScan with angle_min, angle_increment, ranges[]
            robot_x, robot_y, robot_theta: robot pose in world frame
        """
        ranges = list(scan.ranges)
        if not ranges:
            return

        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        range_min = scan.range_min if scan.range_min > 0 else 0.05
        range_max = min(scan.range_max, self.max_range) if scan.range_max > 0 else self.max_range

        rx, ry = self.world_to_grid(robot_x, robot_y)

        for i, r in enumerate(ranges):
            if r < range_min or r > range_max or not math.isfinite(r):
                continue

            beam_angle = robot_theta + angle_min + i * angle_inc
            hit_wx = robot_x + r * math.cos(beam_angle)
            hit_wy = robot_y + r * math.sin(beam_angle)
            hx, hy = self.world_to_grid(hit_wx, hit_wy)

            # Bresenham ray from robot to hit: mark free
            self._trace_ray(rx, ry, hx, hy)

            # Mark hit cell as occupied
            if self.in_bounds(hx, hy):
                self.grid[hy, hx] = min(
                    self.grid[hy, hx] + self.log_odds_hit, self.log_odds_max
                )

        self._dirty = True

    def _trace_ray(self, x0: int, y0: int, x1: int, y1: int) -> None:
        """Bresenham line: mark all cells from (x0,y0) to (x1,y1) exclusive as free."""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        cx, cy = x0, y0
        while True:
            if cx == x1 and cy == y1:
                break
            if self.in_bounds(cx, cy):
                self.grid[cy, cx] = max(
                    self.grid[cy, cx] + self.log_odds_miss, self.log_odds_min
                )
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                cx += sx
            if e2 < dx:
                err += dx
                cy += sy

    def _log_odds_to_prob(self, lo: np.ndarray) -> np.ndarray:
        """Convert log-odds to probability [0, 1]."""
        return 1.0 / (1.0 + np.exp(-lo))

    def to_pgm_array(self) -> np.ndarray:
        """Convert grid to PGM-style uint8 array.

        Returns: (height, width) uint8 array
          205 = unknown, 254 = free, 0 = occupied
        """
        prob = self._log_odds_to_prob(self.grid)
        result = np.full((self.height, self.width), 205, dtype=np.uint8)

        free_mask = prob < (1.0 - self.free_thresh)
        occupied_mask = prob > self.occupied_thresh
        # Cells with near-zero log-odds remain unknown (205)
        unknown_mask = np.abs(self.grid) < 0.1

        result[free_mask & ~unknown_mask] = 254
        result[occupied_mask & ~unknown_mask] = 0
        return result

    def to_qimage(self) -> QImage:
        """Convert grid to QImage for rendering on MapWidget."""
        pgm = self.to_pgm_array()
        # Flip vertically: grid row 0 is world-bottom, QImage row 0 is top
        pgm_flipped = np.flipud(pgm).copy()
        h, w = pgm_flipped.shape
        return QImage(pgm_flipped.data, w, h, w, QImage.Format.Format_Grayscale8).copy()

    def save(self, file_path: str | Path) -> tuple[str, str]:
        """Save map as .pgm + .yaml (ROS2 map_server compatible).

        Args:
            file_path: path without extension (e.g., "my_map")

        Returns:
            (pgm_path, yaml_path) as strings
        """
        base = Path(file_path)
        pgm_path = base.with_suffix(".pgm")
        yaml_path = base.with_suffix(".yaml")

        pgm = self.to_pgm_array()
        # PGM: row 0 = top of image = world top (flip Y)
        pgm_flipped = np.flipud(pgm)

        # Write PGM (P5 binary)
        with open(pgm_path, "wb") as f:
            header = f"P5\n{self.width} {self.height}\n255\n"
            f.write(header.encode("ascii"))
            f.write(pgm_flipped.tobytes())

        # Write YAML metadata
        yaml_content = (
            f"image: {pgm_path.name}\n"
            f"mode: trinary\n"
            f"resolution: {self.resolution}\n"
            f"origin: [{self.origin_x}, {self.origin_y}, 0.0]\n"
            f"negate: 0\n"
            f"occupied_thresh: {self.occupied_thresh}\n"
            f"free_thresh: {self.free_thresh}\n"
        )
        yaml_path.write_text(yaml_content)

        return str(pgm_path), str(yaml_path)
