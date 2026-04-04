import pyqtgraph as pg
from PyQt6.QtWidgets import QWidget, QVBoxLayout
import numpy as np
import math

class LidarViewWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        
        self.plot_widget = pg.PlotWidget(title="LiDAR 2D Scan")
        self.plot_widget.setAspectLocked(True)
        self.plot_widget.setXRange(-5, 5)
        self.plot_widget.setYRange(-5, 5)
        self.plot_widget.showGrid(x=True, y=True)
        
        # We will plot as a scatter plot
        self.scatter = pg.ScatterPlotItem(
            size=5, pen=pg.mkPen(None), brush=pg.mkBrush(0, 255, 0, 200)
        )
        self.plot_widget.addItem(self.scatter)
        
        layout.addWidget(self.plot_widget)

    def update_scan(self, ranges: list, angle_min: float, angle_increment: float):
        """
        ranges: list of distances in meters
        angle_min: starting angle in radians
        angle_increment: angle between each range in radians
        """
        x_pts = []
        y_pts = []

        for i, r in enumerate(ranges):
            # 0 is usually invalid reading in LiDAR or > max
            if 0.1 < r < 12.0:
                angle = angle_min + i * angle_increment
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                x_pts.append(x)
                y_pts.append(y)

        if x_pts:
            self.scatter.setData(
                x=np.asarray(x_pts, dtype=np.float64),
                y=np.asarray(y_pts, dtype=np.float64),
            )
        else:
            self.scatter.clear()

    def update_sectors(self, sectors: list):
        """
        sectors: list of 24 normalized distance values (0-1 range, /3.5m)
        Each sector covers 15 degrees.
        """
        x_pts = []
        y_pts = []
        sector_angle = 2 * math.pi / len(sectors)
        for i, s in enumerate(sectors):
            # Denormalize: sectors are normalized by /3.5m
            r = s * 3.5
            if r < 0.05:
                continue
            angle = i * sector_angle
            x_pts.append(r * math.cos(angle))
            y_pts.append(r * math.sin(angle))

        if x_pts:
            self.scatter.setData(
                x=np.asarray(x_pts, dtype=np.float64),
                y=np.asarray(y_pts, dtype=np.float64),
            )
        else:
            self.scatter.clear()
