from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel, QSizePolicy
from PyQt6.QtGui import QPixmap, QImage
from PyQt6.QtCore import Qt


class VideoViewWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.lbl_image = QLabel("No Camera Feed")
        self.lbl_image.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_image.setStyleSheet(
            "background-color: #060912; color: #1e3050; "
            "border: 1px solid #162032; border-radius: 6px; "
            "font-size: 14px; font-weight: bold;"
        )
        # Prevent the label from dictating widget size based on pixmap
        self.lbl_image.setSizePolicy(
            QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored
        )
        self.lbl_image.setMinimumSize(160, 120)

        layout.addWidget(self.lbl_image)

    def update_frame(self, jpeg_data: bytes):
        if not jpeg_data:
            return

        image = QImage()
        if image.loadFromData(jpeg_data, "JPEG"):
            image = image.mirrored(True, False)  # horizontal flip
            pixmap = QPixmap.fromImage(image)
            w = self.lbl_image.width()
            h = self.lbl_image.height()
            if w > 0 and h > 0:
                scaled = pixmap.scaled(
                    w, h,
                    Qt.AspectRatioMode.KeepAspectRatio,
                    Qt.TransformationMode.SmoothTransformation,
                )
                self.lbl_image.setPixmap(scaled)
            else:
                self.lbl_image.setPixmap(pixmap)
