from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt6.QtGui import QPixmap, QImage
from PyQt6.QtCore import Qt

class VideoViewWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        
        self.lbl_image = QLabel("No Camera Feed")
        self.lbl_image.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_image.setStyleSheet("background-color: black; color: white;")
        
        layout.addWidget(self.lbl_image)

    def update_frame(self, jpeg_data: bytes):
        """
        Updates the QLabel with the new JPEG frame.
        """
        if not jpeg_data:
            return

        image = QImage()
        if image.loadFromData(jpeg_data, "JPEG"):
            pixmap = QPixmap.fromImage(image)
            w = self.lbl_image.width()
            h = self.lbl_image.height()
            if w > 0 and h > 0:
                scaled_pixmap = pixmap.scaled(
                    w, h,
                    Qt.AspectRatioMode.KeepAspectRatio,
                    Qt.TransformationMode.FastTransformation
                )
                self.lbl_image.setPixmap(scaled_pixmap)
            else:
                # Widget not yet laid out — set pixmap directly; Qt will scale later
                self.lbl_image.setPixmap(pixmap)
        else:
            print("Failed to decode JPEG frame.")
