from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QComboBox, QPlainTextEdit, QPushButton
from PyQt6.QtGui import QTextCursor, QTextCharFormat, QColor
from PyQt6.QtCore import Qt
import struct
from pinky_station.protocol.serializer import ParsedMessage

class TerminalWidget(QWidget):
    def __init__(self, max_lines: int = 5000, default_filter: str = "All",
                 parent=None):
        super().__init__(parent)
        self._max_lines = max_lines
        layout = QVBoxLayout(self)

        # Tools layout
        tools_layout = QHBoxLayout()
        self.combo_filter = QComboBox()
        filters = ["All", "DEBUG", "INFO", "WARN", "ERROR"]
        self.combo_filter.addItems(filters)
        if default_filter in filters:
            self.combo_filter.setCurrentText(default_filter)
        self.btn_clear = QPushButton("Clear")

        tools_layout.addWidget(self.combo_filter)
        tools_layout.addWidget(self.btn_clear)
        tools_layout.addStretch()

        self.text_edit = QPlainTextEdit()
        self.text_edit.setReadOnly(True)
        self.text_edit.setMaximumBlockCount(max_lines)
        self.text_edit.setStyleSheet("")  # use QSS theme
        
        layout.addLayout(tools_layout)
        layout.addWidget(self.text_edit)
        
        # Connections
        self.btn_clear.clicked.connect(self.text_edit.clear)
        self.combo_filter.currentTextChanged.connect(self._on_filter_changed)
        
        self._all_logs = []  # Stores (severity, text)

    def append_log(self, severity_idx: int, text: str):
        severities = ["TRACE", "DEBUG", "INFO", "WARN", "ERROR"]
        severity_str = severities[severity_idx] if severity_idx < len(severities) else "UNKNOWN"
        
        self._all_logs.append((severity_str, text))
        if len(self._all_logs) > self._max_lines:
            self._all_logs.pop(0)
            
        # Draw immediately if it matches filter
        current_filter = self.combo_filter.currentText()
        if current_filter == "All" or current_filter == severity_str:
            self._draw_log(severity_str, text)

    def append_msg(self, msg: ParsedMessage):
        try:
            # C++ SerializeDebugLog: severity(u8) + timestamp_ns(u64) + text + null
            severity = msg.payload[0]
            text = msg.payload[9:].split(b"\x00", 1)[0].decode('utf-8')
            self.append_log(severity, text)
        except Exception:
            pass

    def _on_filter_changed(self, filter_text: str):
        self.text_edit.clear()
        for sev, text in self._all_logs:
            if filter_text == "All" or filter_text == sev:
                self._draw_log(sev, text)

    def _draw_log(self, sev: str, text: str):
        # Move cursor to end
        cursor = self.text_edit.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.End)
        self.text_edit.setTextCursor(cursor)
        
        fmt = QTextCharFormat()
        if sev == "ERROR":
            fmt.setForeground(QColor("red"))
            fmt.setFontWeight(75) # Bold
        elif sev == "WARN":
            fmt.setForeground(QColor("yellow"))
        elif sev == "INFO":
            fmt.setForeground(QColor("white"))
        else:
            fmt.setForeground(QColor("gray"))
            
        cursor.setCharFormat(fmt)
        cursor.insertText(f"[{sev}] {text}\n")
        
        # Scroll to bottom
        scrollbar = self.text_edit.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
