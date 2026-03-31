from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QComboBox, QPlainTextEdit, QPushButton
from PyQt6.QtGui import QTextCursor, QTextCharFormat, QColor
from PyQt6.QtCore import Qt
import struct
from pinky_station.protocol.serializer import ParsedMessage

class TerminalWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        
        # Tools layout
        tools_layout = QHBoxLayout()
        self.combo_filter = QComboBox()
        self.combo_filter.addItems(["All", "DEBUG", "INFO", "WARN", "ERROR"])
        self.btn_clear = QPushButton("Clear")
        
        tools_layout.addWidget(self.combo_filter)
        tools_layout.addWidget(self.btn_clear)
        tools_layout.addStretch()
        
        self.text_edit = QPlainTextEdit()
        self.text_edit.setReadOnly(True)
        self.text_edit.setMaximumBlockCount(5000)
        self.text_edit.setStyleSheet("background-color: #1e1e1e; color: #d4d4d4; font-family: monospace;")
        
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
        if len(self._all_logs) > 5000:
            self._all_logs.pop(0)
            
        # Draw immediately if it matches filter
        current_filter = self.combo_filter.currentText()
        if current_filter == "All" or current_filter == severity_str:
            self._draw_log(severity_str, text)

    def append_msg(self, msg: ParsedMessage):
        try:
            # Struct: severity(1), timestamp_ns(8), length(2), string
            severity = msg.payload[0]
            # ts = struct.unpack('<Q', msg.payload[1:9])[0]
            str_len = struct.unpack('<H', msg.payload[9:11])[0]
            text = msg.payload[11:11+str_len].decode('utf-8')
            
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
