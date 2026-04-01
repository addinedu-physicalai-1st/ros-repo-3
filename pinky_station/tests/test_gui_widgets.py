"""Unit tests for GUI widgets."""

import struct
import pytest
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QApplication

from pinky_station.gui.widgets.teleop_widget import TeleopWidget
from pinky_station.gui.widgets.terminal_widget import TerminalWidget
from pinky_station.gui.widgets.battery_widget import BatteryWidget
from pinky_station.gui.widgets.map_widget import MapWidget
from pinky_station.gui.widgets.toolbar import ToolbarWidget
from pinky_station.protocol.serializer import ParsedMessage


def test_teleop_widget_signals(qapp):
    """Test TeleopWidget speed slider and button signals."""
    widget = TeleopWidget(default_speed=0.2, max_speed=0.5, angular_speed=1.0)
    
    emitted_cmds = []
    widget.sig_cmd_vel.connect(lambda v, w: emitted_cmds.append((v, w)))
    
    # Simulate clicking 'W' (forward)
    widget.btn_fwd.pressed.emit()
    assert len(emitted_cmds) == 1
    assert emitted_cmds[-1] == (0.2, 0.0)
    
    # Simulate clicking 'A' (left)
    widget.btn_left.pressed.emit()
    assert len(emitted_cmds) == 2
    assert emitted_cmds[-1] == (0.0, 1.0)
    
    # Simulate clicking 'S' (backward)
    widget.btn_bwd.pressed.emit()
    assert len(emitted_cmds) == 3
    assert emitted_cmds[-1] == (-0.2, 0.0)

    # Simulate clicking 'D' (right)
    widget.btn_right.pressed.emit()
    assert len(emitted_cmds) == 4
    assert emitted_cmds[-1] == (0.0, -1.0)

    # Change speed
    widget.slider_speed.setValue(40) # 0.4 m/s
    widget.btn_fwd.pressed.emit()
    assert emitted_cmds[-1] == (0.4, 0.0)


def test_terminal_widget(qapp):
    """Test TerminalWidget appending logs and filtering."""
    widget = TerminalWidget(max_lines=10, default_filter="All")
    
    # 0=TRACE, 1=DEBUG, 2=INFO, 3=WARN, 4=ERROR
    widget.append_log(2, "Info message")
    widget.append_log(3, "Warn message")
    widget.append_log(4, "Error message")
    
    text = widget.text_edit.toPlainText()
    assert "Info message" in text
    assert "Warn message" in text
    assert "Error message" in text
    
    # Change filter to ERROR
    widget.combo_filter.setCurrentText("ERROR")
    text_filtered = widget.text_edit.toPlainText()
    assert "Error message" in text_filtered
    assert "Info message" not in text_filtered
    
    # Clear terminal
    widget.btn_clear.click()
    assert widget.text_edit.toPlainText() == ""


def test_battery_widget(qapp):
    """Test BatteryWidget updating values."""
    widget = BatteryWidget()
    
    # C++ SerializeBattery: voltage(f32) + percentage(f32) + status(u8) = 9 bytes
    payload = struct.pack('<ffB', 12.5, 80.0, 0)
    msg = ParsedMessage(msg_type=0, payload=payload)
    
    widget.update_status(msg)
    
    assert "12.50 V" in widget.lbl_voltage.text()
    assert widget.progress.value() == 80


def test_map_widget_signals(qapp):
    """Test MapWidget updating robot pose."""
    widget = MapWidget(scale=50.0)
    
    # C++ SerializeOdom: x,y,theta,vx,vth as 5 x float32 = 20 bytes
    payload = struct.pack('<5f', 1.0, 2.0, 3.14, 0.1, 0.2)
    msg = ParsedMessage(msg_type=0, payload=payload)
    
    widget.update_odom(msg)
    
    assert widget.robot_x == 1.0
    assert widget.robot_y == 2.0
    # Floating point comparison
    assert abs(widget.robot_theta - 3.14) < 1e-5


def test_toolbar_widget_signals(qapp):
    """Test ToolbarWidget UI signals."""
    widget = ToolbarWidget(default_host="192.168.1.5")
    assert widget.input_ip.text() == "192.168.1.5"
    
    emitted_ips = []
    widget.sig_connect_toggled.connect(lambda ip: emitted_ips.append(ip))
    
    # Click Connect
    widget.btn_connect.click()
    assert len(emitted_ips) == 1
    assert emitted_ips[-1] == "192.168.1.5"
    
    # Update state to CONNECTED
    widget.set_status(True, "CONNECTED", "green")
    assert widget.btn_connect.text() == "Disconnect"
    
    # Click Disconnect
    widget.btn_connect.click()
    assert len(emitted_ips) == 2
    assert emitted_ips[-1] == ""
