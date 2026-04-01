"""Station configuration loader.

Reads ``station_config.yaml`` and exposes typed dataclass objects.  Every
field has a default that matches the previous hardcoded values, so the
station works identically when no YAML file is present.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict

import yaml


# ── Dataclasses ──────────────────────────────────────────────────────
@dataclass
class ConnectionConfig:
    default_host: str = "192.168.0.100"
    tcp_port: int = 9100
    udp_port: int = 9200
    ping_interval_s: float = 2.0
    connect_timeout_s: float = 5.0


@dataclass
class GuiConfig:
    window_width: int = 1200
    window_height: int = 800
    camera_fps: int = 15
    map_scale: float = 50.0


@dataclass
class TeleopConfig:
    default_linear_speed: float = 0.10
    max_linear_speed: float = 0.26
    default_angular_speed: float = 0.5
    max_angular_speed: float = 1.0


@dataclass
class TerminalConfig:
    max_lines: int = 5000
    default_filter: str = "All"


@dataclass
class StationConfig:
    connection: ConnectionConfig = field(default_factory=ConnectionConfig)
    gui: GuiConfig = field(default_factory=GuiConfig)
    teleop: TeleopConfig = field(default_factory=TeleopConfig)
    terminal: TerminalConfig = field(default_factory=TerminalConfig)


# ── Loader ───────────────────────────────────────────────────────────
def _merge(dc_cls: type, raw: Dict[str, Any]) -> object:
    """Construct a dataclass from *raw* dict, ignoring unknown keys."""
    known = {f.name for f in dc_cls.__dataclass_fields__.values()}
    return dc_cls(**{k: v for k, v in raw.items() if k in known})


def load_station_config(path: str | Path) -> StationConfig:
    """Load a YAML file and return a populated :class:`StationConfig`.

    Missing sections or keys fall back to defaults.
    If the file does not exist or is invalid YAML, returns all-default config.
    """
    cfg = StationConfig()
    try:
        with open(path) as fh:
            data = yaml.safe_load(fh)
    except (OSError, yaml.YAMLError):
        return cfg

    if not isinstance(data, dict):
        return cfg

    if "connection" in data and isinstance(data["connection"], dict):
        cfg.connection = _merge(ConnectionConfig, data["connection"])
    if "gui" in data and isinstance(data["gui"], dict):
        cfg.gui = _merge(GuiConfig, data["gui"])
    if "teleop" in data and isinstance(data["teleop"], dict):
        cfg.teleop = _merge(TeleopConfig, data["teleop"])
    if "terminal" in data and isinstance(data["terminal"], dict):
        cfg.terminal = _merge(TerminalConfig, data["terminal"])

    return cfg
