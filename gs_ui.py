"""
gs_ui.py  —  CanSat Ground Station: UI LAYER
Team Kalpana · 2026-CANSAT-ASI-023

This file contains ALL Qt code.
It imports data and logic from gs_logic.py — no business logic lives here.

Structure:
  Theme          — color palette + font helpers
  SimState       — Qt timer that drives playback (needs Qt signals, lives here)
  Widget helpers — make_panel(), make_stat(), vline()
  Painter widgets— AltitudeTapeWidget, AttitudeIndicatorWidget, TrajectoryMapWidget
  Chart          — pyqtgraph time-series chart with fallback plain label
  Tabs           — TelemetryTab, GraphsTab, LocationTab, LiveTab, RecoveryTab
  Shell          — TopBar, Sidebar, CommandDock, SettingsDialog
  MainWindow     — assembles everything; owns recording and tab-switch logic
  main()         — entry point called by ground_station_simple.py
"""

import sys
import os
import csv
import time

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QFrame, QLabel, QPushButton,
    QVBoxLayout, QHBoxLayout, QGridLayout, QStackedWidget, QScrollArea,
    QSlider, QFileDialog, QDialog, QFormLayout, QDoubleSpinBox,
    QSizePolicy, QGraphicsDropShadowEffect,
)
from PyQt6.QtCore import Qt, QTimer, QPointF, QRectF, pyqtSignal, QObject
from PyQt6.QtGui import (
    QPainter, QPen, QBrush, QColor, QFont, QLinearGradient,
    QPolygonF, QPainterPath, QFontMetrics, QCursor, QPixmap,
)

try:
    import pyqtgraph as pg
    import numpy as np
    HAS_PG = True
except ImportError:
    HAS_PG = False
    np = None

# Import everything from the logic layer
import gs_logic as logic
from gs_logic import (
    MISSION_EVENTS, MISSION_DATA, GROUND_STATION,
    STATE_COLOR, fmt_met, haversine,
    TOTAL_PACKETS, PACKET_HZ, MISSION_DURATION,
)

# Resolved once at import time; used by Sidebar and TopBar for the team logo
LOGO_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Team-Kalpana-Logo.png")


def _load_logo(size: int):
    """Return a square QPixmap of the team logo, or None if file is missing."""
    if os.path.exists(LOGO_PATH):
        return QPixmap(LOGO_PATH).scaled(
            size, size,
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
    return None


def _rssi_bars(rssi: float) -> str:
    """
    Convert RSSI (dBm) to a 4-segment bar string.

    How RF Downlink quality is calculated
    ──────────────────────────────────────
    RSSI (Received Signal Strength Indicator) is the raw signal power in dBm
    reported by the radio module.  More negative = weaker signal.

    Signal bars (4 segments):
        ████  rssi ≥ −60 dBm  (excellent)
        ███░  rssi ≥ −70 dBm  (good)
        ██░░  rssi ≥ −80 dBm  (fair)
        █░░░  rssi ≥ −90 dBm  (weak)
        ░░░░  rssi  < −90 dBm  (lost)

    Link quality verdict:
        GOOD  — rssi ≥ −70  (strong signal, minimal packet loss expected)
        WEAK  — rssi ≥ −85  (marginal, some loss possible)
        LOST  — rssi  < −85  (signal below reliable threshold)

    Packet loss % (shown separately):
        loss = (packets_expected − packets_received) / packets_expected × 100
        packets_expected = floor(mission_time × packet_rate) + 1
    """
    if rssi >= -60:   return "████"
    elif rssi >= -70: return "███░"
    elif rssi >= -80: return "██░░"
    elif rssi >= -90: return "█░░░"
    else:             return "░░░░"


# Header state badge: (text_color, background_color) per flight state
_STATE_BADGE = {
    "BOOT":              ("#757575", "#f5f5f5"),
    "TEST_MODE":         ("#e65100", "#fff8e1"),
    "LAUNCH_PAD":        ("#1565c0", "#e3f2fd"),
    "ASCENT":            ("#bf360c", "#fbe9e7"),
    "ROCKET_DEPLOY":     ("#b71c1c", "#ffebee"),
    "DESCENT":           ("#00838f", "#e0f7fa"),
    "AEROBREAK_RELEASE": ("#2e7d32", "#e8f5e9"),
    "IMPACT":            ("#ffffff", "#1a237e"),
}


# ═══════════════════════════════════════════════════════════════════
# THEME
# All colors live in this one dict.
# Edit values here to restyle the entire app — nothing is hard-coded elsewhere.
# ═══════════════════════════════════════════════════════════════════

THEME = {
    "bg0":   "#e4ecf4",   # page background — light blue-grey
    "bg1":   "#ffffff",   # panel background — white
    "bg2":   "#cdd8e4",   # surface / divider
    "line":  "#7a9ab4",   # borders — visible against white
    "text":  "#060c14",   # near-black — maximum contrast
    "dim":   "#1a3650",   # dark navy — label text
    "faint": "#3a5870",   # medium dark — unit text
    "cyan":  "#005898",   # deep blue accent
    "cyan2": "#003870",
    "cyan3": "#001438",
    "green": "#0a5c1e",   # forest green — ok / good
    "amber": "#7a3600",   # dark orange — warning
    "red":   "#8a1414",   # dark red — danger
    "sky":   "#2c5ca0",   # ADI sky color
    "gnd":   "#58310e",   # ADI ground color
}


def c(key: str) -> QColor:
    """Return a QColor for a theme key."""
    return QColor(THEME[key])


def cs(key: str) -> str:
    """Return a hex color string for a theme key (for use in stylesheets)."""
    return THEME[key]


def mono(size: int = 11) -> QFont:
    """Monospace font — used for numeric readouts."""
    f = QFont()
    f.setFamilies(["Menlo", "Courier New", "monospace"])
    f.setPointSize(size)
    return f


def sans(size: int = 11) -> QFont:
    """System sans-serif font — used for labels and UI text."""
    f = QFont()
    f.setPointSize(size)
    return f


# ═══════════════════════════════════════════════════════════════════
# PLAYBACK ENGINE  (SimState)
# A QObject so it can emit signals.
# _tick() fires every 50 ms (20 Hz) and advances mission time.
# The UI connects SimState.updated → MainWindow._on_tick.
# ═══════════════════════════════════════════════════════════════════

class SimState(QObject):
    updated = pyqtSignal()   # fires every 50 ms; UI updates on this signal

    def __init__(self):
        super().__init__()
        self.t       = 0.0    # current mission time in seconds
        self.playing = True   # whether time is advancing
        self.speed   = 1.0    # playback multiplier (0.25 – 6.0)
        self._last   = time.monotonic()

        # 20 Hz timer drives the whole UI refresh loop
        timer = QTimer(self)
        timer.setInterval(50)
        timer.timeout.connect(self._tick)
        timer.start()

    def _tick(self):
        """Advance mission time by wall-clock delta × speed, then notify UI."""
        now = time.monotonic()
        if self.playing:
            self.t += (now - self._last) * self.speed
            if self.t >= logic.MISSION_DURATION:
                self.t = 0.0   # loop back to start
        self._last = now
        self.updated.emit()

    def toggle_play(self):
        self.playing = not self.playing
        self._last   = time.monotonic()

    def seek(self, t: float):
        """Jump to a specific mission time."""
        self.t     = max(0.0, min(logic.MISSION_DURATION, t))
        self._last = time.monotonic()

    @property
    def idx(self) -> int:
        """Index into MISSION_DATA for the current time."""
        return min(logic.TOTAL_PACKETS - 1, int(self.t * logic.PACKET_HZ))

    @property
    def packet(self) -> dict:
        """The telemetry packet dict for the current time."""
        return logic.MISSION_DATA[self.idx]

    @property
    def history(self) -> list:
        """All packets from mission start up to and including now."""
        return logic.MISSION_DATA[:self.idx + 1]


# Global SimState — created in main() after QApplication exists
SIM = None


# ═══════════════════════════════════════════════════════════════════
# LAYOUT HELPERS
# Reusable building blocks so tab code stays short and consistent.
# ═══════════════════════════════════════════════════════════════════

def make_panel(title: str = ""):
    """
    Create a styled card panel.
    Returns (QFrame, body_QVBoxLayout) — add widgets to the layout.
    """
    frame = QFrame()
    frame.setObjectName("panel")
    outer_layout = QVBoxLayout(frame)
    outer_layout.setContentsMargins(0, 0, 0, 0)
    outer_layout.setSpacing(0)

    if title:
        header = QLabel(title.upper())
        header.setObjectName("panel_hdr")
        header.setFixedHeight(22)
        header.setContentsMargins(8, 0, 8, 0)
        header.setFont(sans(10))
        outer_layout.addWidget(header)

    body_widget = QWidget()
    body_layout = QVBoxLayout(body_widget)
    body_layout.setContentsMargins(0, 0, 0, 0)
    body_layout.setSpacing(0)
    outer_layout.addWidget(body_widget)

    return frame, body_layout


def make_stat(label: str, unit: str = ""):
    """
    Create a one-line label → value row for use inside a panel.
    Returns (row_QWidget, value_QLabel).
    Call value_label.setText() to update the displayed value.
    """
    row = QWidget()
    row.setObjectName("stat_row")
    row.setFixedHeight(30)

    hl = QHBoxLayout(row)
    hl.setContentsMargins(10, 0, 10, 0)
    hl.setSpacing(6)

    lbl = QLabel(label)
    lbl.setObjectName("stat_lbl")
    lbl.setFont(sans(12))

    val = QLabel("—")
    val.setObjectName("stat_val")
    val.setFont(mono(14))
    val.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)

    hl.addWidget(lbl)
    hl.addStretch()
    hl.addWidget(val)

    if unit:
        unit_lbl = QLabel(unit)
        unit_lbl.setObjectName("stat_unit")
        unit_lbl.setFont(sans(9))
        hl.addWidget(unit_lbl)

    return row, val


def vline() -> QFrame:
    """Thin vertical divider line for use in horizontal layouts."""
    f = QFrame()
    f.setFrameShape(QFrame.Shape.VLine)
    f.setFixedWidth(1)
    return f


# ═══════════════════════════════════════════════════════════════════
# CUSTOM PAINTER WIDGETS
# These draw directly with QPainter for precise control.
# Each widget has update_data(packet) which stores data and calls
# self.update() to schedule a repaint.
# ═══════════════════════════════════════════════════════════════════

class AltitudeTapeWidget(QWidget):
    """
    Vertical altitude scale with a moving rocket icon.

    Shows:
     - A vertical rail from 0 to 800 m with tick marks every 100 m
     - Apogee marker at 720 m
     - Rocket icon that moves up/down and flips during descent
     - Current altitude readout next to the rocket
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(130, 240)
        self._alt   = 0.0
        self._vel   = 0.0
        self._state = "BOOT"

    def update_data(self, packet):
        self._alt   = packet["altitude"]
        self._vel   = packet["velocity"]
        self._state = packet["state"]
        self.update()   # triggers paintEvent

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        W, H   = self.width(), self.height()
        MAX_ALT = 800
        PAD     = 20
        rail_x  = W // 2

        def alt_to_y(a):
            """Convert altitude in metres to a pixel y-coordinate."""
            return PAD + (1 - min(a, MAX_ALT) / MAX_ALT) * (H - 2 * PAD)

        p.fillRect(0, 0, W, H, c("bg1"))

        # Vertical rail
        p.setPen(QPen(c("line"), 1))
        p.drawLine(rail_x, PAD, rail_x, int(alt_to_y(0)))

        # Tick marks every 100 m; labels every 200 m
        p.setFont(mono(7))
        for alt_m in range(0, MAX_ALT + 1, 100):
            y     = int(alt_to_y(alt_m))
            major = (alt_m % 200 == 0)
            p.setPen(QPen(c("dim"), 1))
            p.drawLine(rail_x - (8 if major else 4), y, rail_x, y)
            if major:
                p.drawText(rail_x - 36, y + 4, str(alt_m))

        # Apogee marker — dashed amber line
        apex_y = int(alt_to_y(720))
        p.setPen(QPen(c("amber"), 1))
        p.drawLine(rail_x - 10, apex_y, rail_x + 10, apex_y)
        p.setFont(mono(7))
        p.drawText(rail_x + 4, apex_y + 3, "APEX 720m")

        # Ground line
        ground_y = int(alt_to_y(0))
        p.setPen(QPen(c("dim"), 1))
        p.drawLine(rail_x - 30, ground_y, rail_x + 30, ground_y)
        p.drawText(rail_x + 4, ground_y + 12, "GND")

        # Rocket icon — flips upside-down during descent
        rocket_y = int(alt_to_y(self._alt))
        p.save()
        p.translate(rail_x, rocket_y)
        if self._state in ("DESCENT", "AEROBREAK_RELEASE", "IMPACT"):
            p.rotate(180)   # flip for descent
        p.setBrush(QBrush(c("cyan")))
        p.setPen(Qt.PenStyle.NoPen)
        body = QPolygonF([
            QPointF(0, -10), QPointF(4, 0), QPointF(3, 7),
            QPointF(-3, 7),  QPointF(-4, 0),
        ])
        p.drawPolygon(body)
        if self._state == "ASCENT":
            # Draw flame during ascent
            p.setBrush(QBrush(c("amber")))
            p.drawPolygon(QPolygonF([QPointF(-2, 7), QPointF(0, 14), QPointF(2, 7)]))
        p.restore()

        # Altitude readout beside the rocket
        p.setFont(mono(9))
        p.setPen(c("cyan"))
        p.drawText(rail_x + 6, max(rocket_y + 7, PAD + 12), f"{self._alt:.1f}m")


class AttitudeIndicatorWidget(QWidget):
    """
    Artificial Horizon / ADI (Attitude Direction Indicator).

    Shows the cansat's roll and pitch by rotating a sky/ground split
    inside a circular bezel.  A fixed amber aircraft symbol stays level
    so you can read attitude against it.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(160, 160)
        self._roll  = 0.0
        self._pitch = 0.0

    def update_data(self, packet):
        # Gyro readings scaled to rough degrees for display
        self._roll  = packet["gyro_r"] * 1.5
        self._pitch = packet["gyro_p"] * 1.2
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        W, H   = self.width(), self.height()
        cx, cy = W // 2, H // 2
        radius = min(W, H) // 2 - 12

        # Clamp pitch shift so the horizon line stays within the bezel
        pitch_shift = max(-radius * 0.75, min(radius * 0.75, self._pitch * 0.9))

        # Outer bezel ring
        p.setPen(QPen(c("line"), 3))
        p.setBrush(QBrush(c("bg2")))
        p.drawEllipse(QPointF(cx, cy), radius + 6, radius + 6)

        # Clip everything inside the round bezel
        clip_path = QPainterPath()
        clip_path.addEllipse(QPointF(cx, cy), radius, radius)
        p.setClipPath(clip_path)

        # Rotate canvas for roll, then draw sky + ground + horizon
        from PyQt6.QtCore import QRect
        p.save()
        p.translate(cx, cy)
        p.rotate(-self._roll)
        p.translate(-cx, -cy)

        sky_top  = cy - radius - 80
        sky_h    = radius + int(pitch_shift) + 80
        gnd_top  = cy + int(pitch_shift)
        gnd_h    = radius + 80

        p.fillRect(QRect(cx - radius - 2, sky_top, (radius + 2) * 2, sky_h), c("sky"))
        p.fillRect(QRect(cx - radius - 2, gnd_top, (radius + 2) * 2, gnd_h), c("gnd"))

        # White horizon line — visible against both sky and ground in daylight
        p.setPen(QPen(QColor("#ffffff"), 2))
        p.drawLine(cx - radius, cy + int(pitch_shift), cx + radius, cy + int(pitch_shift))

        # Pitch ladder — short tick marks above and below horizon
        p.setFont(mono(10))
        for deg in [-20, -10, 10, 20]:
            ty   = cy + int(pitch_shift) - int(deg * 1.2)
            tick = 20 if abs(deg) == 20 else 12
            p.setPen(QPen(QColor("#ffffff"), 1))
            p.drawLine(cx - tick, ty, cx + tick, ty)

        p.restore()
        p.setClipping(False)

        # Fixed amber aircraft crosshair — stays level at centre
        p.setPen(QPen(c("amber"), 2))
        p.drawLine(cx - 28, cy, cx - 6, cy)
        p.drawLine(cx + 6,  cy, cx + 28, cy)
        p.drawLine(cx - 6, cy, cx - 6, cy + 5)
        p.drawLine(cx + 6, cy, cx + 6, cy + 5)
        p.setBrush(QBrush(c("amber")))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(QPointF(cx, cy), 3, 3)

        # Roll / pitch readout boxes — white pill with dark text
        p.setFont(mono(10))
        for text, box_x in [(f"R {self._roll:.1f}°", 4), (f"P {self._pitch:.1f}°", W - 62)]:
            p.setBrush(QBrush(QColor(255, 255, 255, 200)))
            p.setPen(Qt.PenStyle.NoPen)
            p.drawRoundedRect(box_x, H - 20, 58, 16, 3, 3)
            p.setPen(c("text"))
            p.drawText(box_x + 3, H - 7, text)


class TrajectoryMapWidget(QWidget):
    """
    Top-down GPS ground track map.

    Draws:
     - Dashed future path (from remaining simulation packets)
     - Solid trail of where the cansat has been
     - Launch site marker (green cross)
     - Current position (glowing dot)
     - Lat/lon readout at bottom-left
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 180)
        self._history = []
        self._packet  = None
        self._all     = logic.MISSION_DATA   # full path for future dashes

    def update_data(self, packet, history):
        self._packet  = packet
        self._history = history
        self.update()

    def paintEvent(self, event):
        if not self._packet:
            return

        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        W, H = self.width(), self.height()

        # Compute bounding box of the complete path to scale coordinates
        all_lats = [pk["lat"] for pk in self._all]
        all_lons = [pk["lon"] for pk in self._all]
        min_lat  = min(all_lats) - 0.0001
        max_lat  = max(all_lats) + 0.0001
        min_lon  = min(all_lons) - 0.0001
        max_lon  = max(all_lons) + 0.0001

        def to_xy(lat, lon):
            """Convert GPS coordinates to pixel position."""
            x = (lon - min_lon) / (max_lon - min_lon) * W
            y = H - (lat - min_lat) / (max_lat - min_lat) * H
            return QPointF(x, y)

        p.fillRect(0, 0, W, H, c("bg2"))

        # Planned future path — dashed line through remaining points (sampled)
        future_start = len(self._history)
        if future_start < len(self._all):
            future_pts = [to_xy(pk["lat"], pk["lon"])
                          for pk in self._all[future_start::4]]
            if len(future_pts) >= 2:
                p.setPen(QPen(c("dim"), 1, Qt.PenStyle.DashLine))
                for i in range(len(future_pts) - 1):
                    p.drawLine(future_pts[i], future_pts[i + 1])

        # Actual trail — solid line (sampled to max 200 points for performance)
        step  = max(1, len(self._history) // 200)
        trail = [to_xy(pk["lat"], pk["lon"]) for pk in self._history[::step]]
        if len(trail) >= 2:
            p.setPen(QPen(c("cyan"), 2))
            for i in range(len(trail) - 1):
                p.drawLine(trail[i], trail[i + 1])

        # Launch site cross
        launch_pt = to_xy(self._all[0]["lat"], self._all[0]["lon"])
        p.setPen(QPen(c("green"), 2))
        p.drawLine(QPointF(launch_pt.x() - 5, launch_pt.y()),
                   QPointF(launch_pt.x() + 5, launch_pt.y()))
        p.drawLine(QPointF(launch_pt.x(), launch_pt.y() - 5),
                   QPointF(launch_pt.x(), launch_pt.y() + 5))
        p.setFont(mono(12))
        p.setPen(c("green"))
        p.drawText(int(launch_pt.x()) + 6, int(launch_pt.y()), "LAUNCH")

        # Current position — glowing dot
        curr_pt = to_xy(self._packet["lat"], self._packet["lon"])
        glow = c("cyan")
        glow.setAlpha(40)
        p.setBrush(QBrush(glow))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(curr_pt, 10, 10)
        p.setBrush(QBrush(c("cyan")))
        p.drawEllipse(curr_pt, 4, 4)

        # Coordinate readout
        p.setPen(c("dim"))
        p.setFont(mono(10))
        p.drawText(4, H - 14, f"LAT {self._packet['lat']:.5f}°")
        p.drawText(4, H - 5,  f"LON {self._packet['lon']:.5f}°")


# ═══════════════════════════════════════════════════════════════════
# CHART WIDGET
# Uses pyqtgraph when available; falls back to a plain label.
# add_line(fn, color) registers a data series.
# update_data(history) feeds new data on each tick.
# ═══════════════════════════════════════════════════════════════════

class Chart(QWidget):
    """
    Time-series chart.

    Usage:
        ch = Chart("Altitude", "m", color=cs("cyan"), y_min=0, y_max=800)
        ch.add_line(lambda p: p["altitude"], cs("cyan"))
        ch.add_threshold(600, cs("amber"), "600m trigger")
        ch.update_data(history_list)
    """

    def __init__(self, title: str = "", unit: str = "", color: str = "#52d8f0",
                 y_min=None, y_max=None, parent=None):
        super().__init__(parent)
        self._color     = color
        self._accessors = []   # list of (fn, color_str) — one per data line
        self._curves    = []   # matching pyqtgraph PlotDataItem objects

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        if HAS_PG:
            self._plot = pg.PlotWidget()
            self._plot.setBackground(cs("bg1"))
            self._plot.setLabel("left", unit, color=cs("dim"))
            self._plot.showGrid(y=True, alpha=0.15)
            if y_min is not None and y_max is not None:
                self._plot.setYRange(y_min, y_max)
            layout.addWidget(self._plot)
        else:
            self._plot = None
            placeholder = QLabel(f"  {title}  (install pyqtgraph for charts)")
            placeholder.setFont(sans(10))
            layout.addWidget(placeholder)

    def add_line(self, accessor_fn, color=None):
        """
        Add a data series.
        accessor_fn: called with a packet dict, returns a float value.
        color: hex string; defaults to the chart's main color.
        """
        col = color or self._color
        self._accessors.append((accessor_fn, col))
        if HAS_PG and self._plot:
            curve = self._plot.plot(pen=pg.mkPen(color=col, width=1.5))
            self._curves.append(curve)

    def add_threshold(self, y_value: float, color: str = "#7a3600", label: str = ""):
        """Add a dashed horizontal reference line (e.g. the 600 m trigger altitude)."""
        if HAS_PG and self._plot:
            self._plot.addItem(pg.InfiniteLine(
                pos=y_value,
                angle=0,
                pen=pg.mkPen(color=color, width=1.5, style=Qt.PenStyle.DashLine),
                label=label,
                labelOpts={"color": color, "position": 0.95},
            ))

    def update_data(self, history: list):
        """Push the latest packet history into all registered curves."""
        if not history or not HAS_PG or not self._plot:
            return
        ts = np.array([pk["t"] for pk in history])
        for i, (fn, _) in enumerate(self._accessors):
            if i < len(self._curves):
                self._curves[i].setData(ts, np.array([fn(pk) for pk in history]))


# ═══════════════════════════════════════════════════════════════════
# TAB: TELEMETRY
# The main overview tab — altitude tape, ADI, primary values, GNSS,
# environmental sensors, IMU, electrical, event log, descent strip.
# ═══════════════════════════════════════════════════════════════════

class TelemetryTab(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(scroll)

        container = QWidget()
        scroll.setWidget(container)
        grid = QGridLayout(container)
        grid.setContentsMargins(10, 10, 10, 10)
        grid.setSpacing(8)

        # ── Column 0 (rows 0–1): Altitude tape
        alt_panel, alt_body = make_panel("Altitude Profile")
        self._alt_tape = AltitudeTapeWidget()
        alt_body.addWidget(self._alt_tape)
        grid.addWidget(alt_panel, 0, 0, 2, 1)

        # ── Column 1 row 0: Attitude indicator
        adi_panel, adi_body = make_panel("Attitude (ADI)")
        self._adi = AttitudeIndicatorWidget()
        adi_body.addWidget(self._adi)
        grid.addWidget(adi_panel, 0, 1)

        # ── Column 1 row 1: Primary hero values
        hero_panel, hero_body = make_panel("Primary Values")
        self._h_alt = self._add_hero(hero_body, "ALTITUDE", "m")
        self._h_vel = self._add_hero(hero_body, "VELOCITY", "m/s")
        self._h_bat = self._add_hero(hero_body, "VOLTAGE",  "V")
        hero_body.addStretch()
        grid.addWidget(hero_panel, 1, 1)

        # ── Column 2 (rows 0–1): Ground track map
        traj_panel, traj_body = make_panel("Ground Track (GNSS)")
        self._traj = TrajectoryMapWidget()
        traj_body.addWidget(self._traj)
        grid.addWidget(traj_panel, 0, 2, 2, 1)

        # ── Row 2: GNSS · Environmental · IMU
        gnss_panel, gnss_body = make_panel("GNSS · Position")
        self._gnss = {}
        for name, unit in [("TIME", "UTC"), ("LATITUDE", "°N"), ("LONGITUDE", "°E"),
                            ("ALTITUDE", "m"), ("SATELLITES", "")]:
            row, val = make_stat(name, unit)
            gnss_body.addWidget(row)
            self._gnss[name] = val
        grid.addWidget(gnss_panel, 2, 0)

        env_panel, env_body = make_panel("Environmental · Sensors")
        self._env = {}
        for name, unit in [("ALTITUDE", "m"), ("PRESSURE", "hPa"), ("TEMPERATURE", "°C"),
                            ("TVOC", "ppb"), ("eCO₂", "ppm")]:
            row, val = make_stat(name, unit)
            env_body.addWidget(row)
            self._env[name] = val
        grid.addWidget(env_panel, 2, 1)

        imu_panel, imu_body = make_panel("IMU · MPU-6050")
        self._imu = {}
        for name, unit in [("ACC R", "g"), ("ACC P", "g"), ("ACC Y", "g"),
                            ("GYRO R", "°/s"), ("GYRO P", "°/s"), ("GYRO Y", "°/s"),
                            ("SPIN", "°/s")]:
            row, val = make_stat(name, unit)
            imu_body.addWidget(row)
            self._imu[name] = val
        grid.addWidget(imu_panel, 2, 2)

        # ── Row 3: Electrical · Event log
        elec_panel, elec_body = make_panel("Electrical · Power")
        self._elec = {}
        for name, unit in [("VOLTAGE", "V"), ("ESTIMATED", "%"),
                            ("RF RSSI", "dBm"), ("PACKETS RX", "")]:
            row, val = make_stat(name, unit)
            elec_body.addWidget(row)
            self._elec[name] = val
        grid.addWidget(elec_panel, 3, 0)

        evt_panel, evt_body = make_panel("Mission Event Log")
        self._evt_rows = []
        for _ in range(8):
            row = QWidget()
            row.setFixedHeight(24)
            hl = QHBoxLayout(row)
            hl.setContentsMargins(8, 0, 8, 0)
            hl.setSpacing(6)
            ts  = QLabel(); ts.setFont(mono(14)); ts.setFixedWidth(60)
            dot = QLabel("●"); dot.setFont(sans(8)); dot.setFixedWidth(10)
            msg = QLabel(); msg.setFont(sans(14))
            hl.addWidget(ts); hl.addWidget(dot); hl.addWidget(msg); hl.addStretch()
            evt_body.addWidget(row)
            self._evt_rows.append((ts, dot, msg))
        evt_body.addStretch()
        grid.addWidget(evt_panel, 3, 1, 1, 2)

        # ── Row 4: Descent monitoring strip
        desc_panel, desc_body = make_panel("Descent Monitoring")
        desc_inner = QWidget()
        desc_hl    = QHBoxLayout(desc_inner)
        desc_hl.setContentsMargins(12, 6, 12, 6)
        desc_hl.setSpacing(24)

        desc_cells = []
        for title_text in ["Descent Rate", "Primary Chute", "Secondary", "vs 600 m Trigger"]:
            cell = QWidget()
            vl   = QVBoxLayout(cell)
            vl.setContentsMargins(0, 0, 0, 0)
            vl.setSpacing(2)
            ttl = QLabel(title_text.upper())
            ttl.setFont(sans(10))
            vl.addWidget(ttl)
            val_lbl = QLabel("—")
            val_lbl.setFont(mono(13))
            vl.addWidget(val_lbl)
            desc_hl.addWidget(cell)
            desc_cells.append(val_lbl)

        self._dr_val, self._chute1, self._chute2, self._thresh = desc_cells
        desc_hl.addStretch()
        desc_body.addWidget(desc_inner)
        grid.addWidget(desc_panel, 4, 0, 1, 3)

        for col in range(3):
            grid.setColumnStretch(col, 1)

    def _add_hero(self, body, label, unit):
        """Large bold value — used for altitude, velocity, voltage."""
        w  = QWidget()
        vl = QVBoxLayout(w)
        vl.setContentsMargins(10, 6, 10, 4)
        vl.setSpacing(0)
        vl.addWidget(QLabel(label))
        row = QHBoxLayout()
        row.setSpacing(4)
        row.setContentsMargins(0, 0, 0, 0)
        val = QLabel("—")
        val.setFont(mono(22))
        unit_lbl = QLabel(unit)
        unit_lbl.setAlignment(Qt.AlignmentFlag.AlignBottom)
        row.addWidget(val)
        row.addWidget(unit_lbl)
        row.addStretch()
        vl.addLayout(row)
        body.addWidget(w)
        return val

    def update(self, packet, history):
        # Custom painter widgets
        self._alt_tape.update_data(packet)
        self._adi.update_data(packet)
        self._traj.update_data(packet, history)

        # Hero values
        bat_pct = (packet["voltage"] - 6.5) / (8.4 - 6.5) * 100
        self._h_alt.setText(f"{packet['altitude']:.1f}")
        self._h_vel.setText(f"{packet['velocity']:.1f}")
        self._h_bat.setText(f"{packet['voltage']:.2f}")

        # GNSS section
        self._gnss["TIME"].setText(packet["gnss_time"])
        self._gnss["LATITUDE"].setText(f"{packet['lat']:.5f}")
        self._gnss["LONGITUDE"].setText(f"{packet['lon']:.5f}")
        self._gnss["ALTITUDE"].setText(f"{packet['gnss_alt']:.1f}")
        self._gnss["SATELLITES"].setText(str(packet["sats"]))

        # Environmental section
        self._env["ALTITUDE"].setText(f"{packet['altitude']:.1f}")
        self._env["PRESSURE"].setText(f"{packet['pressure']:.1f}")
        self._env["TEMPERATURE"].setText(f"{packet['temp']:.1f}")
        self._env["TVOC"].setText(f"{packet['tvoc']:.0f}")
        self._env["eCO₂"].setText(f"{packet['eco2']:.0f}")

        # IMU section
        self._imu["ACC R"].setText(f"{packet['acc_r']:.2f}")
        self._imu["ACC P"].setText(f"{packet['acc_p']:.2f}")
        self._imu["ACC Y"].setText(f"{packet['acc_y']:.2f}")
        self._imu["GYRO R"].setText(f"{packet['gyro_r']:.1f}")
        self._imu["GYRO P"].setText(f"{packet['gyro_p']:.1f}")
        self._imu["GYRO Y"].setText(f"{packet['gyro_y']:.1f}")
        self._imu["SPIN"].setText(f"{packet['gyro_spin']:.0f}")

        # Electrical section
        self._elec["VOLTAGE"].setText(f"{packet['voltage']:.2f}")
        self._elec["ESTIMATED"].setText(f"{bat_pct:.0f}")
        self._elec["RF RSSI"].setText(f"{packet['rssi']:.0f}")
        self._elec["PACKETS RX"].setText(str(packet["packet"]).zfill(5))

        # Mission event log — show last 8 events in reverse order
        sev_color = {"ok": cs("green"), "warn": cs("amber"), "cyan": cs("cyan")}
        recent_events = [e for e in MISSION_EVENTS if e[0] <= packet["t"]][-8:][::-1]
        for i, (ts_lbl, dot_lbl, msg_lbl) in enumerate(self._evt_rows):
            if i < len(recent_events):
                t, severity, message = recent_events[i]
                ts_lbl.setText(fmt_met(t))
                dot_lbl.setStyleSheet(f"color: {sev_color.get(severity, cs('dim'))};")
                msg_lbl.setText(message)
                ts_lbl.show(); dot_lbl.show(); msg_lbl.show()
            else:
                ts_lbl.hide(); dot_lbl.hide(); msg_lbl.hide()

        # Descent monitoring strip
        vel    = packet["velocity"]
        state  = packet["state"]
        alt    = packet["altitude"]

        # Descent rate — color shows severity
        dr_color = cs("red") if vel < -8 else cs("amber") if vel < -4 else cs("green")
        self._dr_val.setText(f"{vel:+.1f} m/s")
        self._dr_val.setStyleSheet(f"color: {dr_color};")

        # Parachute status
        in_descent = state in ("DESCENT", "AEROBREAK_RELEASE", "IMPACT")
        self._chute1.setText("DEPLOYED" if in_descent else "STOWED")
        self._chute1.setStyleSheet(f"color: {cs('green') if in_descent else cs('dim')};")

        self._chute2.setText("DEPLOYED" if state == "AEROBREAK_RELEASE" else "STOWED")
        self._chute2.setStyleSheet(
            f"color: {cs('green') if state == 'AEROBREAK_RELEASE' else cs('dim')};")

        # Distance from the 600 m deployment trigger altitude
        diff = alt - 600.0
        self._thresh.setText(f"{diff:+.0f} m")
        self._thresh.setStyleSheet(f"color: {cs('amber') if alt < 620 else cs('dim')};")


# ═══════════════════════════════════════════════════════════════════
# TAB: GRAPHS
# Time-series charts in a 3-column scrollable grid.
# To add a chart: add a tuple to single_charts below.
# ═══════════════════════════════════════════════════════════════════

class GraphsTab(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(scroll)

        container = QWidget()
        scroll.setWidget(container)
        grid = QGridLayout(container)
        grid.setContentsMargins(10, 10, 10, 10)
        grid.setSpacing(8)

        # ── Single-line charts ──────────────────────────────────────
        # (title, unit, color, data_accessor_fn, y_min, y_max)
        # Set y_min / y_max to None for auto-scaling.
        single_charts = [
            ("Altitude",    "m",    cs("cyan"),  lambda p: p["altitude"],  0,    800),
            ("Pressure",    "hPa",  "#7878f0",   lambda p: p["pressure"],  None, None),
            ("Voltage",     "V",    cs("amber"), lambda p: p["voltage"],   6.5,  8.6),
            ("Velocity",    "m/s",  "#5090e0",   lambda p: p["velocity"],  None, None),
            ("Temperature", "°C",   cs("green"), lambda p: p["temp"],      None, None),
            ("eCO₂",        "ppm",  cs("amber"), lambda p: p["eco2"],      None, None),
        ]

        self._charts = []
        for i, (title, unit, color, fn, y_min, y_max) in enumerate(single_charts):
            ch = Chart(title, unit, color, y_min, y_max)
            ch.setMinimumHeight(180)
            ch.add_line(fn, color)
            panel, body = make_panel(title)
            body.addWidget(ch)
            grid.addWidget(panel, i // 3, i % 3)
            self._charts.append(ch)

        # Add 600 m trigger line to the altitude chart
        self._charts[0].add_threshold(600, cs("amber"), "600m trigger")

        # ── Multi-line charts ───────────────────────────────────────

        acc_ch = Chart("Accelerometer", "g")
        acc_ch.add_line(lambda p: p["acc_r"], cs("red"))
        acc_ch.add_line(lambda p: p["acc_p"], cs("green"))
        acc_ch.add_line(lambda p: p["acc_y"], "#5090e0")
        acc_ch.setMinimumHeight(180)
        acc_panel, acc_body = make_panel("Accelerometer  R/P/Y")
        acc_body.addWidget(acc_ch)
        grid.addWidget(acc_panel, 2, 0)
        self._charts.append(acc_ch)

        gyro_ch = Chart("Gyroscope", "°/s")
        gyro_ch.add_line(lambda p: p["gyro_r"], cs("red"))
        gyro_ch.add_line(lambda p: p["gyro_p"], cs("green"))
        gyro_ch.add_line(lambda p: p["gyro_y"], "#5090e0")
        gyro_ch.setMinimumHeight(180)
        gyro_panel, gyro_body = make_panel("Gyroscope  R/P/Y")
        gyro_body.addWidget(gyro_ch)
        grid.addWidget(gyro_panel, 2, 1)
        self._charts.append(gyro_ch)

        spin_ch = Chart("Spin Rate", "°/s", cs("cyan"), 0, 4000)
        spin_ch.add_line(lambda p: p["gyro_spin"], cs("cyan"))
        spin_ch.setMinimumHeight(180)
        spin_panel, spin_body = make_panel("Mechanical Spin Rate")
        spin_body.addWidget(spin_ch)
        grid.addWidget(spin_panel, 2, 2)
        self._charts.append(spin_ch)

        for col in range(3):
            grid.setColumnStretch(col, 1)

    def update(self, packet, history):
        for ch in self._charts:
            ch.update_data(history)


# ═══════════════════════════════════════════════════════════════════
# TAB: LOCATION
# Large map + coordinate readout on the left, ADI + orientation on right.
# ═══════════════════════════════════════════════════════════════════

class LocationTab(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        hl = QHBoxLayout(self)
        hl.setContentsMargins(10, 10, 10, 10)
        hl.setSpacing(8)

        # Left column: map + coordinates
        left = QVBoxLayout()
        map_panel, map_body = make_panel("Ground Track · Live")
        self._traj = TrajectoryMapWidget()
        map_body.addWidget(self._traj)
        left.addWidget(map_panel, 3)

        coord_panel, coord_body = make_panel("Coordinates")
        self._loc = {}
        for name, unit in [("LATITUDE", "°N"), ("LONGITUDE", "°E"),
                            ("ALTITUDE", "m"), ("SATS", "")]:
            row, val = make_stat(name, unit)
            coord_body.addWidget(row)
            self._loc[name] = val
        left.addWidget(coord_panel, 1)
        hl.addLayout(left, 3)

        # Right column: ADI + body orientation stats
        right = QVBoxLayout()
        adi_panel, adi_body = make_panel("Attitude Indicator")
        self._adi = AttitudeIndicatorWidget()
        adi_body.addWidget(self._adi)
        right.addWidget(adi_panel, 2)

        orient_panel, orient_body = make_panel("Body Orientation · IMU")
        self._orient = {}
        for name, unit in [("ROLL", "°"), ("PITCH", "°"), ("YAW", "°"), ("SPIN RATE", "°/s")]:
            row, val = make_stat(name, unit)
            orient_body.addWidget(row)
            self._orient[name] = val
        orient_body.addStretch()
        right.addWidget(orient_panel, 1)
        hl.addLayout(right, 2)

    def update(self, packet, history):
        self._traj.update_data(packet, history)
        self._adi.update_data(packet)
        self._loc["LATITUDE"].setText(f"{packet['lat']:.5f}°")
        self._loc["LONGITUDE"].setText(f"{packet['lon']:.5f}°")
        self._loc["ALTITUDE"].setText(f"{packet['gnss_alt']:.1f}")
        self._loc["SATS"].setText(str(packet["sats"]))
        self._orient["ROLL"].setText(f"{packet['gyro_r'] * 1.5:.1f}")
        self._orient["PITCH"].setText(f"{packet['gyro_p'] * 1.2:.1f}")
        self._orient["YAW"].setText(f"{(packet['t'] * 8) % 360:.1f}")
        self._orient["SPIN RATE"].setText(f"{packet['gyro_spin']:.0f}")


# ═══════════════════════════════════════════════════════════════════
# TAB: LIVE
# Video downlink placeholder + telecast stats.
# Replace _CamView.paintEvent with real video rendering when ready.
# ═══════════════════════════════════════════════════════════════════

class LiveTab(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        hl = QHBoxLayout(self)
        hl.setContentsMargins(10, 10, 10, 10)
        hl.setSpacing(8)

        cam_panel, cam_body = make_panel("Onboard Camera · CAM-01")
        self._cam = _CamView()
        cam_body.addWidget(self._cam)
        hl.addWidget(cam_panel, 3)

        stats_panel, stats_body = make_panel("Telecast · Downlink")
        self._tc = {}
        for name, unit in [("BITRATE", "Mbps"), ("FRAMES RX", ""),
                            ("DROPPED", ""), ("LATENCY", "ms"), ("STORAGE", "%")]:
            row, val = make_stat(name, unit)
            stats_body.addWidget(row)
            self._tc[name] = val
        stats_body.addStretch()
        hl.addWidget(stats_panel, 1)

    def update(self, packet, history):
        self._cam.update_data(packet)
        # Placeholder values — replace with real downlink stats when available
        self._tc["BITRATE"].setText("5.2")
        self._tc["FRAMES RX"].setText(str(packet["packet"] * 3).zfill(6))
        self._tc["DROPPED"].setText("0")
        self._tc["LATENCY"].setText("142")
        self._tc["STORAGE"].setText("48.3")


class _CamView(QWidget):
    """Placeholder for video feed — draws a crosshair and 'awaiting' text."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._packet = None

    def update_data(self, packet):
        self._packet = packet
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        W, H   = self.width(), self.height()
        cx, cy = W // 2, H // 2

        p.fillRect(0, 0, W, H, c("bg2"))

        # Crosshair
        p.setPen(QPen(c("cyan"), 1))
        p.drawLine(cx, cy - 20, cx, cy + 20)
        p.drawLine(cx - 20, cy, cx + 20, cy)
        p.drawEllipse(QPointF(cx, cy), 10, 10)

        # Status text
        p.setFont(sans(11))
        p.setPen(c("dim"))
        text = "VIDEO DOWNLINK · AWAITING"
        fm   = QFontMetrics(p.font())
        p.drawText((W - fm.horizontalAdvance(text)) // 2, cy + 50, text)

        # Telemetry overlay when packet is available
        if self._packet:
            p.setFont(mono(9))
            p.setPen(c("cyan"))
            p.drawText(10, 20, f"● REC · {fmt_met(self._packet['t'])}")
            p.setPen(c("dim"))
            p.drawText(W - 160, H - 10,
                       f"ALT {self._packet['altitude']:.0f}m  "
                       f"VEL {self._packet['velocity']:.1f}m/s")


# ═══════════════════════════════════════════════════════════════════
# TAB: RECOVERY  (auto-shown when state reaches IMPACT)
# Displays last-known position, distance + bearing from ground station,
# RF beacon status, ground track map, and auto-saved CSV path.
# ═══════════════════════════════════════════════════════════════════

class RecoveryTab(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        main_vl = QVBoxLayout(self)
        main_vl.setContentsMargins(14, 14, 14, 14)
        main_vl.setSpacing(10)

        # Red banner at the top
        banner = QLabel("▼  RECOVERY MODE  ▼")
        banner.setFont(mono(16))
        banner.setAlignment(Qt.AlignmentFlag.AlignCenter)
        banner.setStyleSheet(f"color: {cs('red')}; padding: 6px 0;")
        main_vl.addWidget(banner)

        hl = QHBoxLayout()
        hl.setSpacing(10)
        main_vl.addLayout(hl, 1)

        # ── Left column: coordinates + distance/bearing + CSV path ──
        left_vl = QVBoxLayout()
        hl.addLayout(left_vl, 1)

        coord_panel, coord_body = make_panel("Last Known Position")
        for name, font_sz in [("LATITUDE", 22), ("LONGITUDE", 22),
                               ("ALTITUDE", 16), ("GNSS TIME", 12)]:
            row = QWidget()
            row.setObjectName("stat_row")
            rl  = QHBoxLayout(row)
            rl.setContentsMargins(8, 4, 8, 4)
            lbl = QLabel(name);  lbl.setObjectName("stat_lbl"); lbl.setFont(sans(9))
            val = QLabel("—");   val.setObjectName("stat_val"); val.setFont(mono(font_sz))
            rl.addWidget(lbl); rl.addStretch(); rl.addWidget(val)
            coord_body.addWidget(row)
            setattr(self, f"_{name.lower().replace(' ', '_')}", val)
        left_vl.addWidget(coord_panel)

        dist_panel, dist_body = make_panel("Distance from Ground Station")
        for name, unit, font_sz in [("DISTANCE", "km", 28), ("BEARING", "°", 22)]:
            row = QWidget()
            row.setObjectName("stat_row")
            rl  = QHBoxLayout(row)
            rl.setContentsMargins(8, 4, 8, 4)
            lbl      = QLabel(name);  lbl.setObjectName("stat_lbl"); lbl.setFont(sans(9))
            val      = QLabel("—");   val.setObjectName("stat_val"); val.setFont(mono(font_sz))
            unit_lbl = QLabel(unit);  unit_lbl.setObjectName("stat_unit"); unit_lbl.setFont(sans(9))
            rl.addWidget(lbl); rl.addStretch(); rl.addWidget(val); rl.addWidget(unit_lbl)
            dist_body.addWidget(row)
            setattr(self, f"_{name.lower()}", val)
        left_vl.addWidget(dist_panel)

        csv_panel, csv_body = make_panel("Auto-saved CSV")
        self._csv_path_lbl = QLabel("No recording")
        self._csv_path_lbl.setFont(mono(9))
        self._csv_path_lbl.setWordWrap(True)
        csv_body.addWidget(self._csv_path_lbl)
        csv_body.addStretch()
        left_vl.addWidget(csv_panel)

        # ── Right column: RF beacon status + map ────────────────────
        right_vl = QVBoxLayout()
        hl.addLayout(right_vl, 2)

        beacon_panel, beacon_body = make_panel("RF Beacon")
        self._beacon_lbl = QLabel("ACTIVE")
        self._beacon_lbl.setFont(mono(20))
        self._beacon_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._beacon_lbl.setStyleSheet(f"color: {cs('green')};")
        beacon_body.addWidget(self._beacon_lbl)
        beacon_body.addStretch()
        right_vl.addWidget(beacon_panel, 1)

        map_panel, map_body = make_panel("Ground Track · Final")
        self._traj = TrajectoryMapWidget()
        map_body.addWidget(self._traj)
        right_vl.addWidget(map_panel, 3)

    def update(self, packet, history, csv_path: str = ""):
        lat, lon = packet["lat"], packet["lon"]
        self._latitude.setText(f"{lat:.6f}°")
        self._longitude.setText(f"{lon:.6f}°")
        self._altitude.setText(f"{packet['gnss_alt']:.1f} m")
        self._gnss_time.setText(packet["gnss_time"])

        # Distance and bearing from ground station using haversine formula
        dist, brg = haversine(GROUND_STATION[0], GROUND_STATION[1], lat, lon)
        self._distance.setText(f"{dist:.3f}")
        self._bearing.setText(f"{brg:.1f}")

        # Beacon health based on RSSI
        if packet["rssi"] >= -80:
            self._beacon_lbl.setText("ACTIVE")
            self._beacon_lbl.setStyleSheet(f"color: {cs('green')};")
        else:
            self._beacon_lbl.setText("WEAK SIGNAL")
            self._beacon_lbl.setStyleSheet(f"color: {cs('amber')};")

        self._traj.update_data(packet, history)

        if csv_path:
            self._csv_path_lbl.setText(csv_path)


# ═══════════════════════════════════════════════════════════════════
# TOP BAR
# Fixed 56 px strip at the top.
# Layout: [logo] [state pill] | [team identity] ··· [REC] [MET] | [PKT] | [RF DOWNLINK]
# ═══════════════════════════════════════════════════════════════════

class TopBar(QWidget):
    """
    Header bar — one unified dark strip.  Four zones separated by vertical lines:
      LEFT    — logo · large state badge · team identity
      CENTRE  — (stretch)
      RIGHT   — REC · MET clock · RF downlink · PKT received/expected · link dot
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setFixedHeight(72)

        hl = QHBoxLayout(self)
        hl.setContentsMargins(16, 0, 20, 0)
        hl.setSpacing(0)

        # ── Logo ────────────────────────────────────────────────────
        logo_lbl = QLabel()
        logo_lbl.setFixedSize(52, 52)
        logo_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        pix = _load_logo(48)
        logo_lbl.setPixmap(pix)

        hl.addWidget(logo_lbl)
        hl.addSpacing(16)

        # ── Flight state badge — dominant, color-coded pill ─────────
        self._state_lbl = QLabel("BOOT")
        self._state_lbl.setFont(mono(16))
        f = self._state_lbl.font()
        f.setBold(True)
        self._state_lbl.setFont(f)
        self._state_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._state_lbl.setContentsMargins(16, 6, 16, 6)
        self._state_lbl.setObjectName("state_badge")
        hl.addWidget(self._state_lbl)
        hl.addSpacing(20)

        # ── Team name + mission ID (stacked) ────────────────────────
        _vl0 = vline(); hl.addWidget(_vl0)
        hl.addSpacing(18)

        id_box = QWidget()
        id_box.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, False)
        id_vl  = QHBoxLayout(id_box)
        id_vl.setContentsMargins(0, 0, 0, 0)
        id_vl.setSpacing(2)

        team_name = QLabel("TEAM KALPANA : ")
        team_name.setFont(mono(16))
        team_name.setObjectName("team_name")

        mission_id = QLabel("2026-INSPACe-CAN-7USAT-056")
        mission_id.setFont(sans(16))
        mission_id.setObjectName("mission_id")

        id_vl.addWidget(team_name)
        id_vl.addWidget(mission_id)
        hl.addWidget(id_box)

        # ── Centre stretch ──────────────────────────────────────────
        hl.addStretch()

        # ── MET clock — largest element on the right ─────────────────
        self._met = QLabel("T+00:00.0")
        self._met.setFont(mono(22))
        self._met.setObjectName("met_clock")
        hl.addWidget(self._met)
        hl.addSpacing(20)

        # ── RF Downlink — signal bars + dBm ─────────────────────────
        _vl1 = vline(); hl.addWidget(_vl1)
        hl.addSpacing(18)

        rf_box = QWidget()
        rf_box.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, False)
        rf_vl  = QVBoxLayout(rf_box)
        rf_vl.setContentsMargins(0, 0, 0, 0)
        rf_hdr = QLabel("RF DOWNLINK")
        rf_hdr.setFont(sans(12))
        rf_hdr.setObjectName("section_hdr")
        rf_row = QHBoxLayout()
        self._rf_bars = QLabel("████")
        self._rf_bars.setFont(mono(11))
        self._rf_rssi = QLabel("-60 dBm")
        self._rf_rssi.setFont(mono(12))
        rf_row.addWidget(self._rf_bars)
        rf_row.addWidget(self._rf_rssi)
        rf_vl.addWidget(rf_hdr)
        rf_vl.addLayout(rf_row)
        hl.addWidget(rf_box)
        hl.addSpacing(18)

        # ── Packet counter — received / expected ─────────────────────
        _vl2 = vline(); hl.addWidget(_vl2)
        hl.addSpacing(18)

        pkt_box = QWidget()
        pkt_box.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, False)
        pkt_vl  = QVBoxLayout(pkt_box)
        pkt_vl.setContentsMargins(0, 0, 0, 0)
        pkt_vl.setSpacing(2)
        pkt_hdr = QLabel("PACKETS")
        pkt_hdr.setFont(sans(20))
        pkt_hdr.setObjectName("section_hdr")
        self._pkt = QLabel("00000 / 00000")
        self._pkt.setFont(mono(12))
        pkt_vl.addWidget(pkt_hdr)
        pkt_vl.addWidget(self._pkt)
        hl.addWidget(pkt_box)
        hl.addSpacing(18)

        # ── Link health dot ──────────────────────────────────────────
        _vl3 = vline(); hl.addWidget(_vl3)
        hl.addSpacing(18)

        link_box = QWidget()
        link_box.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, False)
        link_vl  = QVBoxLayout(link_box)
        link_vl.setContentsMargins(0, 0, 0, 0)
        link_vl.setSpacing(2)
        link_hdr = QLabel("LINK")
        link_hdr.setFont(sans(20))
        link_hdr.setObjectName("section_hdr")
        self._link_dot = QLabel("●")
        self._link_dot.setFont(mono(20))
        self._link_dot.setAlignment(Qt.AlignmentFlag.AlignCenter)
        link_vl.addWidget(link_hdr)
        link_vl.addWidget(self._link_dot)
        hl.addWidget(link_box)

    def update(self, packet, t, recording: bool = False, pkt_expected: int = 0):
        state = packet["state"]

        # State badge — large pill, color changes per state
        txt_col, bg_col = _STATE_BADGE.get(state, ("#757575", "#f5f5f5"))
        self._state_lbl.setText(state.replace("_", " "))
        self._state_lbl.setStyleSheet(
            f"QLabel#state_badge {{"
            f"  color: {txt_col};"
            f"  background-color: {bg_col};"
            f"  border: 2px solid {txt_col};"
            f"  border-radius: 8px;"
            f"}}"
        )

        self._met.setText(fmt_met(t))

        # RF downlink — bars + dBm, both colored by signal strength
        rssi = packet["rssi"]
        if rssi >= -70:
            rf_col = cs("green")
        elif rssi >= -85:
            rf_col = cs("amber")
        else:
            rf_col = cs("red")
        self._rf_bars.setText(_rssi_bars(rssi))
        self._rf_bars.setStyleSheet(f"color: {rf_col};")
        self._rf_rssi.setText(f"{rssi:.0f} dBm")
        self._rf_rssi.setStyleSheet(f"color: {rf_col};")

        # Packet counter — received / expected
        rcvd = packet["packet"]
        self._pkt.setText(f"{rcvd:05d} / {pkt_expected:05d}")
        pkt_loss = max(0.0, (pkt_expected - rcvd) / max(1, pkt_expected))
        if pkt_loss > 0.15:
            self._pkt.setStyleSheet(f"color: {cs('red')};")
        elif pkt_loss > 0.05:
            self._pkt.setStyleSheet(f"color: {cs('amber')};")
        else:
            self._pkt.setStyleSheet(f"color: {cs('green')};")

        # Link health dot — combines RSSI and packet loss
        if rssi >= -70 and pkt_loss < 0.05:
            link_col = cs("green")
        elif rssi >= -85 and pkt_loss < 0.15:
            link_col = cs("amber")
        else:
            link_col = cs("red")
        self._link_dot.setStyleSheet(f"color: {link_col};")

    def apply_theme(self):
        self.setStyleSheet(f"""
            TopBar {{
                background: {cs('bg1')};
                border-bottom: 2px solid {cs('line')};
            }}
            QLabel {{
                color: {cs('text')};
                background: transparent;
                border: none;
            }}
            QFrame {{ border: none; }}
            QFrame[frameShape="5"] {{ border: none; border-left: 1px solid {cs('line')}; min-width: 1px; max-width: 1px; margin: 8px 8px; }}
            QLabel#section_hdr {{ color: {cs('faint')}; font-size: 12pt; }}
            QLabel#mission_id  {{ color: {cs('dim')}; font-weight: bold; }}
            QLabel#team_name   {{ color: {cs('dim')}; font-weight: bold; }}
            QLabel#met_clock   {{ color: {cs('dim')}; }}
            QLabel#rec_lbl     {{ color: {cs('red')}; }}
        """)


# ═══════════════════════════════════════════════════════════════════
# SIDEBAR  (collapsed strip, always 60 px)
# ≡ button opens NavOverlay for full tab names.
# Icon buttons still work as direct shortcuts in collapsed state.
# ═══════════════════════════════════════════════════════════════════

class Sidebar(QWidget):
    """
    Minimal 56 px strip — logo on top, hamburger ≡ opens NavOverlay,
    settings gear at the bottom.  No icon tab buttons — navigation is
    handled entirely through the NavOverlay.
    """
    expand_toggled   = pyqtSignal()
    settings_clicked = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedWidth(56)
        vl = QVBoxLayout(self)
        vl.setContentsMargins(6, 12, 6, 10)
        vl.setSpacing(8)

        # Hamburger button — only way to navigate
        expand_btn = QPushButton("≡")
        expand_btn.setFixedSize(44, 36)
        expand_btn.setObjectName("hamburger_btn")
        expand_btn.setToolTip("Open navigation  (or press 1-4)")
        expand_btn.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
        expand_btn.clicked.connect(self.expand_toggled.emit)
        vl.addWidget(expand_btn)

        vl.addStretch()

        # Settings gear
        settings_btn = QPushButton("⚙")
        settings_btn.setFixedSize(44, 36)
        settings_btn.setObjectName("settings_btn")
        settings_btn.setToolTip("Settings")
        settings_btn.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
        settings_btn.clicked.connect(self.settings_clicked.emit)
        vl.addWidget(settings_btn)

        ver = QLabel("")
        ver.setFont(mono(7))
        vl.addWidget(ver)

    def set_active(self, tab_id: str):
        pass   # highlight is shown in NavOverlay, not sidebar

    def apply_theme(self):
        self.setStyleSheet(f"""
            QWidget {{
                background: {cs('bg1')};
                border-right: 1px solid;
            }}
            QLabel {{ color: {cs('faint')}; background: transparent; border: none; }}
            QPushButton#hamburger_btn {{
                background: {cs('bg2')};
                border: 1px solid;
                border-radius: 8px;
                color: {cs('cyan')};
                font-size: 20px;
                font-weight: bold;
            }}
            QPushButton#hamburger_btn:hover {{
                background: {cs('cyan')};
                color: {cs('bg1')};
                border-color: {cs('cyan')};
            }}
            QPushButton#settings_btn {{
                background: transparent;
                border: 1px solid transparent;
                border-radius: 8px;
                color: {cs('faint')};
                font-size: 35px;
            }}
            QPushButton#settings_btn:hover {{
                background: {cs('bg2')};
                color: {cs('text')};
            }}
        """)


# ═══════════════════════════════════════════════════════════════════
# NAV OVERLAY
# Floating panel that appears over the content when ≡ is pressed.
# Parent must be the central widget so it can be raised above content.
# Width is fixed at 240 px; height tracks the parent on resize.
# ═══════════════════════════════════════════════════════════════════

class NavOverlay(QWidget):
    """
    Floating navigation panel.  Opens when ≡ is pressed; closes when:
      - a tab button is clicked (auto-close + navigate)
      - the dim overlay behind it is clicked
      - Escape is pressed
      - ≡ is pressed again (toggle)

    No logo here (it's in the header). Section labels group the tabs.
    Active tab shows a coloured left-border highlight.
    """
    tab_changed = pyqtSignal(str)
    closed      = pyqtSignal()

    _SECTIONS = [
        ("VIEWS", [
            ("telemetry", "📊", "Telemetry Data"),
            ("graphs",    "📈", "Graphs"),
            ("location",  "🗺 ", "Location & 3D Plot"),
        ]),
        ("TOOLS", [
            ("live", "📹", "Live Telecast"),
        ]),
        ("RECOVERY", [
            ("recovery", "🔴", "Recovery"),
        ]),
    ]

    WIDTH = 250

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setFixedWidth(self.WIDTH)

        shadow = QGraphicsDropShadowEffect(self)
        shadow.setBlurRadius(24)
        shadow.setOffset(6, 0)
        shadow.setColor(QColor(0, 0, 0, 90))
        self.setGraphicsEffect(shadow)

        vl = QVBoxLayout(self)
        vl.setContentsMargins(0, 0, 0, 16)
        vl.setSpacing(0)

        # ── Header — team identity only, no logo ─────────────────────
        header = QWidget()
        header.setObjectName("nav_header")
        header.setFixedHeight(68)
        h_vl = QVBoxLayout(header)
        h_vl.setContentsMargins(16, 14, 16, 10)
        h_vl.setSpacing(3)

        name_lbl = QLabel("TEAM KALPANA")
        name_lbl.setFont(mono(14))
        name_lbl.setObjectName("nav_team")

        id_lbl = QLabel("2026-INSPACe-CAN-7USAT-056")
        id_lbl.setFont(sans(12))
        id_lbl.setObjectName("nav_id")

        h_vl.addWidget(name_lbl)
        h_vl.addWidget(id_lbl)
        vl.addWidget(header)

        # Divider
        div = QFrame(); div.setFrameShape(QFrame.Shape.HLine); div.setFixedHeight(1)
        vl.addWidget(div)
        vl.addSpacing(8)

        # ── Grouped tab buttons ──────────────────────────────────────
        self._btns = {}
        for section_name, tabs in self._SECTIONS:
            # Section label
            sec_lbl = QLabel(section_name)
            sec_lbl.setFont(sans(12))
            sec_lbl.setObjectName("nav_section")
            sec_lbl.setContentsMargins(16, 4, 0, 4)
            vl.addWidget(sec_lbl)

            for tab_id, icon, name in tabs:
                btn = QPushButton(f"  {icon}   {name}")
                btn.setFixedHeight(48)
                btn.setObjectName("nav_full_btn")
                btn.setProperty("active", "0")
                btn.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
                btn.clicked.connect(lambda _, t=tab_id: self._on_tab(t))
                vl.addWidget(btn)
                self._btns[tab_id] = btn

            vl.addSpacing(4)

        vl.addStretch()

    def _on_tab(self, tab_id: str):
        self.tab_changed.emit(tab_id)
        self.closed.emit()

    def set_active(self, tab_id: str):
        for tid, btn in self._btns.items():
            btn.setProperty("active", "1" if tid == tab_id else "0")
            btn.style().unpolish(btn)
            btn.style().polish(btn)

    def apply_theme(self):
        self.setStyleSheet(f"""
            NavOverlay {{
                background: {cs('bg1')};
            }}
            QWidget#nav_header {{
                background: {cs('bg1')};
            }}
            QLabel {{ background: transparent; border: none; color: {cs('text')}; }}
            QLabel#nav_team    {{ color: {cs('dim')}; font-weight: bold; }}
            QLabel#nav_id      {{ color: {cs('faint')}; }}
            QLabel#nav_section {{
                color: {cs('faint')};
                font-size: 12pt;
                letter-spacing: 1px;
                text-transform: uppercase;
            }}
            QFrame {{ border: none; }}
            QPushButton#nav_full_btn {{
                background: transparent;
                border: none;
                border-left: 3px solid transparent;
                border-radius: 0;
                color: {cs('dim')};
                font-size: 12pt;
                text-align: left;
                padding-left: 18px;
            }}
            QPushButton#nav_full_btn:hover {{
                background: {cs('bg2')};
                color: {cs('text')};
                border-left-color: {cs('line')};
            }}
            QPushButton#nav_full_btn[active="1"] {{
                background: {cs('bg2')};
                color: {cs('cyan')};
                border-left: 4px solid {cs('cyan')};
                font-weight: bold;
                padding-left: 17px;
            }}
        """)


# ═══════════════════════════════════════════════════════════════════
# COMMAND DOCK
# Fixed bar at the bottom.  Left side: ground command buttons.
# Right side: playback controls (play/pause, scrubber, clock).
# Emits command_sent(str) for every button press; MainWindow handles it.
# ═══════════════════════════════════════════════════════════════════

class CommandDock(QWidget):
    play_toggled   = pyqtSignal()
    seek_requested = pyqtSignal(float)
    command_sent   = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(76)
        hl = QHBoxLayout(self)
        hl.setContentsMargins(12, 0, 12, 0)
        hl.setSpacing(6)

        # Standard uplink commands
        for label, cmd in [("Boot", "CMD:BOOT"), ("Set Time", "CMD:SET_TIME"),
                            ("Calibrate", "CMD:CAL")]:
            btn = QPushButton(label)
            btn.setFixedHeight(30)
            btn.clicked.connect(lambda _, c=cmd: self.command_sent.emit(c))
            hl.addWidget(btn)

        hl.addWidget(vline())

        # CX toggle — turns on/off and fires recording start/stop
        self._cx_btn = QPushButton("CX ON")
        self._cx_btn.setFixedHeight(30)
        self._cx_btn.setCheckable(True)
        self._cx_btn.setChecked(True)
        self._cx_btn.toggled.connect(self._on_cx_toggle)
        hl.addWidget(self._cx_btn)

        hl.addWidget(vline())

        # SIM mode commands (used in testing)
        for label, cmd in [("SIM EN", "SIM:ENABLE"), ("SIM ACT", "SIM:ACTIVATE"),
                            ("SIM DIS", "SIM:DISABLE")]:
            btn = QPushButton(label)
            btn.setFixedHeight(30)
            btn.clicked.connect(lambda _, c=cmd: self.command_sent.emit(c))
            hl.addWidget(btn)

        hl.addWidget(vline())

        csv_btn = QPushButton("CSV Export")
        csv_btn.setFixedHeight(30)
        csv_btn.clicked.connect(self._export_csv)
        hl.addWidget(csv_btn)

        hl.addStretch()

        # Playback controls (right side)
        self._play_btn = QPushButton("▶ Play")
        self._play_btn.setFixedSize(80, 30)
        self._play_btn.clicked.connect(self.play_toggled.emit)
        hl.addWidget(self._play_btn)

        self._scrubber = QSlider(Qt.Orientation.Horizontal)
        self._scrubber.setRange(0, 1000)
        self._scrubber.setMinimumWidth(200)
        self._scrubber.sliderMoved.connect(
            lambda v: self.seek_requested.emit(v / 1000.0 * logic.MISSION_DURATION))
        hl.addWidget(self._scrubber)

        self._clock = QLabel("T+00:00.0")
        self._clock.setFont(mono(15))
        self._clock.setFixedWidth(100)
        self._clock.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        hl.addWidget(self._clock)

    def _on_cx_toggle(self, on: bool):
        self._cx_btn.setText("CX ON" if on else "CX OFF")
        self.command_sent.emit("CX:ON" if on else "CX:OFF")

    def update(self, packet, t, playing: bool, speed: float):
        self._play_btn.setText("❚❚ Pause" if playing else "▶ Play")
        # Update scrubber without triggering sliderMoved
        self._scrubber.blockSignals(True)
        self._scrubber.setValue(int(t / logic.MISSION_DURATION * 1000))
        self._scrubber.blockSignals(False)
        self._clock.setText(fmt_met(t))

    def _export_csv(self):
        """Manual export of all received data to a user-chosen file."""
        path, _ = QFileDialog.getSaveFileName(
            self, "Export CSV", "Flight_2026-CANSAT-ASI-023.csv", "CSV Files (*.csv)")
        if not path:
            return
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["TIME", "PACKET", "STATE", "ALTITUDE", "PRESSURE", "TEMP",
                              "VOLTAGE", "LAT", "LON", "SATS", "GNSS_ALT", "GNSS_TIME",
                              "ACC_R", "ACC_P", "ACC_Y", "GYRO_R", "GYRO_P", "GYRO_Y",
                              "GYRO_SPIN", "TVOC", "ECO2", "RSSI", "VELOCITY"])
            for pk in logic.MISSION_DATA[:SIM.idx + 1]:
                writer.writerow([
                    f"{pk['t']:.1f}",      pk["packet"],           pk["state"],
                    f"{pk['altitude']:.1f}", f"{pk['pressure']:.1f}", f"{pk['temp']:.1f}",
                    f"{pk['voltage']:.2f}",  f"{pk['lat']:.5f}",      f"{pk['lon']:.5f}",
                    pk["sats"],             f"{pk['gnss_alt']:.1f}", pk["gnss_time"],
                    f"{pk['acc_r']:.2f}",   f"{pk['acc_p']:.2f}",   f"{pk['acc_y']:.2f}",
                    f"{pk['gyro_r']:.2f}",  f"{pk['gyro_p']:.2f}",  f"{pk['gyro_y']:.2f}",
                    f"{pk['gyro_spin']:.1f}", f"{pk['tvoc']:.0f}",
                    f"{pk['eco2']:.0f}",    f"{pk['rssi']:.1f}",    f"{pk['velocity']:.2f}",
                ])

    def apply_theme(self):
        self.setStyleSheet(f"""
            QWidget {{ background: {cs('bg1')}; border-top: 1px solid {cs('line')}; }}
            QPushButton {{
                background: {cs('bg2')}; border: 1px solid {cs('line')};
                border-radius: 4px; color: {cs('text')}; padding: 3px 10px;
            }}
            QPushButton:hover   {{ border-color: {cs('cyan')}; color: {cs('cyan')}; }}
            QPushButton:checked {{ border-color: {cs('cyan')}; color: {cs('cyan')}; }}
            QLabel  {{ color: {cs('text')}; background: transparent; border: none; }}
            QFrame  {{ background: {cs('line')}; }}
            QSlider::groove:horizontal {{
                background: {cs('bg2')}; height: 6px; border-radius: 3px;
            }}
            QSlider::handle:horizontal {{
                background: {cs('cyan')}; width: 14px; height: 14px;
                border-radius: 7px; margin: -4px 0;
            }}
            QSlider::sub-page:horizontal {{
                background: {cs('cyan2')}; border-radius: 3px;
            }}
        """)


# ═══════════════════════════════════════════════════════════════════
# SETTINGS DIALOG
# ═══════════════════════════════════════════════════════════════════

class SettingsDialog(QDialog):
    speed_changed = pyqtSignal(float)
    seek_start    = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Settings")
        self.setFixedSize(300, 230)
        fl = QFormLayout(self)
        fl.setContentsMargins(20, 20, 20, 20)
        fl.setSpacing(12)

        # Playback speed
        speed_sb = QDoubleSpinBox()
        speed_sb.setRange(0.25, 6.0)
        speed_sb.setSingleStep(0.25)
        speed_sb.setValue(1.0)
        speed_sb.setSuffix("×")
        speed_sb.valueChanged.connect(self.speed_changed.emit)
        fl.addRow("Playback speed:", speed_sb)

        restart_btn = QPushButton("Restart Mission")
        restart_btn.clicked.connect(self.seek_start.emit)
        fl.addRow("", restart_btn)

        # Ground station coordinates — used for recovery distance/bearing
        lat_sb = QDoubleSpinBox()
        lat_sb.setRange(-90.0, 90.0)
        lat_sb.setDecimals(4)
        lat_sb.setSingleStep(0.001)
        lat_sb.setValue(GROUND_STATION[0])
        lat_sb.setSuffix("°")
        lat_sb.valueChanged.connect(lambda v: GROUND_STATION.__setitem__(0, v))
        fl.addRow("GS Latitude:", lat_sb)

        lon_sb = QDoubleSpinBox()
        lon_sb.setRange(-180.0, 180.0)
        lon_sb.setDecimals(4)
        lon_sb.setSingleStep(0.001)
        lon_sb.setValue(GROUND_STATION[1])
        lon_sb.setSuffix("°")
        lon_sb.valueChanged.connect(lambda v: GROUND_STATION.__setitem__(1, v))
        fl.addRow("GS Longitude:", lon_sb)


# ═══════════════════════════════════════════════════════════════════
# MAIN WINDOW
# Assembles the full layout: sidebar + topbar + tab stack + dock.
# Owns all session state: recording, CSV file handle, recovery flag.
# ═══════════════════════════════════════════════════════════════════

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CanSat Ground Station · Team Kalpana  [2026-INSPACe-CAN-7USAT-056]")
        self.resize(1400, 860)
        self.setMinimumSize(1024, 680)

        # Session state
        self._settings       = None    # SettingsDialog instance (lazy)
        self._recording      = False
        self._csv_file       = None
        self._csv_writer_obj = None
        self._csv_path       = ""
        self._recovery_shown = False
        self._nav_open       = False   # whether NavOverlay is visible

        self._build_ui()
        self._connect_signals()
        self.apply_theme()
        SIM.updated.connect(self._on_tick)

    # ──────────────────────────────────────────────────────────────
    # UI construction
    # ──────────────────────────────────────────────────────────────

    def _build_ui(self):
        self._central = QWidget()
        self.setCentralWidget(self._central)
        root = QHBoxLayout(self._central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        self._sidebar = Sidebar()
        root.addWidget(self._sidebar)

        right = QWidget()
        right_vl = QVBoxLayout(right)
        right_vl.setContentsMargins(0, 0, 0, 0)
        right_vl.setSpacing(0)

        self._topbar = TopBar()
        right_vl.addWidget(self._topbar)

        # Tab stack — each tab is a full-screen widget
        self._stack         = QStackedWidget()
        self._tab_telemetry = TelemetryTab()
        self._tab_graphs    = GraphsTab()
        self._tab_location  = LocationTab()
        self._tab_live      = LiveTab()
        self._tab_recovery  = RecoveryTab()
        for tab in [self._tab_telemetry, self._tab_graphs,
                    self._tab_location,  self._tab_live, self._tab_recovery]:
            self._stack.addWidget(tab)
        right_vl.addWidget(self._stack, 1)

        self._dock = CommandDock()
        right_vl.addWidget(self._dock)
        root.addWidget(right, 1)

        # Dim overlay — covers content area when nav is open, click closes nav
        self._dim_overlay = QWidget(self._central)
        self._dim_overlay.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self._dim_overlay.setStyleSheet("QWidget { background-color: rgba(0, 0, 0, 110); }")
        self._dim_overlay.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
        self._dim_overlay.mousePressEvent = lambda _e: self._close_nav()
        self._dim_overlay.hide()

        # NavOverlay — child of central, NOT in any layout, floats above content
        self._nav_overlay = NavOverlay(self._central)
        self._nav_overlay.hide()

        self._tab_index = {
            "telemetry": 0,
            "graphs":    1,
            "location":  2,
            "live":      3,
            "recovery":  4,
        }

    def _connect_signals(self):
        self._sidebar.expand_toggled.connect(self._toggle_nav)
        self._sidebar.settings_clicked.connect(self._open_settings)
        self._nav_overlay.tab_changed.connect(self._switch_tab)
        self._nav_overlay.closed.connect(self._close_nav)
        self._dock.play_toggled.connect(SIM.toggle_play)
        self._dock.seek_requested.connect(SIM.seek)
        self._dock.command_sent.connect(self._on_command)

        from PyQt6.QtGui import QShortcut, QKeySequence
        for key, tab in [("1", "telemetry"), ("2", "graphs"),
                          ("3", "location"),  ("4", "live")]:
            QShortcut(QKeySequence(key), self).activated.connect(
                lambda t=tab: self._switch_tab(t))
        QShortcut(QKeySequence(Qt.Key.Key_Space), self).activated.connect(SIM.toggle_play)
        QShortcut(QKeySequence(Qt.Key.Key_Escape), self).activated.connect(self._close_nav)

    # ──────────────────────────────────────────────────────────────
    # Navigation overlay
    # ──────────────────────────────────────────────────────────────

    def _toggle_nav(self):
        if self._nav_open:
            self._close_nav()
        else:
            self._open_nav()

    def _open_nav(self):
        self._position_overlay()
        self._dim_overlay.show()
        self._nav_overlay.show()
        self._nav_overlay.raise_()
        self._nav_open = True

    def _close_nav(self):
        self._nav_overlay.hide()
        self._dim_overlay.hide()
        self._nav_open = False

    def _position_overlay(self):
        """Dim overlay covers full central widget; nav panel sits at x=56 (after sidebar)."""
        w = self._central.width()
        h = self._central.height()
        self._dim_overlay.setGeometry(0, 0, w, h)
        self._nav_overlay.setGeometry(56, 0, NavOverlay.WIDTH, max(h, 400))

    def showEvent(self, event):
        super().showEvent(event)
        QTimer.singleShot(0, self._position_overlay)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._position_overlay()

    # ──────────────────────────────────────────────────────────────
    # Navigation
    # ──────────────────────────────────────────────────────────────

    def _switch_tab(self, tab_id: str):
        self._sidebar.set_active(tab_id)
        self._nav_overlay.set_active(tab_id)
        self._stack.setCurrentIndex(self._tab_index[tab_id])

    def _open_settings(self):
        """Open the settings dialog (created once, reused after that)."""
        if self._settings is None:
            dlg = SettingsDialog(self)
            dlg.speed_changed.connect(lambda s: setattr(SIM, "speed", s))
            dlg.seek_start.connect(lambda: SIM.seek(0))
            self._settings = dlg
        self._settings.show()

    # ──────────────────────────────────────────────────────────────
    # Command handling
    # ──────────────────────────────────────────────────────────────

    def _on_command(self, cmd: str):
        """Called whenever a command button is pressed in the dock."""
        t = SIM.t if SIM is not None else 0.0
        MISSION_EVENTS.append((t, "cyan", f"CMD  {cmd}"))

        # CX ON/OFF controls the auto-recording feature
        if cmd == "CX:ON":
            self._start_recording()
        elif cmd == "CX:OFF":
            self._stop_recording()

    # ──────────────────────────────────────────────────────────────
    # CSV auto-recording
    # CX ON starts a new file; every incoming packet is written live.
    # CX OFF closes the file.
    # ──────────────────────────────────────────────────────────────

    def _start_recording(self):
        if self._recording:
            return
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self._csv_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            f"Flight_{timestamp}_2026-CANSAT-ASI-023.csv")
        self._csv_file       = open(self._csv_path, "w", newline="")
        self._csv_writer_obj = csv.writer(self._csv_file)
        self._csv_writer_obj.writerow([
            "TIME", "PACKET", "STATE", "ALTITUDE", "PRESSURE", "TEMP", "VOLTAGE",
            "LAT", "LON", "SATS", "GNSS_ALT", "GNSS_TIME",
            "ACC_R", "ACC_P", "ACC_Y", "GYRO_R", "GYRO_P", "GYRO_Y",
            "GYRO_SPIN", "TVOC", "ECO2", "RSSI", "VELOCITY",
        ])
        self._recording = True
        MISSION_EVENTS.append((SIM.t if SIM else 0.0, "ok", "Recording started"))

    def _stop_recording(self):
        if not self._recording:
            return
        if self._csv_file:
            self._csv_file.flush()
            self._csv_file.close()
            self._csv_file = None
        self._recording = False
        MISSION_EVENTS.append((SIM.t if SIM else 0.0, "warn", "Recording stopped"))

    # ──────────────────────────────────────────────────────────────
    # Main update loop — fires 20× per second via SimState.updated
    # ──────────────────────────────────────────────────────────────

    def _on_tick(self):
        pk   = SIM.packet
        hist = SIM.history

        # Write one row per packet if recording is active
        if self._recording and self._csv_writer_obj:
            self._csv_writer_obj.writerow([
                f"{pk['t']:.1f}",        pk["packet"],            pk["state"],
                f"{pk['altitude']:.1f}", f"{pk['pressure']:.1f}", f"{pk['temp']:.1f}",
                f"{pk['voltage']:.2f}",  f"{pk['lat']:.5f}",      f"{pk['lon']:.5f}",
                pk["sats"],              f"{pk['gnss_alt']:.1f}",  pk["gnss_time"],
                f"{pk['acc_r']:.2f}",    f"{pk['acc_p']:.2f}",    f"{pk['acc_y']:.2f}",
                f"{pk['gyro_r']:.2f}",   f"{pk['gyro_p']:.2f}",   f"{pk['gyro_y']:.2f}",
                f"{pk['gyro_spin']:.1f}", f"{pk['tvoc']:.0f}",
                f"{pk['eco2']:.0f}",     f"{pk['rssi']:.1f}",     f"{pk['velocity']:.2f}",
            ])
            self._csv_file.flush()

        # Auto-switch to Recovery tab on first IMPACT packet
        if not self._recovery_shown and pk["state"] == "IMPACT":
            self._recovery_shown = True
            self._switch_tab("recovery")
            MISSION_EVENTS.append((SIM.t, "warn", "IMPACT — recovery mode"))

        # Update top bar (uses pkt_expected to compute packet loss %)
        pkt_expected = int(SIM.t * logic.PACKET_HZ) + 1 if SIM.t > 0 else 1
        self._topbar.update(pk, SIM.t, recording=self._recording, pkt_expected=pkt_expected)
        self._dock.update(pk, SIM.t, SIM.playing, SIM.speed)

        # Only update the currently visible tab (saves CPU)
        idx = self._stack.currentIndex()
        if   idx == 0: self._tab_telemetry.update(pk, hist)
        elif idx == 1: self._tab_graphs.update(pk, hist)
        elif idx == 2: self._tab_location.update(pk, hist)
        elif idx == 3: self._tab_live.update(pk, hist)
        elif idx == 4: self._tab_recovery.update(pk, hist, self._csv_path)

    # ──────────────────────────────────────────────────────────────
    # Styling
    # ──────────────────────────────────────────────────────────────

    def apply_theme(self):
        """Apply the THEME palette to every widget. Call once on startup."""
        self.setStyleSheet(f"QMainWindow {{ background: {cs('bg0')}; }}")

        # Qt Style Sheets for the tab content area — panels and stat rows
        self._stack.setStyleSheet(f"""
            QStackedWidget {{ background: {cs('bg0')}; }}
            QScrollArea    {{ background: {cs('bg0')}; border: none; }}
            QWidget        {{ background: {cs('bg0')}; color: {cs('text')}; }}
            QFrame#panel   {{
                background: {cs('bg1')};
                border: 1px solid {cs('line')};
                border-radius: 6px;
            }}
            QLabel#panel_hdr {{
                background: transparent;
                border: none;
                border-bottom: 1px solid {cs('line')};
                color: {cs('dim')};
                letter-spacing: 1px;
            }}
            QWidget#stat_row {{
                background: transparent;
                border-bottom: 1px solid {cs('line')};
            }}
            QLabel#stat_lbl  {{ color: {cs('dim')};   background: transparent; border: none; }}
            QLabel#stat_val  {{ color: {cs('text')};  background: transparent; border: none; }}
            QLabel#stat_unit {{ color: {cs('faint')}; background: transparent; border: none; }}
        """)

        self._topbar.apply_theme()
        self._sidebar.apply_theme()
        self._dock.apply_theme()
        self._nav_overlay.apply_theme()


# ═══════════════════════════════════════════════════════════════════
# ENTRY POINT
# Called by ground_station_simple.py.
# ═══════════════════════════════════════════════════════════════════

def main():
    """
    Boot sequence:
      1. Try to load trial_data.csv next to the script.
      2. Fall back to the built-in simulation if CSV is missing or empty.
      3. Create QApplication and SimState, show MainWindow.
    """
    global SIM

    # Re-bind module-level globals that may be replaced by CSV data
    global MISSION_DATA, TOTAL_PACKETS, MISSION_DURATION, PACKET_HZ

    csv_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "trial_data.csv")
    if os.path.exists(csv_path):
        print(f"Loading {csv_path}…")
        packets = logic.load_csv_data(csv_path)
        if packets:
            # Replace the simulation data with real CSV data
            logic.MISSION_DATA     = packets
            logic.TOTAL_PACKETS    = len(packets)
            logic.MISSION_DURATION = float(len(packets))
            logic.PACKET_HZ        = 1          # 1 packet per second from CSV
            logic.CSV_MODE         = True
            # Also update the local imports
            MISSION_DATA     = logic.MISSION_DATA
            TOTAL_PACKETS    = logic.TOTAL_PACKETS
            MISSION_DURATION = logic.MISSION_DURATION
            PACKET_HZ        = logic.PACKET_HZ
            print(f"  {logic.TOTAL_PACKETS} packets · 1 pkt/s")
        else:
            print("  CSV empty — using built-in simulation")
    else:
        print("trial_data.csv not found — using built-in simulation")

    if HAS_PG:
        pg.setConfigOptions(antialias=True)

    app = QApplication(sys.argv)
    app.setApplicationName("CanSat Ground Station")

    SIM = SimState()
    win = MainWindow()

    if logic.CSV_MODE:
        win.setWindowTitle(
            f"CanSat Ground Station · Team Kalpana  "
            f"[CSV · {logic.TOTAL_PACKETS} packets · 1 pkt/s]")

    win.show()
    sys.exit(app.exec())
