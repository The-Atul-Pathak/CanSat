"""
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
import math
import time


def _ensure_qt_plugin_path():
    """
    Make sure Qt can find its platform plugin (cocoa/xcb/windows) before the
    first PyQt6 import creates a QApplication.

    When the virtual-env's Python is a *symlink* into a framework build (e.g.
    Apple's CommandLineTools Python on macOS), Qt resolves its plugin search
    path relative to the real interpreter deep inside the framework and fails
    to look inside the PyQt6 package — QApplication then aborts with
    "Could not find the Qt platform plugin 'cocoa'". Pointing Qt at the
    bundled plugins explicitly fixes that. It is a no-op when the environment
    is already set or the plugins live elsewhere.
    """
    try:
        import PyQt6
    except ImportError:
        return
    plugins = os.path.join(os.path.dirname(PyQt6.__file__), "Qt6", "plugins")
    if not os.path.isdir(os.path.join(plugins, "platforms")):
        return
    _fix_platform_plugins(plugins)
    if not os.environ.get("QT_QPA_PLATFORM_PLUGIN_PATH"):
        os.environ.setdefault("QT_PLUGIN_PATH", plugins)
        os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = os.path.join(plugins, "platforms")


def _fix_platform_plugins(plugins_dir):
    """
    Make the Qt platform plugins loadable again on an iCloud-synced checkout.

    Two separate macOS behaviours conspire to break QApplication with
    "Could not find the Qt platform plugin 'cocoa'" even though the dylib is
    sitting right there in plugins/platforms:

    1. UF_HIDDEN — pip leaves the "hidden" flag on the files it extracts from
       the PyQt6-Qt6 wheel. Qt enumerates plugin directories with QDir's default
       filter, which excludes hidden entries, so the scan returns *nothing* and
       Qt never even opens the file. This is the actual cause of the abort.

    2. SF_DATALESS — when the project lives under an iCloud-synced folder such
       as ~/Documents, "Optimise Mac Storage" evicts file contents and leaves a
       placeholder. chflags() on a dataless file reports success but does not
       persist, so clearing UF_HIDDEN silently does nothing.

    Order therefore matters: materialise first (reading the file forces the File
    Provider to fetch it), then clear the hidden flag. Only the handful of files
    in plugins/platforms is touched, so the cost is bounded, and both steps are
    no-ops once they have been applied.
    """
    import stat
    uf_hidden   = getattr(stat, "UF_HIDDEN", 0x8000)
    sf_dataless = getattr(stat, "SF_DATALESS", 0x40000000)
    if not hasattr(os, "chflags"):
        return                 # not macOS
    platforms = os.path.join(plugins_dir, "platforms")
    try:
        names = os.listdir(platforms)
    except OSError:
        return
    for name in names:
        path = os.path.join(platforms, name)
        try:
            flags = os.stat(path).st_flags
            if flags & sf_dataless:
                with open(path, "rb") as fh:
                    while fh.read(1 << 20):
                        pass   # read through to force full materialisation
                flags = os.stat(path).st_flags
            if flags & uf_hidden:
                # Mask to the user-settable flags; SF_* bits cannot be written
                # back and would make chflags fail with EPERM.
                os.chflags(path, flags & ~uf_hidden & 0xffff)
        except OSError:
            pass               # nothing we can do; Qt will report the failure


def _warn_if_cloud_evicted():
    """Print an actionable warning when the venv is being streamed from iCloud."""
    import stat
    sf_dataless = getattr(stat, "SF_DATALESS", 0x40000000)
    venv = os.environ.get("VIRTUAL_ENV") or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), ".venv")
    site = os.path.join(venv, "lib")
    evicted = 0
    for root, dirs, files in os.walk(site):
        for name in files:
            try:
                if os.stat(os.path.join(root, name)).st_flags & sf_dataless:
                    evicted += 1
            except OSError:
                pass
        if evicted > 200:      # enough evidence; stop walking
            break
    if evicted:
        print(f"WARNING: {evicted}+ files in {venv} are evicted to iCloud.")
        print("         Every launch re-downloads them, which is very slow.")
        print("         Move the virtualenv outside the synced folder, e.g.:")
        print("           python3 -m venv ~/.venvs/cansat26 && "
              "~/.venvs/cansat26/bin/pip install -r requirements.txt")


_ensure_qt_plugin_path()

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QFrame, QLabel, QPushButton,
    QVBoxLayout, QHBoxLayout, QGridLayout, QStackedWidget, QScrollArea,
    QSlider, QFileDialog, QDialog, QFormLayout, QDoubleSpinBox,
    QGraphicsDropShadowEffect,
)
from PyQt6.QtCore import Qt, QTimer, QPointF, QRect, pyqtSignal, QObject
from PyQt6.QtGui import (
    QPainter, QPen, QBrush, QColor, QFont, QPolygonF, QPainterPath,
    QFontMetrics, QCursor, QPixmap, QShortcut, QKeySequence,
)

try:
    import pyqtgraph as pg
    import numpy as np
    HAS_PG = True
except ImportError:
    HAS_PG = False
    np = None

try:
    from PyQt6.QtWebEngineWidgets import QWebEngineView
    import json as _json
    HAS_WEBENGINE = True
except ImportError:
    HAS_WEBENGINE = False

# Import everything from the logic layer
import gs_logic as logic
from gs_logic import (
    MISSION_EVENTS, GROUND_STATION, fmt_met, haversine,
    TEAM_ID, CSV_FILENAME, TELEMETRY_HEADER, telemetry_row,
)
# NOTE: MISSION_DATA / TOTAL_PACKETS / PACKET_HZ / MISSION_DURATION are
# deliberately NOT imported by value — main() rebinds them on gs_logic when
# trial_data.csv loads, so they must always be read as logic.<NAME>.

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

        # Cached history slice — MISSION_DATA[:idx+1] is rebuilt only when idx
        # actually moves, not on every one of the 20 ticks per second.
        self._hist_idx  = -1
        self._hist      = []

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
        """
        All packets from mission start up to and including now.

        The slice is cached: at 20 Hz with 1 Hz CSV data this would otherwise
        copy a several-hundred-element list 20 times per second to produce the
        identical result 19 of those times.
        """
        idx = self.idx
        if idx != self._hist_idx:
            self._hist     = logic.MISSION_DATA[:idx + 1]
            self._hist_idx = idx
        return self._hist


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
     - Apogee marker at the actual max altitude in the loaded mission data
     - Rocket icon that moves up/down and flips during descent
     - Current altitude readout next to the rocket
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(130, 240)
        self._apex  = max((pk["altitude"] for pk in logic.MISSION_DATA), default=0.0)
        # Scale the rail to the mission's actual apex, rounded up to the next
        # 100 m, with 800 m as a floor. It used to be hard-coded to 800 m, so a
        # flight that went higher (trial_data.csv peaks at 840 m) pinned the
        # rocket to the top of the tape and drew the apex label over the scale.
        self._max_alt = max(800, int(math.ceil((self._apex * 1.05) / 100.0)) * 100)
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
        MAX_ALT = self._max_alt
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

        # Apogee marker — amber line, labelled on the LEFT of the rail. The
        # label used to sit on the right, where it printed directly on top of
        # the live altitude readout every time the cansat was near apogee.
        apex_y = int(alt_to_y(self._apex))
        p.setPen(QPen(c("amber"), 1))
        p.drawLine(rail_x - 10, apex_y, rail_x + 10, apex_y)
        p.setFont(mono(7))
        apex_txt = f"APEX {self._apex:.0f}m"
        p.drawText(rail_x - 14 - QFontMetrics(p.font()).horizontalAdvance(apex_txt),
                   apex_y + 3, apex_txt)

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

        # Pre-compute the bounding box and the unit-square (0..1) projection of
        # every point once. paintEvent used to re-scan all of MISSION_DATA and
        # rebuild hundreds of QPointF objects on every single repaint.
        lats = [pk["lat"] for pk in self._all] or [0.0]
        lons = [pk["lon"] for pk in self._all] or [0.0]
        self._min_lat = min(lats) - 0.0001
        self._max_lat = max(lats) + 0.0001
        self._min_lon = min(lons) - 0.0001
        self._max_lon = max(lons) + 0.0001
        self._span_lat = self._max_lat - self._min_lat
        self._span_lon = self._max_lon - self._min_lon
        self._norm = [self._to_unit(pk["lat"], pk["lon"]) for pk in self._all]

    def _to_unit(self, lat, lon):
        """Project a coordinate into the unit square (independent of widget size)."""
        return ((lon - self._min_lon) / self._span_lon,
                1.0 - (lat - self._min_lat) / self._span_lat)

    def update_data(self, packet, history):
        self._packet  = packet
        self._history = history
        self.update()

    def paintEvent(self, event):
        if not self._packet or not self._norm:
            return

        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        W, H = self.width(), self.height()

        def to_xy(lat, lon):
            """Convert GPS coordinates to pixel position."""
            ux, uy = self._to_unit(lat, lon)
            return QPointF(ux * W, uy * H)

        def scale(pts):
            """Scale cached unit-square points to the current widget size."""
            return QPolygonF([QPointF(ux * W, uy * H) for ux, uy in pts])

        p.fillRect(0, 0, W, H, c("bg2"))

        # Planned future path — dashed polyline through remaining points (sampled)
        future_start = len(self._history)
        if future_start < len(self._norm):
            future = scale(self._norm[future_start::4])
            if future.count() >= 2:
                p.setPen(QPen(c("dim"), 1, Qt.PenStyle.DashLine))
                p.drawPolyline(future)

        # Actual trail — solid polyline (sampled to max 200 points for performance)
        step  = max(1, len(self._history) // 200)
        trail = scale(self._norm[:future_start:step])
        if trail.count() >= 2:
            p.setPen(QPen(c("cyan"), 2))
            p.drawPolyline(trail)

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


class MapWidget(QWidget):
    """
    Tile-based GPS map using Leaflet.js + OpenStreetMap inside a QWebEngineView.
    Falls back to TrajectoryMapWidget if PyQt6-WebEngine is unavailable.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self._vl = QVBoxLayout(self)
        self._vl.setContentsMargins(0, 0, 0, 0)
        self._ready    = False
        self._queued   = None
        self._web      = None
        self._fallback = None
        self._last_idx = -1        # length of the last history we pushed

        if not HAS_WEBENGINE:
            self._use_fallback()

    def showEvent(self, event):
        """
        Build the QWebEngineView the first time this map actually becomes
        visible.  Two MapWidgets exist (Telemetry and Location tabs); creating
        both up front spawned two renderer processes and fetched Leaflet twice
        before the window had even appeared.
        """
        super().showEvent(event)
        if HAS_WEBENGINE and self._web is None and self._fallback is None:
            self._web = QWebEngineView()
            self._vl.addWidget(self._web)
            self._load_map()

    def _use_fallback(self):
        """Swap in the offline painter map (no WebEngine, or Leaflet unreachable)."""
        if self._fallback is not None:
            return
        if self._web is not None:
            self._web.setParent(None)
            self._web.deleteLater()
            self._web = None
        self._fallback = TrajectoryMapWidget()
        self._vl.addWidget(self._fallback)
        if self._queued:
            self._fallback.update_data(*self._queued)
            self._queued = None

    def _load_map(self):
        step     = max(1, len(logic.MISSION_DATA) // 400)
        all_pts  = [[round(pk["lat"], 6), round(pk["lon"], 6)]
                    for pk in logic.MISSION_DATA[::step]]
        center   = [logic.MISSION_DATA[0]["lat"], logic.MISSION_DATA[0]["lon"]]
        pts_json = _json.dumps(all_pts)

        html = f"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8"/>
<style>html,body,#map{{margin:0;padding:0;width:100%;height:100%;}}</style>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
</head>
<body>
<div id="map"></div>
<script>
var map = L.map('map',{{zoomControl:false,attributionControl:false}})
           .setView([{center[0]},{center[1]}],16);
L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png',{{maxZoom:19}}).addTo(map);

var allPts={pts_json};
var planned=L.polyline(allPts,{{color:'#7a9ab4',weight:1.5,opacity:0.5,dashArray:'6,5'}}).addTo(map);
var trail=L.polyline([],{{color:'#005898',weight:3,opacity:0.95}}).addTo(map);

L.circleMarker(allPts[0],{{radius:6,color:'#0a5c1e',fillColor:'#0a5c1e',fillOpacity:1,weight:2}})
 .addTo(map).bindTooltip('LAUNCH',{{permanent:true,direction:'right',offset:[6,0]}});

var pos=L.circleMarker(allPts[0],{{radius:7,color:'#ffffff',fillColor:'#005898',fillOpacity:1,weight:2.5}}).addTo(map);

function updateMap(t,lat,lon){{
  trail.setLatLngs(t);
  pos.setLatLng([lat,lon]);
}}
</script>
</body>
</html>"""
        self._web.setHtml(html)
        self._web.loadFinished.connect(self._on_load)

    def _on_load(self, ok):
        # setHtml() reports success even when the Leaflet <script> tag failed,
        # so probe for the library itself. Without a network (a real ground
        # station in a field) both maps would otherwise render blank forever.
        self._web.page().runJavaScript("typeof L", self._on_leaflet_probe)

    def _on_leaflet_probe(self, result):
        if result != "object":
            print("Leaflet unavailable (no network?) — using offline map")
            self._use_fallback()
            return
        self._ready = True
        if self._queued:
            self._push(*self._queued)
            self._queued = None

    def _push(self, packet, history):
        step = max(1, len(history) // 150)
        pts  = [[round(pk["lat"], 6), round(pk["lon"], 6)] for pk in history[::step]]
        js   = f"updateMap({_json.dumps(pts)},{packet['lat']},{packet['lon']});"
        self._web.page().runJavaScript(js)

    def update_data(self, packet, history):
        if self._fallback:
            self._fallback.update_data(packet, history)
            return
        # Each push is an IPC round-trip into the renderer plus a full Leaflet
        # polyline rebuild. Skip it unless the track has actually grown.
        if len(history) == self._last_idx:
            return
        self._last_idx = len(history)
        if self._ready:
            self._push(packet, history)
        else:
            self._queued = (packet, history)


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
        self._series    = []   # matching plain lists of y-values, grown in place
        self._ts        = []   # shared x-axis (mission time) values
        self._n         = 0    # number of packets already consumed

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
        self._series.append([])
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
        """
        Push the latest packet history into all registered curves.

        Only the packets appended since the last call are evaluated. The old
        implementation re-ran every accessor over the *entire* history on every
        frame — with 9 charts and 13 curves that was tens of thousands of dict
        lookups and a fresh numpy allocation 20 times a second, all to redraw
        points that had not changed.
        """
        if not history or not HAS_PG or not self._plot:
            return

        n = len(history)
        if n == self._n:
            return                      # nothing new — curves are already correct
        if n < self._n:                 # seek backwards / mission looped
            self._ts.clear()
            for s in self._series:
                s.clear()
            self._n = 0

        fresh = history[self._n:]
        self._ts.extend(pk["t"] for pk in fresh)
        for i, (fn, _) in enumerate(self._accessors):
            self._series[i].extend(fn(pk) for pk in fresh)
        self._n = n

        ts = np.asarray(self._ts)
        for i, curve in enumerate(self._curves):
            curve.setData(ts, np.asarray(self._series[i]))


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

        # ── Column 2 (rows 0–1): Live map
        traj_panel, traj_body = make_panel("Live Map (GNSS)")
        self._traj = MapWidget()
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
                            ("PACKETS RX", "")]:
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
            # 60 px could not fit "T+03:20.0" at 14 pt and clipped every
            # timestamp in the log to "T+01::".
            ts  = QLabel(); ts.setFont(mono(14)); ts.setFixedWidth(104)
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

        # Last colour pushed to each label. Qt re-parses a stylesheet and
        # repolishes the widget on every setStyleSheet() call, so the value is
        # only written when it actually changes (see update_data).
        self._css_cache = {}

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

    def _set_color(self, key, label, color):
        """setStyleSheet() only when the colour actually changed."""
        if self._css_cache.get(key) != color:
            self._css_cache[key] = color
            label.setStyleSheet(f"color: {color};")

    def update_data(self, packet, history):
        # Custom painter widgets
        self._alt_tape.update_data(packet)
        self._adi.update_data(packet)
        self._traj.update_data(packet, history)

        # Hero values
        bat_pct = max(0.0, min(100.0,
                               (packet["voltage"] - 6.5) / (8.4 - 6.5) * 100))
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
        self._elec["PACKETS RX"].setText(str(packet["packet"]).zfill(5))

        # Mission event log — show last 8 events in reverse order
        sev_color = {"ok": cs("green"), "warn": cs("amber"), "cyan": cs("cyan")}
        now = packet["t"]
        recent_events = [e for e in MISSION_EVENTS if e[0] <= now][-8:][::-1]
        for i, (ts_lbl, dot_lbl, msg_lbl) in enumerate(self._evt_rows):
            if i < len(recent_events):
                t, severity, message = recent_events[i]
                ts_lbl.setText(fmt_met(t))
                self._set_color(f"evt{i}", dot_lbl,
                                sev_color.get(severity, cs("dim")))
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
        self._set_color("dr", self._dr_val, dr_color)

        # Parachute status
        in_descent = state in ("DESCENT", "AEROBREAK_RELEASE", "IMPACT")
        self._chute1.setText("DEPLOYED" if in_descent else "STOWED")
        self._set_color("chute1", self._chute1,
                        cs("green") if in_descent else cs("dim"))

        aerobrake = state == "AEROBREAK_RELEASE"
        self._chute2.setText("DEPLOYED" if aerobrake else "STOWED")
        self._set_color("chute2", self._chute2,
                        cs("green") if aerobrake else cs("dim"))

        # Distance from the 600 m deployment trigger altitude
        diff = alt - 600.0
        self._thresh.setText(f"{diff:+.0f} m")
        self._set_color("thresh", self._thresh,
                        cs("amber") if alt < 620 else cs("dim"))


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

    def update_data(self, packet, history):
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
        map_panel, map_body = make_panel("Live Map · GNSS")
        self._traj = MapWidget()
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

    def update_data(self, packet, history):
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

    def update_data(self, packet, history):
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

    def update_data(self, packet, history, csv_path: str = ""):
        lat, lon = packet["lat"], packet["lon"]
        self._latitude.setText(f"{lat:.6f}°")
        self._longitude.setText(f"{lon:.6f}°")
        self._altitude.setText(f"{packet['gnss_alt']:.1f} m")
        self._gnss_time.setText(packet["gnss_time"])

        # Distance and bearing from ground station using haversine formula
        dist, brg = haversine(GROUND_STATION[0], GROUND_STATION[1], lat, lon)
        self._distance.setText(f"{dist:.3f}")
        self._bearing.setText(f"{brg:.1f}")

        self._traj.update_data(packet, history)

        if csv_path:
            self._csv_path_lbl.setText(csv_path)


# ═══════════════════════════════════════════════════════════════════
# TOP BAR
# Fixed 56 px strip at the top.
# Layout: [logo] [state pill] | [team identity] ··· [REC] [MET] | [PKT] | [LINK]
# ═══════════════════════════════════════════════════════════════════

class TopBar(QWidget):
    """
    Header bar — one unified dark strip.  Four zones separated by vertical lines:
      LEFT    — logo · large state badge · team identity
      CENTRE  — (stretch)
      RIGHT   — REC · MET clock · PKT received/expected · link dot
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setFixedHeight(72)

        # Last values pushed to the styled labels — see update_data(). Rewriting
        # a stylesheet forces Qt to re-parse it and repolish the widget, which
        # is far too expensive to do on every 20 Hz tick.
        self._last_state    = None
        self._last_pkt_css  = None
        self._last_link_css = None

        hl = QHBoxLayout(self)
        hl.setContentsMargins(16, 0, 20, 0)
        hl.setSpacing(0)

        # ── Logo ────────────────────────────────────────────────────
        logo_lbl = QLabel()
        logo_lbl.setFixedSize(52, 52)
        logo_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        pix = _load_logo(48)
        if pix is not None:
            logo_lbl.setPixmap(pix)   # falls back to empty label if logo is missing

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

        mission_id = QLabel(TEAM_ID)
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

        # ── Packet counter — received / expected ─────────────────────
        _vl1 = vline(); hl.addWidget(_vl1)
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

    def set_met(self, t: float):
        """Mission clock only — cheap enough to run on every 20 Hz tick."""
        self._met.setText(fmt_met(t))

    def update_data(self, packet, t, recording: bool = False,
                    pkt_expected: int = 0, pkt_received: int = 0):
        state = packet["state"]

        # State badge — large pill, color changes per state. Only restyled on an
        # actual state transition (a handful of times per flight).
        if state != self._last_state:
            self._last_state = state
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

        self.set_met(t)

        # Packet counter — rows logged / packets the CanSat reports sending
        rcvd = pkt_received or packet["packet"]
        self._pkt.setText(f"{rcvd:05d} / {pkt_expected:05d}")
        pkt_loss = min(1.0, max(0.0, (pkt_expected - rcvd) / max(1, pkt_expected)))
        if pkt_loss > 0.15:
            pkt_css = cs("red")
        elif pkt_loss > 0.05:
            pkt_css = cs("amber")
        else:
            pkt_css = cs("green")
        if pkt_css != self._last_pkt_css:
            self._last_pkt_css = pkt_css
            self._pkt.setStyleSheet(f"color: {pkt_css};")

        # Link health dot — based on packet loss only (no RSSI telemetry from
        # the CanSat, so link health is judged purely by packets received vs
        # expected).
        if pkt_loss < 0.05:
            link_col = cs("green")
        elif pkt_loss < 0.15:
            link_col = cs("amber")
        else:
            link_col = cs("red")
        if link_col != self._last_link_css:
            self._last_link_css = link_col
            self._link_dot.setStyleSheet(f"color: {link_col};")

    def apply_theme(self):
        # Restyling the bar drops the per-label styles set in update_data(),
        # so drop their caches too and let the next tick rewrite them.
        self._last_state    = None
        self._last_pkt_css  = None
        self._last_link_css = None
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

        # Gear — the only way to reach playback speed, mission restart and the
        # ground-station coordinates used for the recovery bearing. The button
        # had been removed while SettingsDialog and the settings_clicked signal
        # stayed behind, leaving the whole dialog unreachable.
        settings_btn = QPushButton("⚙")
        settings_btn.setFixedSize(44, 40)
        settings_btn.setObjectName("settings_btn")
        settings_btn.setToolTip("Settings — playback speed, ground-station position")
        settings_btn.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
        settings_btn.clicked.connect(self.settings_clicked.emit)
        vl.addWidget(settings_btn)

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

        id_lbl = QLabel(TEAM_ID)
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
        for label, cmd in [("Boot", "CMD:BOOT"), ("Set Time", "CMD:SET_TIME")]:
            btn = QPushButton(label)
            btn.setFixedHeight(30)
            btn.clicked.connect(lambda _, c=cmd: self.command_sent.emit(c))
            hl.addWidget(btn)

        hl.addWidget(vline())

        # Sensor calibration — command the CanSat to zero its gyros, barometric
        # altitude and accelerometer while it sits on the launch pad
        # (§6.1.vi / requirements-compliance item 35).
        cal_btn = QPushButton("Calibrate")
        cal_btn.setFixedHeight(30)
        cal_btn.setToolTip("Uplink CMD:CAL — zero gyro, baro & accelerometer on the launch pad")
        cal_btn.clicked.connect(lambda: self.command_sent.emit("CMD:CAL"))
        hl.addWidget(cal_btn)

        hl.addWidget(vline())

        # CX toggle — commands the CanSat to start/stop transmitting telemetry
        # and drives CSV recording. Defaults to OFF so nothing is recorded until
        # the operator commands transmission (§6.1.v: no telemetry until
        # commanded). Previously it read "CX ON" but never actually started
        # recording, since setChecked() ran before the signal was connected.
        self._cx_btn = QPushButton("CX OFF")
        self._cx_btn.setFixedHeight(30)
        self._cx_btn.setCheckable(True)
        self._cx_btn.setChecked(False)
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

        # Playback controls (hidden for PDR screenshots)
        self._play_btn = QPushButton("▶ Play")
        self._play_btn.setFixedSize(80, 30)
        self._play_btn.clicked.connect(self.play_toggled.emit)
        self._play_btn.hide()

        self._scrubber = QSlider(Qt.Orientation.Horizontal)
        self._scrubber.setRange(0, 1000)
        self._scrubber.setMinimumWidth(200)
        self._scrubber.sliderMoved.connect(
            lambda v: self.seek_requested.emit(v / 1000.0 * logic.MISSION_DURATION))
        self._scrubber.hide()

        self._clock = QLabel("T+00:00.0")
        self._clock.setFont(mono(15))
        self._clock.setFixedWidth(100)
        self._clock.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self._clock.hide()

    def _on_cx_toggle(self, on: bool):
        self._cx_btn.setText("CX ON" if on else "CX OFF")
        self.command_sent.emit("CX:ON" if on else "CX:OFF")

    def update_data(self, packet, t, playing: bool, speed: float):
        # The playback controls are hidden (see __init__), so there is nothing
        # to repaint — skip the work rather than formatting text nobody sees.
        if self._play_btn.isHidden():
            return
        self._play_btn.setText("❚❚ Pause" if playing else "▶ Play")
        # Update scrubber without triggering sliderMoved
        self._scrubber.blockSignals(True)
        self._scrubber.setValue(int(t / logic.MISSION_DURATION * 1000))
        self._scrubber.blockSignals(False)
        self._clock.setText(fmt_met(t))

    def _export_csv(self):
        """
        Manual export of all received data to a user-chosen file.

        Defaults to the spec-mandated name Flight_<TEAM_ID>.csv (§6.3.iii) and
        writes the exact §6.3 field order via the shared telemetry_row() helper,
        so this file and the live recording are always byte-for-byte consistent.
        """
        path, _ = QFileDialog.getSaveFileName(
            self, "Export CSV", CSV_FILENAME, "CSV Files (*.csv)")
        if not path:
            return
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(TELEMETRY_HEADER)
            for pk in logic.MISSION_DATA[:SIM.idx + 1]:
                writer.writerow(telemetry_row(pk))

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
        self.setWindowTitle(f"CanSat Ground Station · Team Kalpana  [{TEAM_ID}]")
        self.resize(1400, 860)
        self.setMinimumSize(1024, 680)

        # Session state
        self._settings       = None    # SettingsDialog instance (lazy)
        self._recording      = False
        self._csv_file       = None
        self._csv_writer_obj = None
        self._csv_path       = ""
        self._nav_open       = False   # whether NavOverlay is visible

        # Refresh gating — see _on_tick(). _needs_repaint forces one full refresh
        # after a tab switch, so a newly shown tab paints immediately instead of
        # waiting for the next packet.
        self._last_pkt_idx   = -1
        self._needs_repaint  = True

        self._build_expected_counts()
        self._build_ui()
        self._connect_signals()
        self.apply_theme()
        SIM.updated.connect(self._on_tick)

    def _build_expected_counts(self):
        """
        Pre-compute, for each packet index, how many packets the CanSat should
        have sent by that point.

        The CanSat stamps every packet with its own PACKET_COUNT. If that value
        jumps by more than one between two packets we hold, the difference is
        exactly what the radio link dropped. Summing those jumps gives a real
        packet-loss figure; comparing the raw counter against elapsed seconds
        (what the code used to do) does not, because nothing guarantees one
        packet per second.
        """
        self._expected_at = []
        expected, prev = 0, None
        for pk in logic.MISSION_DATA:
            count     = pk["packet"]
            expected += 1 if prev is None else max(1, count - prev)
            prev      = count
            self._expected_at.append(expected)
        if not self._expected_at:
            self._expected_at = [1]

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
        for tab in [self._tab_telemetry, self._tab_graphs,
                    self._tab_location,  self._tab_live]:
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
        }

    def _connect_signals(self):
        self._sidebar.expand_toggled.connect(self._toggle_nav)
        self._sidebar.settings_clicked.connect(self._open_settings)
        self._nav_overlay.tab_changed.connect(self._switch_tab)
        self._nav_overlay.closed.connect(self._close_nav)
        self._dock.play_toggled.connect(SIM.toggle_play)
        self._dock.seek_requested.connect(SIM.seek)
        self._dock.command_sent.connect(self._on_command)

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
        self._needs_repaint = True   # paint the newly visible tab on the next tick

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
        # Timestamp keeps each session's file distinct so a re-launch never
        # clobbers a completed flight; the team-id suffix keeps it identifiable.
        # The graded, exactly-named Flight_<TEAM_ID>.csv is produced on demand
        # via the CSV Export button (§6.3.iii).
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self._csv_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            f"Flight_{timestamp}_{TEAM_ID}.csv")
        self._csv_file       = open(self._csv_path, "w", newline="")
        self._csv_writer_obj = csv.writer(self._csv_file)
        self._csv_writer_obj.writerow(TELEMETRY_HEADER)
        self._recording = True
        MISSION_EVENTS.append((SIM.t if SIM else 0.0, "ok", "Recording started"))
        print(f"Recording to {self._csv_path}")

    def _stop_recording(self):
        if not self._recording:
            return
        if self._csv_file:
            self._csv_file.flush()
            self._csv_file.close()
            self._csv_file = None
        self._csv_writer_obj = None
        self._recording = False
        MISSION_EVENTS.append((SIM.t if SIM else 0.0, "warn", "Recording stopped"))

    def closeEvent(self, event):
        """Close the recording cleanly — quitting mid-flight used to leave the
        last buffered rows unwritten because the file was never closed."""
        self._stop_recording()
        super().closeEvent(event)

    # ──────────────────────────────────────────────────────────────
    # Main update loop — fires 20× per second via SimState.updated
    # ──────────────────────────────────────────────────────────────

    def _on_tick(self):
        """
        The timer fires at 20 Hz, but telemetry arrives far slower — 1 packet per
        second from trial_data.csv, 10/s from the simulator. Everything below the
        clock therefore only runs when a genuinely new packet is available, or
        when the visible tab changed and needs a first paint. Previously the full
        chart/map/painter refresh ran on all 20 ticks, redrawing identical data
        19 times out of 20.
        """
        # The MET clock is the only thing that must move on every tick.
        self._topbar.set_met(SIM.t)

        pkt_idx = SIM.idx
        if pkt_idx == self._last_pkt_idx and not self._needs_repaint:
            return
        new_packet          = pkt_idx != self._last_pkt_idx
        self._last_pkt_idx  = pkt_idx
        self._needs_repaint = False

        pk   = SIM.packet
        hist = SIM.history

        # Write one row per packet if recording is active (spec §6.3 field order).
        # Gated on new_packet: writing on every tick produced ~20 duplicate rows
        # per packet in the recorded CSV and flushed the file 20 times a second.
        # The flush stays — once per packet is cheap, and it keeps the recording
        # intact if the ground station loses power mid-flight.
        if new_packet and self._recording and self._csv_writer_obj:
            self._csv_writer_obj.writerow(telemetry_row(pk))
            self._csv_file.flush()

        # Packet loss comes from gaps in the CanSat's own PACKET_COUNT sequence
        # (see _build_expected_counts). Deriving "expected" from wall-clock time
        # instead assumed exactly one packet per second, which is false for
        # trial_data.csv and pinned the link indicator to red for every flight.
        self._topbar.update_data(pk, SIM.t, recording=self._recording,
                                 pkt_expected=self._expected_at[pkt_idx],
                                 pkt_received=pkt_idx + 1)
        self._dock.update_data(pk, SIM.t, SIM.playing, SIM.speed)

        # Only update the currently visible tab (saves CPU)
        idx = self._stack.currentIndex()
        if   idx == 0: self._tab_telemetry.update_data(pk, hist)
        elif idx == 1: self._tab_graphs.update_data(pk, hist)
        elif idx == 2: self._tab_location.update_data(pk, hist)
        elif idx == 3: self._tab_live.update_data(pk, hist)

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

    _warn_if_cloud_evicted()

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
