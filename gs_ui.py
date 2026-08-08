"""
CanSat Ground Station — display layer.  All the Qt code lives here.

Structure:
  Theme          — color palette + font helpers
  TelemetryFeed  — drains the receiver's packet queue into Qt (needs signals)
  Widget helpers — make_panel(), make_stat(), vline()
  Painter widgets— AltitudeTapeWidget, AttitudeIndicatorWidget, TrajectoryMapWidget
  Chart          — pyqtgraph time-series chart with fallback plain label
  Tabs           — TelemetryTab, GraphsTab, LocationTab, RecoveryTab
  Shell          — TopBar, Sidebar, NavOverlay, CommandDock, SettingsDialog
  MainWindow     — assembles everything; owns the link, recording and commands
  main()         — entry point called by ground_station_simple.py

Every tab exposes update_data(data), where data is the MissionData store from
gs_logic.  Only the visible tab is refreshed.
"""

import sys
import os
import queue
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
    """
    Print an actionable warning when the venv is being streamed from iCloud.

    macOS only.  st_flags is a BSD stat field that does not exist on Windows,
    where reading it raises AttributeError — so the whole check is skipped on
    any platform without chflags rather than relying on the loop below to
    swallow it.
    """
    if not hasattr(os, "chflags"):
        return
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
            except (OSError, AttributeError):
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
    QComboBox, QDialog, QFormLayout, QDoubleSpinBox, QSpinBox,
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
    GROUND_STATION, fmt_met, haversine, compass_point, attitude_from_accel,
    battery_percent, TEAM_ID, DEFAULT_BAUD, DESCENT_STATES,
    SECONDARY_TRIGGER_ALT, VOLTAGE_WARN,
    LINK_OFFLINE, LINK_LIVE, LINK_STALE, LINK_LOST,
)

# Resolved once at import time; used by Sidebar and TopBar for the team logo
LOGO_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Team-Kalpana-Logo.png")

# Where the flight CSV is written
APP_DIR = os.path.dirname(os.path.abspath(__file__))


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
# TELEMETRY FEED
# The one place where data crosses from the reader thread into Qt.
# ═══════════════════════════════════════════════════════════════════

class TelemetryFeed(QObject):
    """
    Drains the receiver's packet queue on a 10 Hz timer.

    Every packet found goes into MissionData, then one signal tells the window
    to redraw.  Collecting on a timer instead of signalling per packet keeps
    the redraw rate bounded no matter how fast telemetry arrives, and keeps
    all Qt work on the UI thread where it belongs.
    """

    updated = pyqtSignal(bool)   # True when at least one new packet arrived

    def __init__(self, receiver: logic.TelemetryReceiver, data: logic.MissionData):
        super().__init__()
        self.receiver = receiver
        self.data     = data

        timer = QTimer(self)
        timer.setInterval(100)
        timer.timeout.connect(self._drain)
        timer.start()
        self._timer = timer

    def _drain(self):
        got_packet = False
        while True:
            try:
                pk = self.receiver.packets.get_nowait()
            except queue.Empty:
                break
            self.data.add(pk)
            got_packet = True
        self.updated.emit(got_packet)


# Created in main() once QApplication exists
RECEIVER = None   # logic.TelemetryReceiver
DATA     = None   # logic.MissionData
FEED     = None   # TelemetryFeed
SIMULATOR = None  # logic.SimulationSender


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
# Each widget has update_data(...) which stores data and calls
# self.update() to schedule a repaint.
# ═══════════════════════════════════════════════════════════════════

class AltitudeTapeWidget(QWidget):
    """
    Vertical altitude scale with a moving rocket icon.

    Shows:
     - A vertical rail with tick marks every 100 m
     - Apogee marker at the highest altitude received so far
     - Rocket icon that moves up/down and flips during descent
     - Current altitude readout next to the rocket

    The rail rescales as the flight climbs, so it always has headroom: the
    scale is the apex rounded up to the next 100 m, never less than 800 m.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(130, 240)
        self._apex    = 0.0
        self._max_alt = 800
        self._alt     = 0.0
        self._state   = "BOOT"

    def update_data(self, packet, apex: float):
        self._apex    = apex
        self._max_alt = max(800, int((apex * 1.05) // 100 + 1) * 100)
        self._alt     = packet["altitude"]
        self._state   = packet["state"]
        self.update()   # triggers paintEvent

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        W, H    = self.width(), self.height()
        MAX_ALT = self._max_alt
        PAD     = 20
        rail_x  = W // 2

        def alt_to_y(a):
            """Convert altitude in metres to a pixel y-coordinate."""
            return PAD + (1 - min(max(a, 0.0), MAX_ALT) / MAX_ALT) * (H - 2 * PAD)

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

        # Secondary-deployment trigger altitude — the number the operator is
        # watching for during descent.
        trig_y = int(alt_to_y(SECONDARY_TRIGGER_ALT))
        p.setPen(QPen(c("cyan"), 1, Qt.PenStyle.DashLine))
        p.drawLine(rail_x - 24, trig_y, rail_x + 24, trig_y)

        # Apogee marker — amber line, labelled on the LEFT of the rail so it
        # never prints on top of the live altitude readout.
        if self._apex > 0:
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
        if self._state in DESCENT_STATES:
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

    Roll and pitch come from the accelerometer — see
    gs_logic.attitude_from_accel() for the axis convention.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(160, 160)
        self._roll  = 0.0
        self._pitch = 0.0

    def update_data(self, packet):
        self._roll, self._pitch = attitude_from_accel(packet)
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
    Offline top-down GPS ground track, drawn on a blank grid.

    Used when there is no internet for map tiles — a real ground station in a
    field usually has none.  The view auto-fits the track received so far:

     - Solid trail of where the cansat has been
     - Launch site marker (green cross) at the first fix
     - Current position (glowing dot)
     - Lat/lon readout at bottom-left
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 180)
        self._history = []

    def update_data(self, packet, history):
        self._history = history
        self.update()

    def _bounds(self):
        """
        Lat/lon box around the track, with a minimum span so a stationary
        CanSat on the pad does not divide by zero or zoom to infinity.
        """
        lats = [pk["lat"] for pk in self._history]
        lons = [pk["lon"] for pk in self._history]
        min_lat, max_lat = min(lats), max(lats)
        min_lon, max_lon = min(lons), max(lons)
        pad = 0.0002
        span_lat = max(max_lat - min_lat, pad) * 1.2
        span_lon = max(max_lon - min_lon, pad) * 1.2
        mid_lat  = (min_lat + max_lat) / 2
        mid_lon  = (min_lon + max_lon) / 2
        return (mid_lat - span_lat / 2, span_lat,
                mid_lon - span_lon / 2, span_lon)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        W, H = self.width(), self.height()
        p.fillRect(0, 0, W, H, c("bg2"))

        if not self._history:
            p.setPen(c("dim"))
            p.setFont(sans(10))
            p.drawText(10, H // 2, "Awaiting GNSS fix…")
            return

        base_lat, span_lat, base_lon, span_lon = self._bounds()

        def to_xy(lat, lon):
            """Convert GPS coordinates to pixel position."""
            return QPointF((lon - base_lon) / span_lon * W,
                           (1.0 - (lat - base_lat) / span_lat) * H)

        # Actual trail — sampled to at most 200 points so a long flight does
        # not slow the repaint down.
        step  = max(1, len(self._history) // 200)
        trail = QPolygonF([to_xy(pk["lat"], pk["lon"])
                           for pk in self._history[::step]])
        if trail.count() >= 2:
            p.setPen(QPen(c("cyan"), 2))
            p.drawPolyline(trail)

        # Launch site cross — the first fix we ever received
        launch_pt = to_xy(self._history[0]["lat"], self._history[0]["lon"])
        p.setPen(QPen(c("green"), 2))
        p.drawLine(QPointF(launch_pt.x() - 5, launch_pt.y()),
                   QPointF(launch_pt.x() + 5, launch_pt.y()))
        p.drawLine(QPointF(launch_pt.x(), launch_pt.y() - 5),
                   QPointF(launch_pt.x(), launch_pt.y() + 5))
        p.setFont(mono(12))
        p.setPen(c("green"))
        p.drawText(int(launch_pt.x()) + 6, int(launch_pt.y()), "LAUNCH")

        # Current position — glowing dot
        packet  = self._history[-1]
        curr_pt = to_xy(packet["lat"], packet["lon"])
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
        p.drawText(4, H - 14, f"LAT {packet['lat']:.5f}°")
        p.drawText(4, H - 5,  f"LON {packet['lon']:.5f}°")


class MapWidget(QWidget):
    """
    Tile-based GPS map using Leaflet.js + OpenStreetMap inside a QWebEngineView.
    Falls back to TrajectoryMapWidget if PyQt6-WebEngine is unavailable or
    there is no network for the tiles.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self._vl = QVBoxLayout(self)
        self._vl.setContentsMargins(0, 0, 0, 0)
        self._ready    = False
        self._queued   = None
        self._web      = None
        self._fallback = None
        self._sent     = 0        # number of track points already pushed

        if not HAS_WEBENGINE:
            self._use_fallback()

    def showEvent(self, event):
        """
        Build the QWebEngineView the first time this map actually becomes
        visible.  Several MapWidgets exist across the tabs; creating them all
        up front would spawn a renderer process each and fetch Leaflet twice
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
        """
        Load an empty map centred on the ground station.

        Nothing about the flight path is known in advance any more, so the map
        starts blank and the launch marker is planted on the first GNSS fix,
        at which point the view re-centres on the CanSat.
        """
        center = [GROUND_STATION[0], GROUND_STATION[1]]

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
           .setView([{center[0]},{center[1]}],15);
L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png',{{maxZoom:19}}).addTo(map);

var trail=L.polyline([],{{color:'#005898',weight:3,opacity:0.95}}).addTo(map);
var pos=null;

function updateMap(track,lat,lon){{
  trail.setLatLngs(track);
  if(pos===null){{
    // First fix — plant the launch marker and fly to the CanSat.
    L.circleMarker([lat,lon],{{radius:6,color:'#0a5c1e',fillColor:'#0a5c1e',
      fillOpacity:1,weight:2}}).addTo(map)
      .bindTooltip('LAUNCH',{{permanent:true,direction:'right',offset:[6,0]}});
    pos=L.circleMarker([lat,lon],{{radius:7,color:'#ffffff',fillColor:'#005898',
      fillOpacity:1,weight:2.5}}).addTo(map);
    map.setView([lat,lon],17);
  }}
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
        # station in a field) the map would otherwise render blank forever.
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
        if len(history) == self._sent:
            return
        self._sent = len(history)
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
        ch = Chart("Altitude", "m", color=cs("cyan"))
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

        Only the packets appended since the last call are evaluated, so the
        cost per redraw is proportional to what actually arrived rather than to
        the whole flight.
        """
        if not history or not HAS_PG or not self._plot:
            return

        n = len(history)
        if n == self._n:
            return                      # nothing new — curves are already correct
        if n < self._n:                 # history was cleared (new session)
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

        imu_panel, imu_body = make_panel("IMU · Accelerometer & Gyro")
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

    def update_data(self, data):
        packet  = data.current
        history = data.packets

        # Custom painter widgets
        self._alt_tape.update_data(packet, data.apex)
        self._adi.update_data(packet)
        self._traj.update_data(packet, history)

        # Hero values
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
        self._elec["ESTIMATED"].setText(f"{battery_percent(packet['voltage']):.0f}")
        self._elec["PACKETS RX"].setText(str(len(history)).zfill(5))

        # Mission event log — newest first
        sev_color = {"ok": cs("green"), "warn": cs("amber"), "cyan": cs("cyan")}
        recent_events = data.recent_events(len(self._evt_rows))
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
        vel   = packet["velocity"]
        state = packet["state"]
        alt   = packet["altitude"]

        # Descent rate — color shows severity
        dr_color = cs("red") if vel < -8 else cs("amber") if vel < -4 else cs("green")
        self._dr_val.setText(f"{vel:+.1f} m/s")
        self._set_color("dr", self._dr_val, dr_color)

        # Parachute status
        in_descent = state in DESCENT_STATES
        self._chute1.setText("DEPLOYED" if in_descent else "STOWED")
        self._set_color("chute1", self._chute1,
                        cs("green") if in_descent else cs("dim"))

        aerobrake = state == "AEROBREAK_RELEASE"
        self._chute2.setText("DEPLOYED" if aerobrake else "STOWED")
        self._set_color("chute2", self._chute2,
                        cs("green") if aerobrake else cs("dim"))

        # Distance from the 600 m deployment trigger altitude
        diff = alt - SECONDARY_TRIGGER_ALT
        self._thresh.setText(f"{diff:+.0f} m")
        self._set_color("thresh", self._thresh,
                        cs("amber") if alt < SECONDARY_TRIGGER_ALT + 20 else cs("dim"))


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
            ("Altitude",    "m",    cs("cyan"),  lambda p: p["altitude"],  None, None),
            ("Pressure",    "hPa",  "#7878f0",   lambda p: p["pressure"],  None, None),
            ("Voltage",     "V",    cs("amber"), lambda p: p["voltage"],   6.0,  8.6),
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

        # Reference lines the operator is watching for
        self._charts[0].add_threshold(SECONDARY_TRIGGER_ALT, cs("amber"), "600m trigger")
        self._charts[2].add_threshold(VOLTAGE_WARN, cs("red"), "low battery")

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

        spin_ch = Chart("Spin Rate", "°/s", cs("cyan"))
        spin_ch.add_line(lambda p: p["gyro_spin"], cs("cyan"))
        spin_ch.setMinimumHeight(180)
        spin_panel, spin_body = make_panel("Mechanical Spin Rate")
        spin_body.addWidget(spin_ch)
        grid.addWidget(spin_panel, 2, 2)
        self._charts.append(spin_ch)

        for col in range(3):
            grid.setColumnStretch(col, 1)

    def update_data(self, data):
        for ch in self._charts:
            ch.update_data(data.packets)


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
        # Roll and pitch are angles derived from the accelerometer; the gyro
        # gives rates, not an absolute heading, so yaw is shown as a rate.
        for name, unit in [("ROLL", "°"), ("PITCH", "°"),
                            ("YAW RATE", "°/s"), ("SPIN RATE", "°/s")]:
            row, val = make_stat(name, unit)
            orient_body.addWidget(row)
            self._orient[name] = val
        orient_body.addStretch()
        right.addWidget(orient_panel, 1)
        hl.addLayout(right, 2)

    def update_data(self, data):
        packet = data.current
        self._traj.update_data(packet, data.packets)
        self._adi.update_data(packet)
        self._loc["LATITUDE"].setText(f"{packet['lat']:.5f}°")
        self._loc["LONGITUDE"].setText(f"{packet['lon']:.5f}°")
        self._loc["ALTITUDE"].setText(f"{packet['gnss_alt']:.1f}")
        self._loc["SATS"].setText(str(packet["sats"]))

        roll, pitch = attitude_from_accel(packet)
        self._orient["ROLL"].setText(f"{roll:.1f}")
        self._orient["PITCH"].setText(f"{pitch:.1f}")
        self._orient["YAW RATE"].setText(f"{packet['gyro_y']:.1f}")
        self._orient["SPIN RATE"].setText(f"{packet['gyro_spin']:.0f}")


# ═══════════════════════════════════════════════════════════════════
# TAB: RECOVERY  (auto-shown when state reaches IMPACT)
# Displays last-known position, distance + bearing from the ground station,
# the ground track and the path of the flight CSV for the judges.
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

        dist_panel, dist_body = make_panel("Walk From Ground Station")
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

        # Compass point — the direction you actually walk in
        self._heading = QLabel("—")
        self._heading.setFont(mono(20))
        self._heading.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._heading.setStyleSheet(f"color: {cs('cyan')}; padding: 6px;")
        dist_body.addWidget(self._heading)
        left_vl.addWidget(dist_panel)

        csv_panel, csv_body = make_panel("Flight CSV  (hand this to the judges)")
        self._csv_path_lbl = QLabel("Not recording")
        self._csv_path_lbl.setFont(mono(9))
        self._csv_path_lbl.setWordWrap(True)
        self._csv_path_lbl.setContentsMargins(8, 6, 8, 6)
        csv_body.addWidget(self._csv_path_lbl)
        csv_body.addStretch()
        left_vl.addWidget(csv_panel)

        # ── Right column: beacon reminder + map ─────────────────────
        right_vl = QVBoxLayout()
        hl.addLayout(right_vl, 2)

        beacon_panel, beacon_body = make_panel("Audio Beacon")
        self._beacon_lbl = QLabel("LISTEN FOR THE BEACON")
        self._beacon_lbl.setFont(mono(16))
        self._beacon_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._beacon_lbl.setStyleSheet(f"color: {cs('green')};")
        beacon_body.addWidget(self._beacon_lbl)
        beacon_body.addStretch()
        right_vl.addWidget(beacon_panel, 1)

        map_panel, map_body = make_panel("Ground Track · Landing Area")
        self._traj = TrajectoryMapWidget()
        map_body.addWidget(self._traj)
        right_vl.addWidget(map_panel, 3)

    def update_data(self, data, csv_path: str = ""):
        packet   = data.current
        lat, lon = packet["lat"], packet["lon"]
        self._latitude.setText(f"{lat:.6f}°")
        self._longitude.setText(f"{lon:.6f}°")
        self._altitude.setText(f"{packet['gnss_alt']:.1f} m")
        self._gnss_time.setText(packet["gnss_time"])

        # Distance and bearing from ground station using the haversine formula
        dist, brg = haversine(GROUND_STATION[0], GROUND_STATION[1], lat, lon)
        self._distance.setText(f"{dist:.3f}")
        self._bearing.setText(f"{brg:.1f}")
        self._heading.setText(f"WALK {compass_point(brg)}  ·  {dist * 1000:.0f} m")

        self._traj.update_data(packet, data.packets)

        self._csv_path_lbl.setText(csv_path or "Not recording")


# ═══════════════════════════════════════════════════════════════════
# TOP BAR
# Fixed strip at the top.
# Layout: [logo] [state pill] | [team identity] ··· [MET] | [PKT] | [LINK]
# ═══════════════════════════════════════════════════════════════════

class TopBar(QWidget):
    """
    Header bar — one unified strip.  Zones separated by vertical lines:
      LEFT    — logo · large state badge · team identity
      CENTRE  — (stretch)
      RIGHT   — MET clock · packets received/lost · link health
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setFixedHeight(72)

        # Last values pushed to the styled labels — see update_data(). Rewriting
        # a stylesheet forces Qt to re-parse it and repolish the widget, which
        # is far too expensive to do on every tick.
        self._last_state    = None
        self._last_pkt_css  = None
        self._last_link     = None

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
        self._state_lbl = QLabel("NO DATA")
        self._state_lbl.setFont(mono(16))
        f = self._state_lbl.font()
        f.setBold(True)
        self._state_lbl.setFont(f)
        self._state_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._state_lbl.setContentsMargins(16, 6, 16, 6)
        self._state_lbl.setObjectName("state_badge")
        hl.addWidget(self._state_lbl)
        hl.addSpacing(20)

        # ── Team name + mission ID ──────────────────────────────────
        hl.addWidget(vline())
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

        # ── Packet counter — received / lost ─────────────────────────
        hl.addWidget(vline())
        hl.addSpacing(18)

        pkt_box = QWidget()
        pkt_box.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, False)
        pkt_vl  = QVBoxLayout(pkt_box)
        pkt_vl.setContentsMargins(0, 0, 0, 0)
        pkt_vl.setSpacing(2)
        pkt_hdr = QLabel("PACKETS  RX / LOST")
        pkt_hdr.setFont(sans(20))
        pkt_hdr.setObjectName("section_hdr")
        self._pkt = QLabel("00000 / 0")
        self._pkt.setFont(mono(12))
        pkt_vl.addWidget(pkt_hdr)
        pkt_vl.addWidget(self._pkt)
        hl.addWidget(pkt_box)
        hl.addSpacing(18)

        # ── Link health ──────────────────────────────────────────────
        hl.addWidget(vline())
        hl.addSpacing(18)

        link_box = QWidget()
        link_box.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, False)
        link_vl  = QVBoxLayout(link_box)
        link_vl.setContentsMargins(0, 0, 0, 0)
        link_vl.setSpacing(2)
        link_hdr = QLabel("LINK")
        link_hdr.setFont(sans(20))
        link_hdr.setObjectName("section_hdr")
        self._link_lbl = QLabel("OFFLINE")
        self._link_lbl.setFont(mono(13))
        link_vl.addWidget(link_hdr)
        link_vl.addWidget(self._link_lbl)
        hl.addWidget(link_box)

    def set_met(self, t: float):
        """Mission clock only — cheap enough to run on every tick."""
        self._met.setText(fmt_met(t))

    def update_data(self, data, receiver):
        # ── Flight state badge ───────────────────────────────────────
        state = data.current["state"] if data.current else "NO DATA"
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

        self.set_met(data.met())

        # ── Packet counter — what arrived vs what the link lost ──────
        received = receiver.parser.accepted
        dropped  = receiver.parser.dropped
        self._pkt.setText(f"{received:05d} / {dropped}")
        loss = dropped / max(1, received + dropped)
        pkt_css = cs("red") if loss > 0.15 else cs("amber") if loss > 0.05 else cs("green")
        if pkt_css != self._last_pkt_css:
            self._last_pkt_css = pkt_css
            self._pkt.setStyleSheet(f"color: {pkt_css};")

        # ── Link health — from the receiver, not from the data ───────
        status = receiver.link_status()
        if status != self._last_link:
            self._last_link = status
            colors = {
                LINK_LIVE:    cs("green"),
                LINK_STALE:   cs("amber"),
                LINK_LOST:    cs("red"),
                LINK_OFFLINE: cs("faint"),
            }
            self._link_lbl.setText(status)
            self._link_lbl.setStyleSheet(f"color: {colors.get(status, cs('faint'))};")

    def apply_theme(self):
        # Restyling the bar drops the per-label styles set in update_data(),
        # so drop their caches too and let the next tick rewrite them.
        self._last_state   = None
        self._last_pkt_css = None
        self._last_link    = None
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
        """)


# ═══════════════════════════════════════════════════════════════════
# SIDEBAR  (collapsed strip, always 56 px)
# ≡ button opens NavOverlay for full tab names.
# ═══════════════════════════════════════════════════════════════════

class Sidebar(QWidget):
    """
    Minimal 56 px strip — hamburger ≡ opens NavOverlay, settings gear at the
    bottom.  Navigation is handled entirely through the NavOverlay.
    """
    expand_toggled   = pyqtSignal()
    settings_clicked = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedWidth(56)
        vl = QVBoxLayout(self)
        vl.setContentsMargins(6, 12, 6, 10)
        vl.setSpacing(8)

        expand_btn = QPushButton("≡")
        expand_btn.setFixedSize(44, 36)
        expand_btn.setObjectName("hamburger_btn")
        expand_btn.setToolTip("Open navigation  (or press 1-4)")
        expand_btn.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
        expand_btn.clicked.connect(self.expand_toggled.emit)
        vl.addWidget(expand_btn)

        vl.addStretch()

        settings_btn = QPushButton("⚙")
        settings_btn.setFixedSize(44, 40)
        settings_btn.setObjectName("settings_btn")
        settings_btn.setToolTip("Settings — radio baud rate, ground-station position")
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
# ═══════════════════════════════════════════════════════════════════

class NavOverlay(QWidget):
    """
    Floating navigation panel.  Opens when ≡ is pressed; closes when:
      - a tab button is clicked (auto-close + navigate)
      - the dim overlay behind it is clicked
      - Escape is pressed
      - ≡ is pressed again (toggle)
    """
    tab_changed = pyqtSignal(str)
    closed      = pyqtSignal()

    _SECTIONS = [
        ("VIEWS", [
            ("telemetry", "📊", "Telemetry Data"),
            ("graphs",    "📈", "Graphs"),
            ("location",  "🗺 ", "Location & Track"),
        ]),
        ("AFTER LANDING", [
            ("recovery", "📍", "Recovery"),
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

        # ── Header — team identity ───────────────────────────────────
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

        div = QFrame(); div.setFrameShape(QFrame.Shape.HLine); div.setFixedHeight(1)
        vl.addWidget(div)
        vl.addSpacing(8)

        # ── Grouped tab buttons ──────────────────────────────────────
        self._btns = {}
        for section_name, tabs in self._SECTIONS:
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
# Fixed bar at the bottom.  Left: the radio link.  Middle: uplink commands.
# Every button emits a signal; MainWindow decides what to do with it.
# ═══════════════════════════════════════════════════════════════════

class CommandDock(QWidget):
    connect_requested    = pyqtSignal(str)    # port name
    disconnect_requested = pyqtSignal()
    ports_refreshed      = pyqtSignal()
    command_sent         = pyqtSignal(str)    # key into logic.COMMANDS

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(76)
        hl = QHBoxLayout(self)
        hl.setContentsMargins(12, 0, 12, 0)
        hl.setSpacing(6)

        # ── Radio link ───────────────────────────────────────────────
        link_lbl = QLabel("XBee")
        link_lbl.setFont(sans(10))
        hl.addWidget(link_lbl)

        self._port_box = QComboBox()
        self._port_box.setFixedHeight(30)
        self._port_box.setMinimumWidth(260)
        self._port_box.setToolTip("USB port the XBee is plugged into")
        hl.addWidget(self._port_box)

        rescan_btn = QPushButton("⟳")
        rescan_btn.setFixedSize(32, 30)
        rescan_btn.setToolTip("Rescan the USB ports")
        rescan_btn.clicked.connect(self.ports_refreshed.emit)
        hl.addWidget(rescan_btn)

        self._connect_btn = QPushButton("Connect")
        self._connect_btn.setFixedSize(96, 30)
        self._connect_btn.clicked.connect(self._on_connect_clicked)
        hl.addWidget(self._connect_btn)

        hl.addWidget(vline())

        # ── Standard uplink commands ─────────────────────────────────
        for label, key, tip in [
            ("Boot",      "BOOT",      "Uplink CMD,BOOT — restart the flight software"),
            ("Set Time",  "SET_TIME",  "Uplink CMD,SETTIME — sync the CanSat clock to UTC now"),
            ("Calibrate", "CALIBRATE", "Uplink CMD,CALIBRATE — zero gyro, baro and accelerometer on the pad"),
        ]:
            btn = QPushButton(label)
            btn.setFixedHeight(30)
            btn.setToolTip(tip)
            btn.clicked.connect(lambda _, k=key: self.command_sent.emit(k))
            hl.addWidget(btn)

        hl.addWidget(vline())

        # ── CX toggle ────────────────────────────────────────────────
        # Commands the CanSat to start/stop transmitting telemetry, and starts
        # the flight CSV at the same moment.  Defaults to OFF so nothing is
        # recorded until the operator commands transmission (§6.1.v).
        self._cx_btn = QPushButton("CX OFF")
        self._cx_btn.setFixedHeight(30)
        self._cx_btn.setCheckable(True)
        self._cx_btn.setToolTip("Start telemetry downlink and begin recording the flight CSV")
        self._cx_btn.toggled.connect(self._on_cx_toggle)
        hl.addWidget(self._cx_btn)

        hl.addWidget(vline())

        # ── Simulation mode ──────────────────────────────────────────
        for label, key, tip in [
            ("SIM EN",  "SIM_ENABLE",   "Arm simulation mode"),
            ("SIM ACT", "SIM_ACTIVATE", "Activate simulation and start uplinking the altitude profile"),
            ("SIM DIS", "SIM_DISABLE",  "Return the CanSat to its real sensors"),
        ]:
            btn = QPushButton(label)
            btn.setFixedHeight(30)
            btn.setToolTip(tip)
            btn.clicked.connect(lambda _, k=key: self.command_sent.emit(k))
            hl.addWidget(btn)

        hl.addStretch()

        # ── Status block — connection state above, live packet health below ──
        status_box = QWidget()
        status_vl  = QVBoxLayout(status_box)
        status_vl.setContentsMargins(0, 0, 0, 0)
        status_vl.setSpacing(2)

        self._status = QLabel("Not connected")
        self._status.setFont(mono(10))
        self._status.setObjectName("dock_status")
        self._status.setAlignment(Qt.AlignmentFlag.AlignRight)

        # Counts what the radio actually delivered and why anything was thrown
        # away — the difference between "no bytes" and "bytes I cannot read" is
        # the first question to answer when the screen stays empty.
        self._diag = QLabel("")
        self._diag.setFont(mono(10))
        self._diag.setObjectName("dock_diag")
        self._diag.setAlignment(Qt.AlignmentFlag.AlignRight)
        self._last_diag_css = None

        status_vl.addWidget(self._status)
        status_vl.addWidget(self._diag)
        hl.addWidget(status_box)

    # ── port list ─────────────────────────────────────────────────

    def set_ports(self, ports: list):
        """
        Repopulate the port dropdown from (device, label) pairs.

        The label is what the operator reads ("usbserial-A50285BI — FT231X USB
        UART"); the device path is carried along as item data.  A port that is
        still present keeps the selection, so a rescan never moves the
        operator's choice out from under them.
        """
        previous = self.selected_port()
        self._port_box.clear()
        if ports:
            for device, label in ports:
                self._port_box.addItem(label, device)
            index = self._port_box.findData(previous) if previous else -1
            self._port_box.setCurrentIndex(max(0, index))
        else:
            self._port_box.addItem("— plug in the XBee —", "")

    def selected_port(self) -> str:
        return self._port_box.currentData() or ""

    # ── button handlers ───────────────────────────────────────────

    def _on_connect_clicked(self):
        if self._connect_btn.text() == "Connect":
            self.connect_requested.emit(self.selected_port())
        else:
            self.disconnect_requested.emit()

    def _on_cx_toggle(self, on: bool):
        self._cx_btn.setText("CX ON" if on else "CX OFF")
        self.command_sent.emit("CX_ON" if on else "CX_OFF")

    # ── state pushed in by MainWindow ─────────────────────────────

    def set_connected(self, connected: bool):
        self._connect_btn.setText("Disconnect" if connected else "Connect")
        self._port_box.setEnabled(not connected)

    def set_status(self, text: str):
        self._status.setText(text)

    def set_diagnostics(self, text: str, color: str):
        """Packet health line.  Restyled only when the colour actually changes."""
        self._diag.setText(text)
        if color != self._last_diag_css:
            self._last_diag_css = color
            self._diag.setStyleSheet(f"color: {color};")

    def set_cx(self, on: bool):
        """Force the CX button into a state without re-emitting the command."""
        self._cx_btn.blockSignals(True)
        self._cx_btn.setChecked(on)
        self._cx_btn.setText("CX ON" if on else "CX OFF")
        self._cx_btn.blockSignals(False)

    def apply_theme(self):
        self.setStyleSheet(f"""
            QWidget {{ background: {cs('bg1')}; border-top: 1px solid {cs('line')}; }}
            QPushButton {{
                background: {cs('bg2')}; border: 1px solid {cs('line')};
                border-radius: 4px; color: {cs('text')}; padding: 3px 10px;
            }}
            QPushButton:hover   {{ border-color: {cs('cyan')}; color: {cs('cyan')}; }}
            QPushButton:checked {{ border-color: {cs('cyan')}; color: {cs('cyan')}; }}
            QComboBox {{
                background: {cs('bg2')}; border: 1px solid {cs('line')};
                border-radius: 4px; color: {cs('text')}; padding: 3px 8px;
            }}
            QLabel  {{ color: {cs('text')}; background: transparent; border: none; }}
            QLabel#dock_status {{ color: {cs('faint')}; }}
            QFrame  {{ background: {cs('line')}; }}
        """)
        self._last_diag_css = None   # the block above dropped the inline colour


# ═══════════════════════════════════════════════════════════════════
# SETTINGS DIALOG
# ═══════════════════════════════════════════════════════════════════

class SettingsDialog(QDialog):
    """Radio baud rate and the ground-station coordinates used for recovery."""

    baud_changed = pyqtSignal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Settings")
        self.setFixedSize(320, 200)
        fl = QFormLayout(self)
        fl.setContentsMargins(20, 20, 20, 20)
        fl.setSpacing(12)

        # Radio baud rate — takes effect on the next Connect
        baud_sb = QSpinBox()
        baud_sb.setRange(1200, 921600)
        baud_sb.setSingleStep(1200)
        baud_sb.setValue(DEFAULT_BAUD)
        baud_sb.valueChanged.connect(self.baud_changed.emit)
        fl.addRow("Baud rate:", baud_sb)

        # Ground station coordinates — used for the recovery distance/bearing
        lat_sb = QDoubleSpinBox()
        lat_sb.setRange(-90.0, 90.0)
        lat_sb.setDecimals(5)
        lat_sb.setSingleStep(0.001)
        lat_sb.setValue(GROUND_STATION[0])
        lat_sb.setSuffix("°")
        lat_sb.valueChanged.connect(lambda v: GROUND_STATION.__setitem__(0, v))
        fl.addRow("GS Latitude:", lat_sb)

        lon_sb = QDoubleSpinBox()
        lon_sb.setRange(-180.0, 180.0)
        lon_sb.setDecimals(5)
        lon_sb.setSingleStep(0.001)
        lon_sb.setValue(GROUND_STATION[1])
        lon_sb.setSuffix("°")
        lon_sb.valueChanged.connect(lambda v: GROUND_STATION.__setitem__(1, v))
        fl.addRow("GS Longitude:", lon_sb)


# ═══════════════════════════════════════════════════════════════════
# MAIN WINDOW
# Assembles the full layout: sidebar + topbar + tab stack + dock.
# Owns the radio link, the recording and the command handling.
# ═══════════════════════════════════════════════════════════════════

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(f"CanSat Ground Station · Team Kalpana  [{TEAM_ID}]")
        self.resize(1400, 860)
        self.setMinimumSize(1024, 680)

        # Session state
        self._settings       = None    # SettingsDialog instance (lazy)
        self._baud           = DEFAULT_BAUD
        self._nav_open       = False   # whether NavOverlay is visible
        self._recovery_shown = False   # recovery tab already auto-opened?
        self._link_open      = False   # what the dock is currently showing
        self._known_ports    = []      # last port list pushed to the dock

        # Refresh gating — see _on_feed(). _needs_repaint forces one full
        # refresh after a tab switch, so a newly shown tab paints immediately
        # instead of waiting for the next packet.
        self._needs_repaint = True

        self._build_ui()
        self._connect_signals()
        self.apply_theme()

        self._refresh_ports()
        FEED.updated.connect(self._on_feed)

        # Watch the USB ports so the XBee shows up the moment it is plugged in.
        self._port_timer = QTimer(self)
        self._port_timer.setInterval(2000)
        self._port_timer.timeout.connect(self._poll_ports)
        self._port_timer.start()

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
        self._tab_recovery  = RecoveryTab()
        for tab in [self._tab_telemetry, self._tab_graphs,
                    self._tab_location,  self._tab_recovery]:
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
            "recovery":  3,
        }

    def _connect_signals(self):
        self._sidebar.expand_toggled.connect(self._toggle_nav)
        self._sidebar.settings_clicked.connect(self._open_settings)
        self._nav_overlay.tab_changed.connect(self._switch_tab)
        self._nav_overlay.closed.connect(self._close_nav)

        self._dock.connect_requested.connect(self._connect_link)
        self._dock.disconnect_requested.connect(self._disconnect_link)
        self._dock.ports_refreshed.connect(self._refresh_ports)
        self._dock.command_sent.connect(self._on_command)

        for key, tab in [("1", "telemetry"), ("2", "graphs"),
                          ("3", "location"),  ("4", "recovery")]:
            QShortcut(QKeySequence(key), self).activated.connect(
                lambda t=tab: self._switch_tab(t))
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

    def _switch_tab(self, tab_id: str):
        self._sidebar.set_active(tab_id)
        self._nav_overlay.set_active(tab_id)
        self._stack.setCurrentIndex(self._tab_index[tab_id])
        self._needs_repaint = True   # paint the newly visible tab on the next tick

    def _open_settings(self):
        """Open the settings dialog (created once, reused after that)."""
        if self._settings is None:
            dlg = SettingsDialog(self)
            dlg.baud_changed.connect(lambda b: setattr(self, "_baud", b))
            self._settings = dlg
        self._settings.show()

    # ──────────────────────────────────────────────────────────────
    # The radio link
    # ──────────────────────────────────────────────────────────────

    def _refresh_ports(self):
        """Manual rescan (the ⟳ button) — always repopulates and reports back."""
        self._known_ports = logic.TelemetryReceiver.available_ports()
        self._dock.set_ports(self._known_ports)
        self._report_ports()

    def _poll_ports(self):
        """
        While the link is down, watch for the XBee being plugged in or pulled
        out, so the dropdown is always current without anyone pressing ⟳.
        Skipped once a port is open — rescanning then would only fight with the
        operator's own selection.
        """
        if self._link_open:
            return
        ports = logic.TelemetryReceiver.available_ports()
        if ports != self._known_ports:
            self._known_ports = ports
            self._dock.set_ports(ports)
            self._report_ports()

    def _report_ports(self):
        if not logic.HAS_SERIAL:
            self._dock.set_status("pyserial not installed — pip install pyserial")
        elif not self._known_ports:
            self._dock.set_status("No XBee found — check the USB cable")
        else:
            self._dock.set_status(f"{len(self._known_ports)} USB port(s) — press Connect")

    def _connect_link(self, port: str):
        if not port:
            self._dock.set_status("No serial port selected")
            return
        if RECEIVER.connect(port, self._baud, log_dir=APP_DIR):
            self._link_open = True
            self._dock.set_connected(True)
            self._dock.set_status(f"{port} @ {self._baud} baud")
            DATA.log("ok", f"Link opened on {port}")
            if RECEIVER.raw_log_path:
                print(f"Raw radio capture: {RECEIVER.raw_log_path}")
        else:
            self._dock.set_status(RECEIVER.error)
            DATA.log("warn", f"Could not open {port}")

    def _disconnect_link(self):
        RECEIVER.disconnect()
        self._link_open = False
        self._dock.set_connected(False)
        self._dock.set_status("Not connected")
        DATA.log("warn", "Link closed")

    # ──────────────────────────────────────────────────────────────
    # Command handling
    # Every button writes one ASCII string up to the CanSat, and CX ON/OFF
    # also starts and stops the flight CSV.
    # ──────────────────────────────────────────────────────────────

    def _on_command(self, key: str):
        value = None
        if key == "SET_TIME":
            value = time.strftime("%H:%M:%S", time.gmtime())

        text = logic.build_command(key, value)
        if RECEIVER.send(text):
            DATA.log("cyan", f"Sent  {text}")
            self._dock.set_status(f"Sent  {text}")
        else:
            DATA.log("warn", f"NOT SENT (link offline)  {text}")
            self._dock.set_status("Not sent — link offline")
            # CX drives recording, so an unsent CX ON must not leave the button
            # claiming the downlink is running.
            if key == "CX_ON":
                self._dock.set_cx(False)
            return

        if key == "CX_ON":
            self._start_recording()
        elif key == "CX_OFF":
            self._stop_recording()
        elif key == "SIM_ACTIVATE":
            SIMULATOR.start()
            DATA.log("cyan", "Simulation profile started (1 Hz)")
        elif key == "SIM_DISABLE":
            SIMULATOR.stop()
            DATA.log("ok", "Simulation stopped — real sensors")

    # ──────────────────────────────────────────────────────────────
    # CSV recording
    # The file opens on CX ON so it captures the pad wait and calibration
    # readings too, not just the flight.
    # ──────────────────────────────────────────────────────────────

    def _start_recording(self):
        path = RECEIVER.start_recording(APP_DIR)
        DATA.log("ok", "Recording started")
        print(f"Recording to {path}")

    def _stop_recording(self):
        if RECEIVER.recording:
            print(f"Recording saved to {RECEIVER.csv_path}")
            RECEIVER.stop_recording()
            DATA.log("warn", "Recording stopped")

    def closeEvent(self, event):
        """Close the port and the recording cleanly on the way out."""
        SIMULATOR.stop()
        RECEIVER.stop_recording()
        RECEIVER.disconnect()
        super().closeEvent(event)

    # ──────────────────────────────────────────────────────────────
    # Packet health — the line that explains an empty screen
    # ──────────────────────────────────────────────────────────────

    def _update_diagnostics(self):
        """
        Report what the radio delivered and what became of it.

        Three failure modes look identical on a blank display, so they are
        spelled out differently here:
          - link open, no lines at all   → wrong port, wrong baud, CanSat off
          - lines arriving, none usable  → packet format or team ID mismatch
          - some usable, some not        → interference, normal at low signal
        """
        p = RECEIVER.parser
        if not self._link_open and not p.accepted and not p.rejected:
            self._dock.set_diagnostics("", cs("faint"))
            return

        if RECEIVER.raw_lines == 0:
            self._dock.set_diagnostics("no data on the port — check baud rate",
                                       cs("amber"))
            return

        text = f"rx {p.accepted} · lost {p.dropped} · unusable {p.rejected}"
        if p.rejected:
            text += f"  ({p.last_error})"

        if p.rejected and not p.accepted:
            color = cs("red")        # nothing is getting through at all
        elif p.rejected:
            color = cs("amber")
        else:
            color = cs("green")
        self._dock.set_diagnostics(text, color)

    # ──────────────────────────────────────────────────────────────
    # Main update loop — fires 10× per second via TelemetryFeed.updated
    # ──────────────────────────────────────────────────────────────

    def _on_feed(self, new_packets: bool):
        """
        The feed ticks at 10 Hz, but telemetry arrives once a second.  The
        header is cheap and always refreshed so the clock and the link status
        stay live; the tabs are only redrawn when there is genuinely something
        new to show, or when a tab switch needs its first paint.
        """
        self._topbar.update_data(DATA, RECEIVER)
        self._update_diagnostics()

        # The reader thread stops itself when the cable is pulled, so nothing
        # tells the dock — it has to notice here.
        if self._link_open and not RECEIVER.connected:
            self._link_open = False
            self._dock.set_connected(False)
            self._dock.set_status(RECEIVER.error or "Link lost")
            DATA.log("warn", "LINK LOST — check the radio")

        if DATA.current is None:
            return
        if not new_packets and not self._needs_repaint:
            return
        self._needs_repaint = False

        # After landing, put the operator straight on the recovery screen —
        # once, so they can still navigate away afterwards.
        if DATA.landed and not self._recovery_shown:
            self._recovery_shown = True
            self._switch_tab("recovery")
            DATA.log("cyan", "IMPACT — recovery mode")

        # Only update the currently visible tab (saves CPU)
        idx = self._stack.currentIndex()
        if   idx == 0: self._tab_telemetry.update_data(DATA)
        elif idx == 1: self._tab_graphs.update_data(DATA)
        elif idx == 2: self._tab_location.update_data(DATA)
        elif idx == 3: self._tab_recovery.update_data(DATA, RECEIVER.csv_path)

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
      1. Create the receiver (serial + parser + CSV) and the data store.
      2. Create QApplication and the feed that bridges them into Qt.
      3. Show the window.  The operator picks a port and presses Connect.
    """
    global RECEIVER, DATA, FEED, SIMULATOR

    _warn_if_cloud_evicted()

    if not logic.HAS_SERIAL:
        print("WARNING: pyserial is not installed — the ground station cannot")
        print("         open a radio link.  Fix with:  pip install pyserial")

    if HAS_PG:
        pg.setConfigOptions(antialias=True)

    app = QApplication(sys.argv)
    app.setApplicationName("CanSat Ground Station")

    RECEIVER  = logic.TelemetryReceiver()
    DATA      = logic.MissionData()
    SIMULATOR = logic.SimulationSender(RECEIVER)
    FEED      = TelemetryFeed(RECEIVER, DATA)

    win = MainWindow()
    win.show()
    sys.exit(app.exec())
