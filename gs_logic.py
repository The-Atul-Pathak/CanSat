"""
CanSat Ground Station — logic layer.  Pure Python, no Qt.

This file holds everything that is not drawing:

    Layer 1  SerialLink        reads raw lines from the XBee on a USB port
    Layer 2  PacketParser      turns one raw line into a validated packet dict
    Layer 3  MissionData       everything received this session, in memory
    Layer 5  CSVWriter         appends every valid packet to disk, flushed

    TelemetryReceiver          ties layers 1, 2 and 5 together and hands
                               finished packets to the UI through a queue
    SimulationSender           uplinks the altitude profile during SIM mode

Layer 4 (the display) lives in gs_ui.py and is the only part that needs Qt.

Threading
---------
Two threads, and one queue between them:

    reader thread   read line → parse → write CSV row → put packet on queue
    UI thread       drain queue → store → redraw

Everything that must survive a slow or broken display happens in the reader
thread, so a stalled repaint can never lose a row from the flight CSV.  The
queue is the only shared mutable state, and Queue is thread-safe.
"""

import csv
import math
import os
import queue
import threading
import time

try:
    import serial
    from serial.tools import list_ports
    HAS_SERIAL = True
except ImportError:              # the UI still runs, it just cannot connect
    HAS_SERIAL = False


# ═══════════════════════════════════════════════════════════════════
# FLIGHT STATES
# Each CanSat state maps to a numeric code transmitted in the packet.
# ═══════════════════════════════════════════════════════════════════

# Forward map: name → number
STATE_NUMBER = {
    "BOOT":              0,
    "TEST_MODE":         1,
    "LAUNCH_PAD":        2,
    "ASCENT":            3,
    "ROCKET_DEPLOY":     4,
    "DESCENT":           5,
    "AEROBREAK_RELEASE": 6,
    "IMPACT":            7,
}

# Reverse map: number → name  (used when parsing the state field)
STATE_NAME = {v: k for k, v in STATE_NUMBER.items()}

# Color key each state should use in the UI top-bar pill
STATE_COLOR = {
    "BOOT":              "dim",
    "TEST_MODE":         "dim",
    "LAUNCH_PAD":        "green",
    "ASCENT":            "amber",
    "ROCKET_DEPLOY":     "cyan",
    "DESCENT":           "amber",
    "AEROBREAK_RELEASE": "green",
    "IMPACT":            "green",
}

# States in which the CanSat is on its way down — used by the descent panel
DESCENT_STATES = ("DESCENT", "AEROBREAK_RELEASE", "IMPACT")


# ═══════════════════════════════════════════════════════════════════
# TEAM IDENTITY & TELEMETRY FORMAT  (IN-SPACe CAN-7USAT §6.3)
# Single source of truth for the team ID and the graded .csv layout.
# ═══════════════════════════════════════════════════════════════════

# Official team identifier — format per §6.3: 2026-IN-SPACe-CAN-7USAT-XXX
TEAM_ID = "2026-IN-SPACe-CAN-7USAT-056"

# Graded telemetry file name (§6.3.iii: "Flight_<TEAM_ID>.csv")
CSV_FILENAME = f"Flight_{TEAM_ID}.csv"

# Radio defaults — the XBee ships at 9600 baud
DEFAULT_BAUD = 9600

# A packet is expected every second.  If none arrives for this long the link
# is reported as stale, which is the operator's cue to check the radio.
LINK_TIMEOUT_S = 5.0

# Telemetry field order (§6.3.i).  These columns mirror the flight telemetry
# format exactly, so the ground station records exactly what it receives.  The
# block up to and including eCO2 is what the CanSat transmits; the trailing
# columns are ground-station-derived extras that the spec permits after the
# required set (§6.3.B) — including the mandatory mechanical GYRO_SPIN_RATE
# (§6.3 item 14) when the CanSat does not transmit it itself.
TELEMETRY_HEADER = [
    # ── telemetry fields, matching the on-board packet format ──
    "TEAM_ID",                 # 2026-IN-SPACe-CAN-7USAT-056
    "TIME_STAMPING",           # seconds since initial power-up
    "PACKET_COUNT",            # count of transmitted packets
    "ALTITUDE",                # metres, relative to ground   (0.1 m)
    "PRESSURE",                # pascals                      (1 Pa)
    "TEMP",                    # degrees Celsius              (0.1 C)
    "VOLTAGE",                 # power-bus volts              (0.01 V)
    "GNSS_TIME",               # time from the GNSS receiver
    "GNSS_LATITUDE",           # degrees                      (0.0001 deg)
    "GNSS_LONGITUDE",          # degrees                      (0.0001 deg)
    "GNSS_ALTITUDE",           # metres                       (0.1 m)
    "GNSS_SATS",               # satellites in view (integer)
    "ACC_R", "ACC_P", "ACC_Y", # accelerometer roll/pitch/yaw
    "GYRO_R", "GYRO_P", "GYRO_Y",  # gyro roll/pitch/yaw (deg/s)
    "FLIGHT_SOFTWARE_STATE",   # numeric state code 0-7
    "TVOC", "eCO2",            # air-quality payload sensor
    # ── ground-station-derived extras (appended after telemetry fields) ──
    "GYRO_SPIN_RATE",          # mechanical gyro spin rate (deg/s, §6.3 item 14)
    "VELOCITY", "STATE",
]


def telemetry_row(pk: dict) -> list:
    """
    Serialise one packet into a CSV row in TELEMETRY_HEADER order.

    Units follow the §6.3 resolution table exactly:
      - PRESSURE is emitted in pascals (we store hPa internally, so ×100).
      - FLIGHT_SOFTWARE_STATE is the numeric code 0-7; the human-readable name
        is kept as the extra trailing STATE column.
    """
    return [
        pk["team_id"],
        f"{pk['t']:.1f}",
        pk["packet"],
        f"{pk['altitude']:.1f}",
        f"{pk['pressure'] * 100:.0f}",   # hPa → Pa (1 Pa resolution)
        f"{pk['temp']:.1f}",
        f"{pk['voltage']:.2f}",
        pk["gnss_time"],
        f"{pk['lat']:.5f}",
        f"{pk['lon']:.5f}",
        f"{pk['gnss_alt']:.1f}",
        pk["sats"],
        f"{pk['acc_r']:.2f}",
        f"{pk['acc_p']:.2f}",
        f"{pk['acc_y']:.2f}",
        f"{pk['gyro_r']:.2f}",
        f"{pk['gyro_p']:.2f}",
        f"{pk['gyro_y']:.2f}",
        pk["state_num"],
        f"{pk['tvoc']:.0f}",
        f"{pk['eco2']:.0f}",
        # ground-station-derived extras
        f"{pk['gyro_spin']:.1f}",
        f"{pk['velocity']:.2f}",
        pk["state"],
    ]


# ═══════════════════════════════════════════════════════════════════
# LAYER 2 — PACKET PARSER
# One raw ASCII line in, one validated packet dict out.
# ═══════════════════════════════════════════════════════════════════

# The fields the CanSat transmits, in order, as (dict key, type).  Every one of
# these is mandatory — a line with fewer fields is not a usable packet.
_REQUIRED_FIELDS = [
    ("team_id",   str),
    ("t",         float),   # TIME_STAMPING, seconds since power-up
    ("packet",    int),
    ("altitude",  float),
    ("pressure",  float),   # pascals on the wire, converted to hPa below
    ("temp",      float),
    ("voltage",   float),
    ("gnss_time", str),
    ("lat",       float),
    ("lon",       float),
    ("gnss_alt",  float),
    ("sats",      int),
    ("acc_r",     float),
    ("acc_p",     float),
    ("acc_y",     float),
    ("gyro_r",    float),
    ("gyro_p",    float),
    ("gyro_y",    float),
    ("state_num", int),
]

# Fields that may or may not be present depending on which sensors are fitted.
# Missing ones default to 0 rather than rejecting the whole packet.
_OPTIONAL_FIELDS = [
    ("tvoc",      float),
    ("eco2",      float),
    ("gyro_spin", float),
]

# Physically plausible ranges.  A reading outside its range means a sensor
# fault, not a real measurement, so the packet is rejected rather than drawn.
# Pressure is checked in hPa, i.e. after the Pa → hPa conversion.
_LIMITS = {
    "altitude": (-100.0, 5000.0),
    "pressure": (10.0,   1200.0),
    "temp":     (-90.0,  100.0),
    "voltage":  (0.0,    16.0),
    "lat":      (-90.0,  90.0),
    "lon":      (-180.0, 180.0),
    "sats":     (0,      64),
}


def _to_seconds(text: str) -> float:
    """
    Read TIME_STAMPING, which the flight software may send either as plain
    seconds ("125.0") or as a clock string ("0:02:05").
    """
    if ":" in text:
        parts = [float(p) for p in text.split(":")]
        seconds = 0.0
        for part in parts:            # h:m:s, or m:s
            seconds = seconds * 60 + part
        return seconds
    return float(text)


class PacketParser:
    """
    Layer 2 — validates raw lines and turns the good ones into packet dicts.

    Also derives the two things the CanSat does not transmit:
      - velocity, by differentiating altitude over time
      - dropped-packet count, from gaps in the CanSat's own PACKET_COUNT
    """

    def __init__(self, team_id: str = TEAM_ID):
        self.team_id  = team_id
        self.dropped  = 0     # packets the radio link lost (gaps in PACKET_COUNT)
        self.rejected = 0     # lines that arrived but could not be used
        self.accepted = 0     # packets successfully parsed
        self.last_error = ""

        self._last_count = None
        self._last_alt   = None
        self._last_t     = None

    def reset(self):
        """Forget the previous flight — called when a new session starts."""
        self.dropped = self.rejected = self.accepted = 0
        self.last_error = ""
        self._last_count = self._last_alt = self._last_t = None

    def parse(self, raw_line: str):
        """
        Return a packet dict, or None if the line is not usable.

        Rejected lines are counted and the reason is kept in last_error so the
        operator can see what is arriving; the display simply holds its last
        good values.
        """
        line = raw_line.strip()
        # Some flight software wraps each packet in angle brackets.
        if line.startswith("<") and line.endswith(">"):
            line = line[1:-1]
        if not line:
            return None

        fields = [f.strip() for f in line.split(",")]
        if len(fields) < len(_REQUIRED_FIELDS):
            return self._reject(f"only {len(fields)} fields")

        pk = {}
        try:
            for i, (key, kind) in enumerate(_REQUIRED_FIELDS):
                pk[key] = _to_seconds(fields[i]) if key == "t" else kind(fields[i])
            for j, (key, kind) in enumerate(_OPTIONAL_FIELDS):
                i = len(_REQUIRED_FIELDS) + j
                pk[key] = kind(fields[i]) if i < len(fields) and fields[i] else 0.0
        except (ValueError, IndexError):
            return self._reject("bad number format")

        # Foreign packet — another team's CanSat on a nearby frequency.
        if pk["team_id"] != self.team_id:
            return self._reject(f"foreign team id {pk['team_id']}")

        pk["pressure"] /= 100.0        # Pa → hPa for display
        pk["altitude"] = max(-100.0, pk["altitude"])

        for key, (low, high) in _LIMITS.items():
            if not low <= pk[key] <= high:
                return self._reject(f"{key} out of range ({pk[key]})")

        pk["state"] = STATE_NAME.get(pk["state_num"], "BOOT")

        # Velocity is not transmitted — differentiate altitude over time.
        if self._last_alt is None or self._last_t is None:
            pk["velocity"] = 0.0
        else:
            dt = pk["t"] - self._last_t
            pk["velocity"] = (pk["altitude"] - self._last_alt) / dt if dt > 0 else 0.0
        self._last_alt = pk["altitude"]
        self._last_t   = pk["t"]

        # A jump in PACKET_COUNT is exactly what the radio link lost.
        if self._last_count is not None and pk["packet"] > self._last_count:
            self.dropped += pk["packet"] - self._last_count - 1
        self._last_count = pk["packet"]

        self.accepted += 1
        return pk

    def _reject(self, reason: str):
        self.rejected  += 1
        self.last_error = reason
        return None


# ═══════════════════════════════════════════════════════════════════
# LAYER 5 — CSV WRITER
# One row per valid packet, flushed to disk immediately.
# ═══════════════════════════════════════════════════════════════════

class CSVWriter:
    """
    Writes the graded flight file.

    Every row is flushed the moment it is written.  Without that, rows sit in
    the operating system's buffer until the file is closed, and a laptop that
    dies mid-flight takes the whole recording with it.  Flushing costs nothing
    at one packet per second and caps the worst-case loss at a single row.
    """

    def __init__(self, directory: str, filename: str = CSV_FILENAME):
        self.path  = os.path.join(directory, filename)
        self._file = open(self.path, "w", newline="")
        self._csv  = csv.writer(self._file)
        self._csv.writerow(TELEMETRY_HEADER)
        self._file.flush()

    def write(self, pk: dict):
        self._csv.writerow(telemetry_row(pk))
        self._file.flush()

    def close(self):
        if self._file:
            self._file.flush()
            self._file.close()
            self._file = None


# ═══════════════════════════════════════════════════════════════════
# LAYER 1 + THE GLUE — TELEMETRY RECEIVER
# Owns the serial port, the reader thread, the parser and the CSV writer.
# ═══════════════════════════════════════════════════════════════════

# Link states reported to the UI
LINK_OFFLINE = "OFFLINE"    # no port open
LINK_LIVE    = "LIVE"       # packets arriving normally
LINK_STALE   = "NO DATA"    # port open but nothing received recently
LINK_LOST    = "LINK LOST"  # the port disappeared (cable pulled)


class TelemetryReceiver:
    """
    Everything between the USB port and the display.

    connect() opens the port and starts a daemon reader thread.  That thread
    reads one line at a time, parses it, writes it to the flight CSV and puts
    it on self.packets for the UI to collect.  Nothing here touches Qt, and
    nothing here waits on the UI.
    """

    def __init__(self):
        self.parser  = PacketParser()
        self.packets = queue.Queue()   # parsed packets waiting for the UI
        self.port_name = ""
        self.error     = ""            # last connection error, shown in the UI

        self._serial  = None
        self._thread  = None
        self._running = False
        self._last_rx = 0.0            # monotonic time of the last valid packet

        # The writer is swapped in and out by the UI thread (CX ON / CX OFF)
        # while the reader thread is using it, so both go through this lock.
        self._writer      = None
        self._writer_lock = threading.Lock()

    # ── port discovery ────────────────────────────────────────────

    @staticmethod
    def available_ports() -> list:
        """
        Return the USB serial ports the XBee could be on, as a list of
        (device_path, human_label) pairs.

        The XBee plugs in over USB, and a USB adapter is the only kind of
        serial port that reports a vendor ID — so `vid is not None` is exactly
        the test for "something is physically plugged in here".  Everything
        else the operating system offers is built in and permanently present
        whether or not any hardware exists: the debug console, and one entry
        per paired Bluetooth device (headphones included).  Listing those on
        launch day would only give the operator a chance to open the wrong
        port, so they are left out.

        The label carries the adapter's own description, which is how you tell
        two identical-looking ports apart.
        """
        if not HAS_SERIAL:
            return []

        ports = []
        for p in list_ports.comports():
            if p.vid is None:
                continue
            device = p.device or ""
            label  = f"{os.path.basename(device)} — {p.description or 'USB serial'}"
            ports.append((device, label))
        return ports

    # ── connection ────────────────────────────────────────────────

    def connect(self, port: str, baud: int = DEFAULT_BAUD) -> bool:
        """Open the port and start reading.  Returns True on success."""
        if not HAS_SERIAL:
            self.error = "pyserial is not installed (pip install pyserial)"
            return False
        self.disconnect()
        try:
            self._serial = serial.Serial(port, baud, timeout=1)
        except Exception as exc:       # serial raises several unrelated types
            self.error = str(exc)
            self._serial = None
            return False

        self.port_name = port
        self.error     = ""
        self._last_rx  = time.monotonic()
        self._running  = True
        self._thread   = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        return True

    def disconnect(self):
        """Stop the reader thread and close the port."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
        self.port_name = ""

    @property
    def connected(self) -> bool:
        return self._serial is not None and self._running

    def link_status(self) -> str:
        """One of LINK_OFFLINE / LINK_LIVE / LINK_STALE / LINK_LOST."""
        if not self.connected:
            # An error while a port was open means the link died on us; no
            # error means the operator simply has not connected yet.
            return LINK_LOST if self.error else LINK_OFFLINE
        if time.monotonic() - self._last_rx > LINK_TIMEOUT_S:
            return LINK_STALE
        return LINK_LIVE

    # ── the reader thread ─────────────────────────────────────────

    def _read_loop(self):
        """
        Read one line at a time until disconnect() or the cable is pulled.

        This runs in a background thread precisely so that reading never waits
        on the display: readline() blocks for up to a second, which would
        freeze the whole window if it ran on the UI thread.
        """
        while self._running:
            try:
                raw = self._serial.readline()
            except Exception as exc:   # cable pulled, adapter removed, …
                # A read that fails while we were being shut down anyway is not
                # something to report as a lost link.
                if self._running:
                    self.error = str(exc)
                self._running = False
                break

            if not raw:
                continue               # readline() timed out — no data yet

            try:
                line = raw.decode("ascii", errors="ignore")
            except Exception:
                continue               # unusable bytes — drop the line, keep going

            pk = self.parser.parse(line)
            if pk is None:
                continue

            self._last_rx = time.monotonic()

            # Write to disk before handing the packet to the UI, so the flight
            # record does not depend on the display keeping up.
            with self._writer_lock:
                if self._writer:
                    try:
                        self._writer.write(pk)
                    except Exception as exc:
                        self.error = f"CSV write failed: {exc}"

            self.packets.put(pk)

    # ── uplink ────────────────────────────────────────────────────

    def send(self, text: str) -> bool:
        """Transmit one command string to the CanSat.  Returns True if sent."""
        if not self.connected:
            return False
        try:
            self._serial.write((text + "\n").encode("ascii"))
            return True
        except Exception as exc:
            self.error = str(exc)
            return False

    # ── recording ─────────────────────────────────────────────────

    def start_recording(self, directory: str) -> str:
        """
        Open the flight CSV.  Called on CX ON, so the file captures the pad
        wait and calibration readings too, not just the flight.
        """
        with self._writer_lock:
            if self._writer:
                return self._writer.path
            self._writer = CSVWriter(directory)
            return self._writer.path

    def stop_recording(self):
        with self._writer_lock:
            if self._writer:
                self._writer.close()
                self._writer = None

    @property
    def recording(self) -> bool:
        return self._writer is not None

    @property
    def csv_path(self) -> str:
        return self._writer.path if self._writer else ""


# ═══════════════════════════════════════════════════════════════════
# COMMAND SET
# Every button in the UI sends one of these ASCII strings up to the CanSat.
# ═══════════════════════════════════════════════════════════════════

COMMANDS = {
    "BOOT":         "CMD,BOOT",
    "SET_TIME":     "CMD,SETTIME,{value}",
    "CALIBRATE":    "CMD,CALIBRATE",
    "CX_ON":        "CMD,CX,ON",
    "CX_OFF":       "CMD,CX,OFF",
    "SIM_ENABLE":   "CMD,SIM,ENABLE",
    "SIM_ACTIVATE": "CMD,SIM,ACTIVATE",
    "SIM_DISABLE":  "CMD,SIM,DISABLE",
    "SIM_PRESSURE": "CMD,SIMP,{value}",
}


def build_command(key: str, value=None) -> str:
    """Fill in the {value} placeholder for the commands that take an argument."""
    return COMMANDS[key].format(value=value)


# ═══════════════════════════════════════════════════════════════════
# SIMULATION MODE
# The CanSat flies on altitudes we send it instead of its own barometer.
# ═══════════════════════════════════════════════════════════════════

# Altitude profile uplinked at 1 Hz once simulation is active (metres).
SIM_ALTITUDE_PROFILE = [
    0, 0, 20, 80, 200, 400, 600, 800, 950, 1000,     # ascent
    980, 900, 800, 700, 650, 610, 595, 580,          # descent through 600 m
    400, 250, 100, 30, 5, 0,                         # post-secondary descent
]


class SimulationSender:
    """
    Sends SIM_ALTITUDE_PROFILE to the CanSat, one value per second.

    Runs on its own thread so the 1 Hz pacing never blocks the display.  The
    telemetry that comes back is received, drawn and recorded exactly like a
    real flight — nothing downstream knows the difference.
    """

    def __init__(self, receiver: TelemetryReceiver):
        self._receiver = receiver
        self._thread   = None
        self._running  = False
        self.index     = 0             # how far through the profile we are

    @property
    def active(self) -> bool:
        return self._running

    def start(self):
        if self._running:
            return
        self.index    = 0
        self._running = True
        self._thread  = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _run(self):
        for i, altitude in enumerate(SIM_ALTITUDE_PROFILE):
            if not self._running:
                return
            self.index = i + 1
            self._receiver.send(build_command("SIM_PRESSURE", altitude))
            time.sleep(1.0)
        self._running = False


# ═══════════════════════════════════════════════════════════════════
# LAYER 3 — MISSION DATA STORE
# Single source of truth for everything received this session.
# Only the UI thread touches this, so it needs no locking.
# ═══════════════════════════════════════════════════════════════════

# Altitude at which the secondary deployment is expected (§6.1)
SECONDARY_TRIGGER_ALT = 600.0

# Battery limits used for the percentage readout and the low-voltage warning
VOLTAGE_FULL  = 8.4
VOLTAGE_EMPTY = 6.5
VOLTAGE_WARN  = 7.0


class MissionData:
    """
    Holds the full packet history plus the small amount of state the display
    needs but no single packet carries: apex altitude, the event log, and the
    mission clock's zero point.
    """

    def __init__(self):
        self.packets = []          # every valid packet, in arrival order
        self.current = None        # the most recent packet
        self.events  = []          # (met_seconds, severity, message)
        self.apex    = 0.0         # highest altitude seen
        self.start_monotonic = None  # set by the first packet

        self._last_state       = None
        self._secondary_logged = False
        self._voltage_warned   = False

    # ── mission clock ─────────────────────────────────────────────

    def met(self) -> float:
        """
        Mission elapsed time in seconds, measured from the first packet.

        Driven by the wall clock rather than by packet timestamps so it keeps
        running during a dropout — a frozen clock would hide exactly the
        problem the operator needs to see.
        """
        if self.start_monotonic is None:
            return 0.0
        return time.monotonic() - self.start_monotonic

    # ── ingest ────────────────────────────────────────────────────

    def add(self, pk: dict):
        """Store one packet and log any event it triggers."""
        if self.start_monotonic is None:
            self.start_monotonic = time.monotonic()

        self.packets.append(pk)
        self.current = pk
        self.apex    = max(self.apex, pk["altitude"])
        self._check_events(pk)

    def log(self, severity: str, message: str):
        """Add a timestamped line to the mission event log."""
        self.events.append((self.met(), severity, message))

    def _check_events(self, pk: dict):
        """Detect the things worth writing down automatically."""
        if pk["state"] != self._last_state:
            if self._last_state is not None:
                self.log("warn", f"State → {pk['state'].replace('_', ' ')}")
            self._last_state = pk["state"]

        if (not self._secondary_logged
                and pk["state"] in DESCENT_STATES
                and pk["altitude"] < SECONDARY_TRIGGER_ALT + 5):
            self.log("cyan", f"Altitude crossed {SECONDARY_TRIGGER_ALT:.0f} m "
                             f"— secondary trigger zone")
            self._secondary_logged = True

        if not self._voltage_warned and pk["voltage"] < VOLTAGE_WARN:
            self.log("warn", f"WARNING: battery below {VOLTAGE_WARN:.1f} V")
            self._voltage_warned = True

    # ── readouts ──────────────────────────────────────────────────

    def recent_events(self, count: int = 8) -> list:
        """The newest events first — that is the order the log panel shows."""
        return self.events[-count:][::-1]

    @property
    def landed(self) -> bool:
        return self.current is not None and self.current["state"] == "IMPACT"


def battery_percent(voltage: float) -> float:
    """Rough state of charge from bus voltage, clamped to 0-100 %."""
    span = VOLTAGE_FULL - VOLTAGE_EMPTY
    return max(0.0, min(100.0, (voltage - VOLTAGE_EMPTY) / span * 100.0))


# ═══════════════════════════════════════════════════════════════════
# GROUND STATION POSITION
# Updated via the Settings dialog or by editing directly below.
# ═══════════════════════════════════════════════════════════════════

# [latitude_deg, longitude_deg]  — Chennai as default
GROUND_STATION = [13.0827, 80.2707]


def haversine(lat1, lon1, lat2, lon2):
    """
    Return (distance_km, bearing_deg) between two GPS coordinates.

    Uses the haversine formula — accurate for short distances (< 500 km).
    Bearing is measured clockwise from North.
    """
    EARTH_RADIUS_KM = 6371.0

    # Convert everything to radians
    phi1    = math.radians(lat1)
    phi2    = math.radians(lat2)
    dphi    = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    # Haversine formula for arc length
    a        = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    distance = EARTH_RADIUS_KM * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    # Forward azimuth (bearing)
    y       = math.sin(dlambda) * math.cos(phi2)
    x       = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
    bearing = (math.degrees(math.atan2(y, x)) + 360) % 360

    return distance, bearing


def attitude_from_accel(pk: dict):
    """
    Return (roll_deg, pitch_deg) for the attitude indicator.

    Roll and pitch are not transmitted, so they are derived from the
    accelerometer the way any tilt sensor does it — gravity points down, and
    how it splits across the three axes gives the angle.  This assumes the yaw
    axis (ACC_Y) runs along the CanSat's body, i.e. it reads about +1 g when
    the CanSat is standing upright on the pad.  If your IMU is mounted with a
    different axis vertical, this function is the only place to change.
    """
    ax, ay, az = pk["acc_r"], pk["acc_p"], pk["acc_y"]
    roll  = math.degrees(math.atan2(ax, az)) if (ax or az) else 0.0
    pitch = math.degrees(math.atan2(-ay, math.sqrt(ax * ax + az * az))) if (ax or ay or az) else 0.0
    return roll, pitch


def compass_point(bearing: float) -> str:
    """Turn a bearing into the 16-point compass name you can walk on."""
    names = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
             "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]
    return names[int((bearing + 11.25) % 360 / 22.5)]


# ═══════════════════════════════════════════════════════════════════
# SMALL PURE-PYTHON UTILITIES
# ═══════════════════════════════════════════════════════════════════

def fmt_met(t: float) -> str:
    """Format mission elapsed time as T+MM:SS.d  (e.g. T+01:23.4)."""
    sign = "+" if t >= 0 else "-"
    a    = abs(t)
    mins = int(a // 60)
    secs = int(a % 60)
    deci = int((a * 10) % 10)
    return f"T{sign}{mins:02d}:{secs:02d}.{deci}"
