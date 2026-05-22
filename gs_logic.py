"""
gs_logic.py  —  CanSat Ground Station: DATA & LOGIC LAYER
Team Kalpana · 2026-CANSAT-ASI-023

This file has ZERO Qt code.  It handles:
  1. Simulation  — generates fake telemetry packets for demo / testing
  2. CSV loading — reads real telemetry from trial_data.csv
  3. Playback    — SimState advances time and serves packets to the UI
  4. Utilities   — time formatting, haversine distance/bearing

The UI layer (gs_ui.py) imports everything it needs from here.
To swap in real radio data, replace or extend SimState._tick().
"""

import math
import csv
import time


# ═══════════════════════════════════════════════════════════════════
# FLIGHT STATES
# Each CanSat state maps to a numeric code transmitted in the packet.
# ═══════════════════════════════════════════════════════════════════

# Forward map: name → number  (used when building simulated packets)
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

# Reverse map: number → name  (used when parsing CSV state column)
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


# ═══════════════════════════════════════════════════════════════════
# SIMULATION ENGINE
# Generates synthetic telemetry for demonstration.
# Not used when trial_data.csv is present.
# ═══════════════════════════════════════════════════════════════════

# Total simulated mission length and packet rate
MISSION_DURATION = 200.0   # seconds
PACKET_HZ        = 10      # packets per second
TOTAL_PACKETS    = int(MISSION_DURATION * PACKET_HZ)

# Altitude profile: list of (time_s, altitude_m, state_name)
# Points are smooth-step interpolated between consecutive entries.
ALTITUDE_PROFILE = [
    (0,   0,   "BOOT"),
    (4,   0,   "TEST_MODE"),
    (6,   0,   "LAUNCH_PAD"),
    (6.5, 4,   "ASCENT"),
    (10,  180, "ASCENT"),
    (14,  480, "ASCENT"),
    (20,  700, "ASCENT"),
    (24,  720, "ROCKET_DEPLOY"),
    (28,  700, "DESCENT"),
    (50,  560, "DESCENT"),
    (75,  500, "AEROBREAK_RELEASE"),
    (100, 280, "AEROBREAK_RELEASE"),
    (125, 50,  "AEROBREAK_RELEASE"),
    (132, 0,   "IMPACT"),
    (200, 0,   "IMPACT"),
]


def _noise(t, seed=1, amplitude=1.0):
    """Deterministic pseudo-random noise — same value for same inputs."""
    raw = math.sin(t * 12.9898 + seed * 78.233) * 43758.5453
    return (raw - math.floor(raw) - 0.5) * 2 * amplitude


def _altitude_at(t):
    """Interpolate altitude from ALTITUDE_PROFILE using smooth-step."""
    for i in range(len(ALTITUDE_PROFILE) - 1):
        t_start, alt_start, _ = ALTITUDE_PROFILE[i]
        t_end,   alt_end,   _ = ALTITUDE_PROFILE[i + 1]
        if t_start <= t < t_end:
            # Normalized progress 0→1 within this segment
            progress = (t - t_start) / (t_end - t_start)
            # Smooth-step easing: slow at edges, fast in middle
            smooth = progress * progress * (3 - 2 * progress)
            return alt_start + (alt_end - alt_start) * smooth
    return float(ALTITUDE_PROFILE[-1][1])


def _state_at(t):
    """Return the flight state name for a given mission time."""
    if t < 4:    return "BOOT"
    if t < 6:    return "TEST_MODE"
    if t < 6.5:  return "LAUNCH_PAD"
    if t < 24:   return "ASCENT"
    if t < 28:   return "ROCKET_DEPLOY"
    if t < 75:   return "DESCENT"
    if t < 132:  return "AEROBREAK_RELEASE"
    return "IMPACT"


def _build_packet(t):
    """
    Build one telemetry packet dict for simulation time t.

    Every key in this dict must also be produced by load_csv_data()
    so the UI can treat both sources identically.
    """
    alt   = _altitude_at(t) + _noise(t, seed=1, amplitude=0.4)
    alt   = max(0.0, alt)

    # Velocity = change in altitude per 0.1 s window
    alt_prev = _altitude_at(max(0, t - 0.1))
    velocity = (alt - alt_prev) / 0.1

    state = _state_at(t)

    # Accelerometer noise scales up during ascent
    accel_scale = 4 if state == "ASCENT" else 1

    # Spin rate ramps up during descent (aerobrake deployment)
    gyro_spin = 0.0
    if state in ("DESCENT", "AEROBREAK_RELEASE"):
        gyro_spin = max(0.0, min(3600.0, (t - 28) * 80)) + _noise(t, 50, 40)

    # GPS drifts slowly during flight
    ascent_frac  = min(1.0, t / 24)           # 0→1 during ascent
    descent_frac = max(0.0, min(1.0, (t - 24) / (132 - 24)))

    # Clock: mission starts at 08:42:00
    clock_sec = 8 * 3600 + 42 * 60 + int(t)

    return {
        "t":         t,
        "packet":    int(t * PACKET_HZ),
        "state":     state,
        "state_num": STATE_NUMBER[state],
        "altitude":  alt,
        "velocity":  velocity,
        "pressure":  1013.25 * pow(max(0, 1 - 2.2557e-5 * alt), 5.2559) + _noise(t, 2, 0.3),
        "temp":      25 - 0.0065 * alt + _noise(t, 3, 0.2),
        "tvoc":      120 + max(0, alt - 200) * 0.1 + _noise(t, 4, 8),
        "eco2":      410 + max(0, alt - 100) * 0.4 + _noise(t, 5, 12),
        "voltage":   max(6.5, 8.42 - (t / MISSION_DURATION) * 1.6 + _noise(t, 6, 0.04)),
        "acc_r":     _noise(t, 10, 0.8 * accel_scale),
        "acc_p":     _noise(t, 11, 0.6 * accel_scale),
        "acc_y":     (velocity * 0.09 if state == "ASCENT" else 0) + _noise(t, 12, 0.5),
        "gyro_r":    _noise(t * 2, 20, 8 * accel_scale),
        "gyro_p":    _noise(t * 2, 21, 8 * accel_scale),
        "gyro_y":    _noise(t * 2, 22, 12 * accel_scale),
        "gyro_spin": gyro_spin,
        "lat":       13.7331 + ascent_frac * 0.00010 + descent_frac * 0.00090
                     + math.cos(t * 0.12) * 0.000015,
        "lon":       80.1850 + ascent_frac * 0.00018 + descent_frac * 0.00050
                     + math.sin(t * 0.15) * 0.000018,
        "sats":      11 + int((math.sin(t * 0.2) + 1) * 0.5),
        "gnss_alt":  max(0.0, alt - 1.2),
        "gnss_time": f"{(clock_sec // 3600) % 24:02d}:"
                     f"{(clock_sec // 60) % 60:02d}:"
                     f"{clock_sec % 60:02d}",
        "rssi":      -42 - max(0, alt) * 0.04 + _noise(t, 99, 2.5),
    }


# Pre-compute all simulation packets at startup
MISSION_DATA = [_build_packet(i / PACKET_HZ) for i in range(TOTAL_PACKETS)]

# Initial mission events — more are appended at runtime (state changes, commands)
MISSION_EVENTS = [
    (0,    "ok",   "Ground link established"),
    (4.1,  "ok",   "State: TEST_MODE"),
    (6.5,  "warn", "Liftoff · ASCENT"),
    (24,   "cyan", "Apogee 720 m · ROCKET_DEPLOY"),
    (28,   "ok",   "Parachute deployed · DESCENT"),
    (75,   "warn", "AEROBREAK_RELEASE"),
    (132,  "cyan", "IMPACT · landing detected"),
]

# Set to True by main() when trial_data.csv is successfully loaded
CSV_MODE = False


# ═══════════════════════════════════════════════════════════════════
# CSV LOADER
# Reads trial_data.csv when it exists next to the script.
# The CSV uses angle-bracket line wrapping and Pa (not hPa) pressure.
# ═══════════════════════════════════════════════════════════════════

def load_csv_data(path: str) -> list:
    """
    Parse trial_data.csv and return a list of packet dicts.

    Returns an empty list on failure so the caller can fall back to
    the built-in simulation.
    """
    with open(path, newline="", encoding="utf-8-sig") as f:
        raw = f.read()

    # Strip the <> wrapper that trial_data.csv puts around every line
    clean_lines = []
    for line in raw.splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        if stripped.startswith("<") and stripped.endswith(">"):
            stripped = stripped[1:-1]
        clean_lines.append(stripped)

    if not clean_lines:
        return []

    reader   = csv.DictReader(clean_lines)
    packets  = []
    prev_alt = 0.0
    prev_state_num = -1

    for i, row in enumerate(reader):
        try:
            alt      = float(row["ALTITUDE"])
            velocity = (alt - prev_alt) if i > 0 else 0.0
            prev_alt = alt

            state_num = int(row["FLIGHT_SOFTWARE_STATE"])
            state     = STATE_NAME.get(state_num, "BOOT")

            # Log state transitions as mission events
            if state_num != prev_state_num and prev_state_num != -1:
                MISSION_EVENTS.append((float(i), "warn", f"State → {state}"))
            prev_state_num = state_num

            packets.append({
                "t":         float(i),                        # 1 row = 1 second
                "packet":    max(0, int(row["PACKET_COUNT"])),
                "state":     state,
                "state_num": state_num,
                "altitude":  max(0.0, alt),
                "velocity":  velocity,
                "pressure":  float(row["PRESSURE"]) / 100.0, # Pa → hPa
                "temp":      float(row["TEMP"]),
                "tvoc":      float(row["TVOC"]),
                "eco2":      float(row["eCO2"]),
                "voltage":   float(row["VOLTAGE"]),
                "acc_r":     float(row["ACC_R"]),
                "acc_p":     float(row["ACC_P"]),
                "acc_y":     float(row["ACC_Y"]),
                "gyro_r":    float(row["GYRO_R"]),
                "gyro_p":    float(row["GYRO_P"]),
                "gyro_y":    float(row["GYRO_Y"]),
                "gyro_spin": 0.0,
                "lat":       float(row["GNSS_LATITUDE"]),
                "lon":       float(row["GNSS_LONGITUDE"]),
                "sats":      int(row["GNSS_SATS"]),
                "gnss_alt":  float(row["GNSS_ALTITUDE"]),
                "gnss_time": row["GNSS_TIME"].strip(),
                "rssi":      -60.0,
            })
        except (ValueError, KeyError):
            continue   # skip malformed rows silently

    return packets


# ═══════════════════════════════════════════════════════════════════
# PLAYBACK ENGINE  (SimState)
# Advances mission time and serves packets on a 20 Hz timer.
# Imported by the UI; the UI connects SimState.updated to _on_tick.
# ═══════════════════════════════════════════════════════════════════

# SimState is a QObject (needs Qt signals), so it is defined in gs_ui.py
# but kept logically separate there with a clear comment.
# All pure-logic helpers live here.


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
    phi1   = math.radians(lat1)
    phi2   = math.radians(lat2)
    dphi   = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    # Haversine formula for arc length
    a        = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    distance = EARTH_RADIUS_KM * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    # Forward azimuth (bearing)
    y       = math.sin(dlambda) * math.cos(phi2)
    x       = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
    bearing = (math.degrees(math.atan2(y, x)) + 360) % 360

    return distance, bearing


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
