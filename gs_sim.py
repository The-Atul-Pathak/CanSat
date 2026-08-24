"""
CanSat Ground Station — hardware simulation layer.  Pure Python, no Qt.

Everything here exists so the ground station can be flown without a CanSat, an
XBee, a rocket or a launch slot.  Nothing in gs_logic.py or gs_ui.py knows it
is being simulated: the virtual radio duck-types the pyserial object the
receiver reads from, so the real parser, the real CSV writer, the real link
timeout and the real display all run exactly as they do on launch day.

    VirtualCanSat      the flight computer — mission state machine, flight
                       dynamics, sensor models, uplink command handling
    VirtualXBee        the radio — paces the downlink, adds timing jitter,
                       loses and corrupts the occasional packet, and carries
                       uplink commands down to the flight computer
    SimulatedReceiver  a TelemetryReceiver whose port is a VirtualXBee

Two different things are called "simulation" in this project, and they nest:

    this module      simulates the *hardware*.  No CanSat, no radio, no rocket.
    SIM EN/ACT/DIS   the competition's simulation mode (§6.1), in which a real
                     CanSat flies on pressures uplinked from the ground.

Both work together.  With the virtual CanSat connected, SIM ACT genuinely makes
it stop trusting its barometer and fly the uplinked altitude profile instead —
the same code path a real CanSat would take.

Layering
--------
The vehicle model is a plain state machine stepped by step(dt); it never reads
the clock.  Wall-clock pacing, jitter and packet loss live in VirtualXBee.  That
split is what lets a whole flight be run in milliseconds by a test while the
same code flies in real time on screen.
"""

import math
import os
import queue
import random
import sys
import threading
import time
from dataclasses import dataclass, field

import gs_logic as logic
from gs_logic import GROUND_STATION, STATE_NUMBER, TEAM_ID


# ═══════════════════════════════════════════════════════════════════
# ATMOSPHERE & SITE
# Constants describing the launch site the rehearsal flies from.
# ═══════════════════════════════════════════════════════════════════

SEA_LEVEL_HPA   = 1013.25   # QNH used to turn altitude back into pressure
GROUND_ELEV_M   = 216.0     # site elevation above mean sea level
GROUND_TEMP_C   = 31.4      # outside air temperature at the pad
LAPSE_C_PER_M   = 0.0065    # standard atmosphere temperature lapse rate
G               = 9.80665

# Where the pad sits relative to the ground station: far enough that the
# recovery distance readout is a real number, close enough to walk.
PAD_RANGE_M   = 180.0
PAD_BEARING   = 42.0

METRES_PER_DEG_LAT = 111_320.0


def pressure_at(altitude_agl: float) -> float:
    """Barometric pressure in hPa at an altitude above the launch site."""
    h = GROUND_ELEV_M + altitude_agl
    return SEA_LEVEL_HPA * (1.0 - 2.25577e-5 * h) ** 5.25588


# ═══════════════════════════════════════════════════════════════════
# CONFIGURATION
# Everything the operator can tune from the command line.
# ═══════════════════════════════════════════════════════════════════

@dataclass
class SimConfig:
    """One rehearsal's parameters.  Defaults describe a nominal mission."""

    speed:      float = 1.0     # wall-clock time scale (2.0 = twice as fast)
    packet_hz:  float = 1.0     # downlink rate
    boot_s:     float = 6.0     # power-up self-test before TEST_MODE
    test_s:     float = 14.0    # prelaunch checks before the pad hold
    pad_hold_s: float = 40.0    # pad wait after CX ON, before the motor lights
    apex_m:     float = 1000.0  # target apex above the pad
    burn_s:     float = 2.6     # motor burn time

    descent_mps:   float = 20.0  # descent rate before the aerobrake releases
    aerobrake_mps: float = 6.5   # descent rate after it releases

    wind_mps:      float = 4.5   # steady wind speed
    wind_from_deg: float = 250.0 # direction the wind blows *from*

    loss:        float = 0.012  # fraction of packets the radio never delivers
    corrupt:     float = 0.004  # fraction that arrive mangled
    blackout_s:  float = 6.5    # radio null while the CanSat tumbles clear
    jitter_s:    float = 0.035  # packet-to-packet timing spread

    seed: int = None            # fix for a byte-identical rehearsal

    # Derived at construction; see __post_init__.
    pad_lat: float = field(init=False, default=0.0)
    pad_lon: float = field(init=False, default=0.0)

    def __post_init__(self):
        east  = PAD_RANGE_M * math.sin(math.radians(PAD_BEARING))
        north = PAD_RANGE_M * math.cos(math.radians(PAD_BEARING))
        self.pad_lat = GROUND_STATION[0] + north / METRES_PER_DEG_LAT
        self.pad_lon = GROUND_STATION[1] + east / self.m_per_deg_lon()

    def m_per_deg_lon(self) -> float:
        return METRES_PER_DEG_LAT * math.cos(math.radians(GROUND_STATION[0]))

    def offset_to_latlon(self, east_m: float, north_m: float):
        """Turn a metres-from-pad offset into a (latitude, longitude) pair."""
        return (self.pad_lat + north_m / METRES_PER_DEG_LAT,
                self.pad_lon + east_m / self.m_per_deg_lon())


# ═══════════════════════════════════════════════════════════════════
# THE VIRTUAL CANSAT
# Flight computer, flight dynamics and every sensor on the vehicle.
# ═══════════════════════════════════════════════════════════════════

# Drag coefficients per unit mass (1/m): the rocket on the way up, and the
# CanSat tumbling in free air after separation.  The ascent value shapes the
# coast; _thrust_accel() then picks a motor to match, so the apex stays on the
# configured target whatever this is set to.
ASCENT_DRAG_K = 0.0012
TUMBLE_DRAG_K = 0.0020

# Altitude at which the aerobrake is commanded (§6.1) and at which the vehicle
# is declared down.
AEROBRAKE_ALT = logic.SECONDARY_TRIGGER_ALT
TOUCHDOWN_ALT = 0.4

# How long after separation the tumble actually swings the antenna off the
# ground station.  Long enough that ROCKET_DEPLOY reaches the display first.
BLACKOUT_LEAD_S = 1.6


class VirtualCanSat:
    """
    A CanSat that exists only in memory.

    Advanced by step(dt) in mission seconds — it never reads the clock itself,
    so the same model can fly in real time on screen or in a few milliseconds
    inside a test.  telemetry_line() renders the current instant in exactly the
    wire format gs_logic.PacketParser expects, and command() accepts exactly
    the ASCII strings gs_logic.COMMANDS builds.
    """

    def __init__(self, config: SimConfig = None, rng: random.Random = None):
        self.cfg = config or SimConfig()
        self.rng = rng or random.Random(self.cfg.seed)
        self.notices = []          # (severity, text) — drained by the radio
        self.power_on()

    # ── power-up / reset ──────────────────────────────────────────

    def power_on(self):
        """Cold boot.  Also what CMD,BOOT does, which is the point of it."""
        self.t       = 0.0         # seconds since power-up (TIME_STAMPING)
        self.packet  = 0           # PACKET_COUNT, counts what we transmit
        self.state   = "BOOT"
        self.phase_t = 0.0         # seconds in the current state

        # Flight dynamics — altitude above the pad and the two horizontal axes
        self.alt   = 0.0
        self.vz    = 0.0
        self.east  = 0.0
        self.north = 0.0

        # Downlink and mode flags set by uplinked commands
        self.cx          = False
        self.launch_at   = None    # mission time the motor lights
        self.sim_enabled = False
        self.sim_active  = False
        self.sim_alt     = None    # last altitude uplinked by CMD,SIMP

        # Sensors.  The biases are what CMD,CALIBRATE exists to remove, so they
        # start deliberately non-zero: an uncalibrated CanSat reads a couple of
        # metres high and its gyro creeps.
        self.baro_bias = 2.7
        self.gyro_bias = [1.8, -0.9, 0.42]
        self.calibrated = False

        self.temp    = GROUND_TEMP_C
        self.voltage = 8.28        # 2S Li-ion, freshly charged
        self.tvoc    = 0.0         # CCS811 outputs nothing until it warms up
        self.eco2    = 400.0
        self.sats    = 0
        self.acc     = [0.0, 0.0, 1.0]    # upright on the pad: 1 g on the body axis
        self.gyro    = [0.0, 0.0, 0.0]
        self.spin    = 0.0         # mechanical gyro spin rate (§6.3 item 14)

        # GNSS clock — real UTC at power-up unless SET_TIME overrides it
        self.utc0 = (time.time() % 86400.0)

        self.blackout = None          # (start, end) mission times of a radio null

        # Sized here rather than at ignition, so the search never lands as a
        # hitch on the reader thread in the middle of the boost.
        self._thrust_cache = None
        self._thrust_accel()

        self._notice("ok", "CanSat powered up — BOOT self-test")

    def _notice(self, severity: str, text: str):
        self.notices.append((severity, text))

    # ── the mission clock ─────────────────────────────────────────

    def step(self, dt: float):
        """Advance the whole vehicle by dt mission seconds."""
        self.t       += dt
        self.phase_t += dt
        self._step_dynamics(dt)
        self._step_state()
        self._step_sensors(dt)

    # ── flight dynamics ───────────────────────────────────────────

    def _apex_for(self, thrust: float) -> float:
        """Fly the ascent on paper and report where it tops out."""
        alt = v = t = 0.0
        dt  = 0.02
        while t < 120.0:
            a  = (thrust if t < self.cfg.burn_s else 0.0) - G \
                 - ASCENT_DRAG_K * v * abs(v)
            v   += a * dt
            alt += v * dt
            t   += dt
            if v <= 0.0 and t > self.cfg.burn_s:
                break
        return alt

    def _thrust_accel(self) -> float:
        """
        Motor acceleration that puts the apex on cfg.apex_m.

        Drag makes the closed-form answer wrong by a good 30 %, so the motor is
        sized by bisecting the ascent above instead — a couple of hundred
        integration steps, once, at power-up.  It keeps the apex honest for any
        --apex the operator asks for rather than only the default.
        """
        if self._thrust_cache is not None:
            return self._thrust_cache
        low, high = G, 400.0
        for _ in range(40):
            mid = (low + high) / 2.0
            if self._apex_for(mid) < self.cfg.apex_m:
                low = mid
            else:
                high = mid
        self._thrust_cache = (low + high) / 2.0
        return self._thrust_cache

    def _step_dynamics(self, dt: float):
        if self.sim_active and self.sim_alt is not None:
            # Flying on uplinked pressure: the barometer follows the commanded
            # altitude with the lag of a real sensor rather than snapping to it.
            previous  = self.alt
            self.alt += (self.sim_alt - self.alt) * min(1.0, dt / 0.45)
            self.vz   = (self.alt - previous) / dt if dt > 0 else 0.0
        elif self.state in ("BOOT", "TEST_MODE", "LAUNCH_PAD"):
            self.alt = 0.0
            self.vz  = 0.0
        elif self.state == "ASCENT":
            tau     = self.t - (self.launch_at or self.t)
            thrust  = self._thrust_accel() if tau < self.cfg.burn_s else 0.0
            self.vz += (thrust - G - ASCENT_DRAG_K * self.vz * abs(self.vz)) * dt
            self.alt = max(0.0, self.alt + self.vz * dt)
        elif self.state == "ROCKET_DEPLOY":
            # Ejected and tumbling: nearly free fall, drag has barely bitten.
            self.vz += (-G - TUMBLE_DRAG_K * self.vz * abs(self.vz)) * dt
            self.alt = max(0.0, self.alt + self.vz * dt)
        elif self.state in ("DESCENT", "AEROBREAK_RELEASE"):
            target   = (-self.cfg.descent_mps if self.state == "DESCENT"
                        else -self.cfg.aerobrake_mps)
            self.vz += (target - self.vz) * min(1.0, dt / 1.4)
            self.alt = max(0.0, self.alt + self.vz * dt)
        else:                                   # IMPACT — down and staying down
            self.alt = 0.0
            self.vz  = 0.0

        self._step_horizontal(dt)

    def _step_horizontal(self, dt: float):
        """
        Wind drift.  How much of it the vehicle picks up depends on what it is
        hanging from: a boosting rocket barely notices, a descending CanSat goes
        wherever the air goes.
        """
        coupling = {"ASCENT": 0.25, "ROCKET_DEPLOY": 0.8,
                    "DESCENT": 1.0, "AEROBREAK_RELEASE": 1.0}.get(self.state, 0.0)
        toward = math.radians((self.cfg.wind_from_deg + 180.0) % 360.0)
        wind_e = self.cfg.wind_mps * math.sin(toward)
        wind_n = self.cfg.wind_mps * math.cos(toward)

        # The rocket also flies slightly downrange off a tilted rail.
        downrange = 0.0
        if self.state == "ASCENT":
            tau       = self.t - (self.launch_at or self.t)
            downrange = 7.0 * math.exp(-tau / 4.0)
        rail = math.radians(70.0)

        self.east  += (wind_e * coupling + downrange * math.sin(rail)) * dt
        self.north += (wind_n * coupling + downrange * math.cos(rail)) * dt

    # ── mission state machine ─────────────────────────────────────

    def _enter(self, state: str, severity: str, text: str):
        self.state   = state
        self.phase_t = 0.0
        self._notice(severity, text)

    def _step_state(self):
        cfg = self.cfg

        if self.state == "BOOT":
            if self.t >= cfg.boot_s:
                self._enter("TEST_MODE", "ok",
                            "Self-test passed — TEST MODE, running prelaunch checks")

        elif self.state == "TEST_MODE":
            if self.t >= cfg.boot_s + cfg.test_s:
                self._enter("LAUNCH_PAD", "ok",
                            "Prelaunch checks complete — armed and standing by on the pad")
                if not self.calibrated:
                    self._notice("warn", "Barometer still shows a pad offset — "
                                         "send CALIBRATE before launch")

        elif self.state == "LAUNCH_PAD":
            launched = self.launch_at is not None and self.t >= self.launch_at
            if launched or self.alt > 5.0:      # alt > 5 catches SIM mode
                self._enter("ASCENT", "warn", "MOTOR IGNITION — liftoff")
                self.launch_at = self.t

        elif self.state == "ASCENT":
            if self.vz <= 0.0 and self.alt > 20.0:
                self._enter("ROCKET_DEPLOY", "cyan",
                            f"Apex {self.alt:.0f} m — separation, CanSat ejected")
                # Tumbling clear of the rocket swings the antenna through a
                # null.  It starts a moment after separation rather than at it,
                # so the operator sees ROCKET_DEPLOY arrive before the link
                # drops out and comes back somewhere in the descent.
                if cfg.blackout_s > 0:
                    self.blackout = (self.t + BLACKOUT_LEAD_S,
                                     self.t + BLACKOUT_LEAD_S + cfg.blackout_s)

        elif self.state == "ROCKET_DEPLOY":
            if self.phase_t >= 2.5:
                self._enter("DESCENT", "warn",
                            f"Primary descent — targeting "
                            f"{cfg.descent_mps:.0f} m/s")

        elif self.state == "DESCENT":
            if self.alt <= AEROBRAKE_ALT:
                self._enter("AEROBREAK_RELEASE", "ok",
                            f"AEROBRAKE RELEASED at {AEROBRAKE_ALT:.0f} m — "
                            f"descent slowing")

        elif self.state == "AEROBREAK_RELEASE":
            if self.alt <= TOUCHDOWN_ALT:
                self._enter("IMPACT", "cyan", "IMPACT — audio beacon ON")

    # ── sensors ───────────────────────────────────────────────────

    def _step_sensors(self, dt: float):
        self._step_gnss(dt)
        self._step_imu(dt)
        self._step_air(dt)
        self._step_power(dt)

        # Outside air cools with altitude; the electronics bay runs a little
        # warm and the difference shrinks once the CanSat is out in the airflow.
        outside  = GROUND_TEMP_C - LAPSE_C_PER_M * self.alt
        internal = 3.4 if self.state in ("BOOT", "TEST_MODE", "LAUNCH_PAD") else 1.1
        self.temp += ((outside + internal) - self.temp) * min(1.0, dt / 4.0)

    def _step_gnss(self, dt: float):
        """
        Satellite count.  A cold receiver climbs to a full constellation over
        the first half-minute, then loses a few every time the vehicle starts
        moving violently — which is exactly when the operator looks at it.
        """
        if self.t < 4.0:
            target = 0
        elif self.t < 9.0:
            target = 4
        elif self.t < 15.0:
            target = 8
        elif self.t < 24.0:
            target = 11
        else:
            target = 12
        if self.state in ("ASCENT", "ROCKET_DEPLOY"):
            target -= 5                       # high dynamics, antenna swinging
        elif self.state == "IMPACT":
            target -= 2                       # lying on its side in the grass

        target = max(0, target)
        if abs(self.sats - target) >= 1 and self.rng.random() < dt * 1.6:
            self.sats += 1 if target > self.sats else -1
        elif self.rng.random() < dt * 0.25:
            self.sats = max(0, min(14, self.sats + self.rng.choice((-1, 1))))

    def _step_imu(self, dt: float):
        """
        Accelerometer, rate gyro and the mechanical spin rate.

        Axes follow gs_logic.attitude_from_accel(): ACC_Y is the body axis, so
        an upright CanSat reads +1 g there and nothing on the other two.
        """
        rng   = self.rng
        state = self.state
        n     = lambda s: rng.gauss(0.0, s)

        if state in ("BOOT", "TEST_MODE", "LAUNCH_PAD"):
            # Sitting still: a whisper of vibration, nothing else.
            self.acc  = [n(0.006), n(0.006), 1.0 + n(0.006)]
            self.gyro = [b + n(0.05) for b in self.gyro_bias]
            self.spin = 0.0

        elif state == "ASCENT":
            tau   = self.t - (self.launch_at or self.t)
            if tau < self.cfg.burn_s:
                axial = 1.0 + self._thrust_accel() / G
                shake = 0.35
                roll  = 110.0 * min(1.0, tau / 1.2)      # spin-stabilised rail exit
            else:
                axial = ASCENT_DRAG_K * self.vz * abs(self.vz) / G
                shake = 0.05
                roll  = 110.0 * math.exp(-(tau - self.cfg.burn_s) / 9.0)
            self.acc  = [n(shake), n(shake), axial + n(shake)]
            self.gyro = [roll + n(4.0), n(6.0), n(6.0)]
            self.spin = 0.0

        elif state == "ROCKET_DEPLOY":
            # Free tumble, decaying as the aerodynamics take hold.
            decay = math.exp(-self.phase_t / 2.2)
            w     = 2 * math.pi * 1.4 * self.phase_t
            self.acc  = [1.4 * decay * math.sin(w) + n(0.12),
                         1.4 * decay * math.cos(w * 0.8) + n(0.12),
                         0.25 + 0.9 * decay * math.sin(w * 1.3) + n(0.12)]
            self.gyro = [280 * decay * math.sin(w * 0.9) + n(9),
                         220 * decay * math.cos(w * 1.1) + n(9),
                         240 * decay * math.sin(w * 0.7) + n(9)]
            # The mechanical gyro spins up as soon as it is in free air.
            self.spin += (330.0 - self.spin) * min(1.0, dt / 3.0)

        elif state in ("DESCENT", "AEROBREAK_RELEASE"):
            # Hanging and swinging: about 1 g down, with a pendulum on top.
            swing  = 2 * math.pi * self.phase_t / 5.5
            amp    = 0.18 if state == "DESCENT" else 0.10
            self.acc  = [amp * math.sin(swing) + n(0.03),
                         amp * math.cos(swing * 1.1) + n(0.03),
                         1.0 + 0.05 * math.sin(swing * 2) + n(0.03)]
            yaw = -38.0 if state == "DESCENT" else -95.0
            self.gyro = [25 * math.sin(swing) + n(2.0),
                         22 * math.cos(swing * 1.1) + n(2.0),
                         yaw + n(3.0)]
            target = 330.0 if state == "DESCENT" else 355.0
            self.spin += (target - self.spin) * min(1.0, dt / 4.0)

        else:                                    # IMPACT
            if self.phase_t < 0.9:               # the hit itself
                self.acc  = [n(2.0), n(2.0), 11.0 + n(2.0)]
                self.gyro = [n(70), n(70), n(70)]
            else:                                # come to rest, tipped over
                self.acc  = [0.34 + n(0.004), 0.12 + n(0.004), 0.93 + n(0.004)]
                self.gyro = [n(0.06), n(0.06), n(0.06)]
            self.spin = max(0.0, self.spin - 180.0 * dt)

        # The gyro bias survives until CALIBRATE removes it — the whole reason
        # the button exists.  It is already folded into the pad case above.
        if state not in ("BOOT", "TEST_MODE", "LAUNCH_PAD"):
            self.gyro = [g + b for g, b in zip(self.gyro, self.gyro_bias)]

    def _step_air(self, dt: float):
        """
        CCS811 air quality.  It reports nothing at all until it has warmed up,
        sits high in the exhaust and the crowd at the pad, and reads clean air
        once the CanSat is a few hundred metres up.
        """
        warmed = self.t > 18.0
        if not warmed:
            self.tvoc, self.eco2 = 0.0, 400.0
            return

        if self.state in ("BOOT", "TEST_MODE", "LAUNCH_PAD"):
            tvoc_t, eco2_t = 95.0, 640.0
        elif self.state == "ASCENT" and self.t - (self.launch_at or 0.0) < 6.0:
            tvoc_t, eco2_t = 1100.0, 1650.0            # motor exhaust
        else:
            clean  = max(0.0, min(1.0, self.alt / 350.0))
            tvoc_t = 95.0 * (1 - clean) + 6.0 * clean
            eco2_t = 560.0 * (1 - clean) + 405.0 * clean

        self.tvoc += (tvoc_t - self.tvoc) * min(1.0, dt / 6.0)
        self.eco2 += (eco2_t - self.eco2) * min(1.0, dt / 6.0)
        self.tvoc = max(0.0, self.tvoc + self.rng.gauss(0, 2.5))
        self.eco2 = max(400.0, self.eco2 + self.rng.gauss(0, 4.0))

    def _step_power(self, dt: float):
        """
        Battery.  A steady housekeeping draw, extra while the motor-side
        pyros and servos are live, and a heavier draw once the recovery beacon
        starts — which is what eventually trips the low-voltage warning if the
        rehearsal is left running after landing.
        """
        drain = 0.0016
        if self.state in ("ASCENT", "ROCKET_DEPLOY"):
            drain += 0.010                      # servos, pyro bus, camera
        if self.state == "IMPACT":
            drain += 0.0080                     # audio beacon
        self.voltage = max(6.55, self.voltage - drain * dt)

    # ── telemetry ─────────────────────────────────────────────────

    @property
    def gnss_time(self) -> str:
        """UTC from the GNSS receiver, as HH:MM:SS."""
        total = (self.utc0 + self.t) % 86400.0
        return (f"{int(total // 3600):02d}:"
                f"{int(total % 3600 // 60):02d}:"
                f"{int(total % 60):02d}")

    def position(self):
        """(latitude, longitude) including parachute swing and GNSS noise."""
        swing_e = swing_n = 0.0
        if self.state in ("DESCENT", "AEROBREAK_RELEASE"):
            phase   = 2 * math.pi * self.phase_t / 8.0
            swing_e = 14.0 * math.sin(phase)
            swing_n = 14.0 * math.cos(phase * 0.7)
        # A receiver with few satellites is a receiver you should not trust.
        sigma = 1.6 if self.sats >= 8 else 6.0
        lat, lon = self.cfg.offset_to_latlon(self.east + swing_e,
                                             self.north + swing_n)
        return (lat + self.rng.gauss(0, sigma) / METRES_PER_DEG_LAT,
                lon + self.rng.gauss(0, sigma) / self.cfg.m_per_deg_lon())

    def telemetry(self) -> dict:
        """Every transmitted field at this instant, in engineering units."""
        lat, lon = self.position()
        alt      = self.alt + self.baro_bias + self.rng.gauss(0, 0.18)
        return {
            "team_id":   TEAM_ID,
            "t":         self.t,
            "packet":    self.packet,
            "altitude":  alt,
            "pressure":  pressure_at(alt) * 100.0,      # hPa → Pa, as on the wire
            "temp":      self.temp + self.rng.gauss(0, 0.08),
            "voltage":   self.voltage + self.rng.gauss(0, 0.006),
            "gnss_time": self.gnss_time,
            "lat":       lat,
            "lon":       lon,
            "gnss_alt":  GROUND_ELEV_M + self.alt + self.rng.gauss(0, 1.4),
            "sats":      self.sats,
            "acc":       self.acc,
            "gyro":      self.gyro,
            "state_num": STATE_NUMBER[self.state],
            "tvoc":      self.tvoc,
            "eco2":      self.eco2,
            "spin":      self.spin,
        }

    def telemetry_line(self) -> str:
        """
        One downlink line, in the field order gs_logic._REQUIRED_FIELDS parses.
        Resolutions match the §6.3 table, so what the CanSat "sends" is exactly
        as coarse as the real thing.
        """
        d = self.telemetry()
        return ",".join(str(v) for v in [
            d["team_id"], f"{d['t']:.1f}", d["packet"],
            f"{d['altitude']:.1f}", f"{d['pressure']:.0f}", f"{d['temp']:.1f}",
            f"{d['voltage']:.2f}", d["gnss_time"],
            f"{d['lat']:.5f}", f"{d['lon']:.5f}", f"{d['gnss_alt']:.1f}",
            d["sats"],
            *(f"{v:.2f}" for v in d["acc"]),
            *(f"{v:.2f}" for v in d["gyro"]),
            d["state_num"], f"{d['tvoc']:.0f}", f"{d['eco2']:.0f}",
            f"{d['spin']:.1f}",
        ])

    # ── uplink ────────────────────────────────────────────────────

    def command(self, text: str):
        """
        Act on one uplinked command string.

        Accepts exactly what gs_logic.build_command() produces.  Unknown or
        out-of-sequence commands are rejected the way flight software would
        reject them — with a reason, not silently.
        """
        parts = [p.strip() for p in text.strip().split(",")]
        if not parts or parts[0].upper() != "CMD" or len(parts) < 2:
            self._notice("warn", f"CanSat: unrecognised uplink '{text.strip()}'")
            return
        verb = parts[1].upper()
        arg  = parts[2].upper() if len(parts) > 2 else ""

        if verb == "BOOT":
            self._notice("warn", "CanSat: CMD,BOOT — flight software restarting")
            self.power_on()

        elif verb == "SETTIME":
            self._set_time(parts[2] if len(parts) > 2 else "")

        elif verb == "CALIBRATE":
            self._calibrate()

        elif verb == "CX":
            self._set_cx(arg == "ON")

        elif verb == "SIM":
            self._sim_mode(arg)

        elif verb == "SIMP":
            self._sim_pressure(parts[2] if len(parts) > 2 else "")

        else:
            self._notice("warn", f"CanSat: unknown command '{verb}'")

    def _set_time(self, value: str):
        try:
            h, m, s = (float(x) for x in value.split(":"))
        except ValueError:
            self._notice("warn", f"CanSat: SETTIME rejected — bad time '{value}'")
            return
        # utc0 is the clock at power-up, so back the mission time out of it.
        self.utc0 = (h * 3600 + m * 60 + s - self.t) % 86400.0
        self._notice("ok", f"CanSat: clock set — GNSS time now {self.gnss_time} UTC")

    def _calibrate(self):
        if self.state not in ("BOOT", "TEST_MODE", "LAUNCH_PAD"):
            self._notice("warn", "CanSat: CALIBRATE refused — vehicle is in flight")
            return
        bias = self.baro_bias
        self.baro_bias  = round(self.rng.gauss(0.0, 0.05), 3)
        self.gyro_bias  = [round(self.rng.gauss(0.0, 0.02), 3) for _ in range(3)]
        self.calibrated = True
        self._notice("ok", f"CanSat: calibrated — baro zeroed ({bias:+.1f} m removed), "
                           f"gyro bias nulled")

    def _set_cx(self, on: bool):
        if on == self.cx:
            self._notice("ok", f"CanSat: CX already {'ON' if on else 'OFF'}")
            return
        self.cx = on
        if not on:
            self._notice("warn", "CanSat: CX OFF — telemetry downlink stopped")
            return
        self._notice("ok", "CanSat: CX ON — telemetry downlink started")
        # The countdown only runs while the ground station is listening; that
        # is what makes the pad wait something the operator actually watches.
        if self.launch_at is None and self.state in ("BOOT", "TEST_MODE", "LAUNCH_PAD"):
            self.launch_at = max(self.t, self.cfg.boot_s + self.cfg.test_s) \
                             + self.cfg.pad_hold_s
            self._notice("cyan", f"Range clear — launch in "
                                 f"{self.launch_at - self.t:.0f} s")

    def _sim_mode(self, arg: str):
        if arg == "ENABLE":
            self.sim_enabled = True
            self._notice("ok", "CanSat: SIM mode ARMED — awaiting ACTIVATE")
        elif arg == "ACTIVATE":
            if not self.sim_enabled:
                self._notice("warn", "CanSat: SIM ACTIVATE refused — not enabled "
                                     "(send SIM EN first)")
                return
            self.sim_active = True
            self.sim_alt    = self.alt
            self._notice("cyan", "CanSat: SIM ACTIVE — flying on uplinked pressure, "
                                 "barometer ignored")
        elif arg == "DISABLE":
            was = self.sim_active
            self.sim_enabled = self.sim_active = False
            self.sim_alt     = None
            self._notice("ok", "CanSat: SIM disabled — back on the real barometer"
                               if was else "CanSat: SIM mode disarmed")
        else:
            self._notice("warn", f"CanSat: unknown SIM argument '{arg}'")

    def _sim_pressure(self, value: str):
        if not self.sim_active:
            return                       # uplink arriving before ACTIVATE
        try:
            self.sim_alt = float(value)
        except ValueError:
            self._notice("warn", f"CanSat: SIMP rejected — bad value '{value}'")

    # ── ground-station-facing helpers ─────────────────────────────

    def slant_range_m(self) -> float:
        """Straight-line distance from the ground station, for the link budget."""
        lat, lon = self.cfg.offset_to_latlon(self.east, self.north)
        ground, _ = logic.haversine(GROUND_STATION[0], GROUND_STATION[1], lat, lon)
        return math.hypot(ground * 1000.0, self.alt)

    def rssi_dbm(self) -> float:
        """
        Received signal strength an XBee would report at this range.

        Free-space path loss referenced to a measured -42 dBm at 100 m, plus a
        few dB of fade so the number moves the way a real one does.
        """
        r = max(20.0, self.slant_range_m())
        return -42.0 - 20.0 * math.log10(r / 100.0) + self.rng.gauss(0, 1.2)


# ═══════════════════════════════════════════════════════════════════
# THE VIRTUAL XBEE
# Duck-types the pyserial object TelemetryReceiver reads from and writes to.
# ═══════════════════════════════════════════════════════════════════

# What the port is called in the dropdown, per platform, so the label looks
# like something you would actually see after plugging a radio in.
def _fake_port() -> tuple:
    if sys.platform.startswith("win"):
        device = "COM7"
    elif sys.platform == "darwin":
        device = "/dev/cu.usbserial-XB0056"
    else:
        device = "/dev/ttyUSB0"
    return device, (f"{os.path.basename(device)} — Digi XBee-PRO S2C 2.4 GHz "
                    f"[SIMULATED]")


class VirtualXBee:
    """
    A radio with no antenna.

    Implements the three methods TelemetryReceiver uses on its port —
    readline(), write() and close() — and nothing else, because nothing else is
    ever called.  readline() blocks briefly like a real one with a read timeout,
    which is what keeps the reader thread from spinning.

    All the ways a downlink is imperfect live here rather than in the vehicle:
    timing jitter, packets that are transmitted but never arrive, packets that
    arrive mangled, and the antenna null while the CanSat tumbles clear of the
    rocket.  The vehicle counts every packet it transmits, so the ones the air
    eats show up in the ground station's own lost-packet counter — the same
    arithmetic that runs on a real flight.
    """

    POLL_S = 0.04        # how long readline() waits before giving up on a poll

    def __init__(self, config: SimConfig = None, rng: random.Random = None):
        self.cfg  = config or SimConfig()
        self.rng  = rng or random.Random(self.cfg.seed)
        self.sat  = VirtualCanSat(self.cfg, self.rng)
        self.notices = queue.Queue()      # (severity, text) for the event log

        self._lock     = threading.Lock()
        self._closed   = False
        self._wall     = time.monotonic()
        self._next_due = 0.0              # mission time of the next downlink
        self._received = []               # lines that arrived, awaiting readline()
        self._drain_notices()

    # ── the reader side ───────────────────────────────────────────

    def readline(self) -> bytes:
        """
        Return the next downlink line, or b"" if none is waiting.

        Sleeps only when there is nothing to hand over, so the reader thread
        neither spins nor falls behind, and never blocks for longer than POLL_S
        — exactly like pyserial's own read timeout.
        """
        if self._closed:
            return b""
        with self._lock:
            self._advance()
            line = self._received.pop(0) if self._received else ""
        self._drain_notices()
        if not line:
            time.sleep(self.POLL_S)
        return line.encode("ascii") if line else b""

    def _advance(self):
        """
        Run the vehicle forward to now, in steps small enough to integrate,
        transmitting whenever a packet falls due along the way.

        Packets are paced by *mission* time rather than by how often readline()
        happens to be called, so --speed changes how fast the mission runs
        without changing the one-second spacing of the telemetry itself.
        """
        now     = time.monotonic()
        elapsed = (now - self._wall) * self.cfg.speed
        self._wall = now
        # A machine that was asleep must not teleport the vehicle downrange.
        elapsed = min(elapsed, 2.0)
        while elapsed > 1e-6:
            dt = min(0.05, elapsed)
            self.sat.step(dt)
            elapsed -= dt
            self._transmit_due()

    def _transmit_due(self):
        """Transmit every packet the vehicle owes at its current mission time."""
        sat = self.sat
        if not sat.cx:
            self._next_due = sat.t          # stay ready to resume instantly
            return

        while sat.t >= self._next_due:
            interval = 1.0 / self.cfg.packet_hz
            self._next_due += max(0.05, interval
                                  + self.rng.gauss(0, self.cfg.jitter_s))

            # The CanSat counts what it transmits, whether or not it arrives.
            sat.packet += 1
            line = sat.telemetry_line()

            blacked_out = (sat.blackout is not None
                           and sat.blackout[0] <= sat.t < sat.blackout[1])
            if blacked_out or self.rng.random() < self.cfg.loss:
                continue                    # transmitted, never received
            if self.rng.random() < self.cfg.corrupt:
                line = self._corrupt(line)
            self._received.append(line + "\r\n")

    def _corrupt(self, line: str) -> str:
        """
        Mangle one field, the way a bit error off a weak link looks.  Never the
        team ID or the packet count, so the line still arrives recognisably ours
        and the ground station's dropped-packet arithmetic still adds up.
        """
        fields    = line.split(",")
        i         = self.rng.randrange(3, len(fields))
        fields[i] = fields[i][:-1] + self.rng.choice("?#@x")
        return ",".join(fields)

    # ── the writer side ───────────────────────────────────────────

    def write(self, data: bytes) -> int:
        """Carry one uplink command down to the flight computer."""
        if self._closed:
            return 0
        text = data.decode("ascii", errors="ignore")
        with self._lock:
            self._advance()
            for command in text.splitlines():
                if command.strip():
                    self.sat.command(command)
        self._drain_notices()
        return len(data)

    def close(self):
        self._closed = True

    # ── housekeeping ──────────────────────────────────────────────

    def _drain_notices(self):
        """Move the vehicle's log lines onto a thread-safe queue for the UI."""
        with self._lock:
            pending, self.sat.notices = self.sat.notices, []
        for item in pending:
            self.notices.put(item)


# ═══════════════════════════════════════════════════════════════════
# THE SIMULATED RECEIVER
# A TelemetryReceiver that opens a VirtualXBee instead of a USB port.
# ═══════════════════════════════════════════════════════════════════

class SimulatedReceiver(logic.TelemetryReceiver):
    """
    The real receiver, with one thing swapped out.

    Everything that matters — the reader thread, the parser, the dropped-packet
    arithmetic, the raw capture, the flight CSV, the link timeout — is inherited
    untouched, so what the operator exercises in a rehearsal is the same code
    that flies.  Only _open_port(), the port list and the two cosmetic strings
    the display asks for are overridden.
    """

    # A rehearsal must never overwrite the graded flight file.
    csv_filename = "SIM_" + logic.CSV_FILENAME

    def __init__(self, config: SimConfig = None):
        super().__init__()
        self.cfg   = config or SimConfig()
        self.radio = None

    # ── what the display asks about the link ──────────────────────

    def available_ports(self, include_all: bool = False) -> list:
        device, label = _fake_port()
        ports = [(device, label)]
        if include_all:                     # the built-ins a laptop always has
            ports.append(("/dev/cu.Bluetooth-Incoming-Port",
                          "cu.Bluetooth-Incoming-Port — built-in"))
        return ports

    def unavailable_reason(self) -> str:
        return ""                           # a virtual radio needs no pyserial

    def link_note(self) -> str:
        """
        The radio detail the dock shows beside the packet counts.

        Doubles as the answer to "why is nothing on screen?": before CX ON the
        CanSat is powered but silent, and saying so is better than leaving the
        operator to work it out from an empty graph.
        """
        if self.radio is None:
            return ""
        sat = self.radio.sat
        if not sat.cx:
            return "SIM · CanSat powered, downlink OFF — press CX ON"
        return (f"SIM · XBee S2C {sat.rssi_dbm():.0f} dBm · "
                f"{sat.slant_range_m():.0f} m slant · 9600 8N1")

    # ── the port itself ───────────────────────────────────────────

    def _open_port(self, port: str, baud: int):
        time.sleep(0.25)                    # a real adapter takes a moment
        self.radio = VirtualXBee(self.cfg)
        return self.radio

    def disconnect(self):
        super().disconnect()
        self.radio = None

    # ── event log bridge ──────────────────────────────────────────

    def drain_notices(self) -> list:
        """
        Collect what the virtual CanSat has said since the last call, as
        (severity, text) pairs ready for MissionData.log().

        The vehicle cannot write to the mission log itself: it lives on the
        reader thread and the log belongs to the UI.  The entrypoint pumps this
        on a timer instead, which keeps the crossing in one obvious place.
        """
        if self.radio is None:
            return []
        out = []
        while True:
            try:
                out.append(self.radio.notices.get_nowait())
            except queue.Empty:
                return out


# ═══════════════════════════════════════════════════════════════════
# HEADLESS FLIGHT
# Used by the self-test, and handy for checking a profile change quickly.
# ═══════════════════════════════════════════════════════════════════

def fly(config: SimConfig = None, seconds: float = 400.0, dt: float = 0.1,
        cx_at: float = 1.0):
    """
    Run a whole mission with no radio, no threads and no clock.

    Returns (packets, notices): one telemetry dict per downlink second, and the
    vehicle's own log lines.  Stops early once the CanSat has been down for
    20 s, so a nominal flight takes a few milliseconds.
    """
    sat      = VirtualCanSat(config or SimConfig())
    packets  = []
    next_due = 0.0
    landed_t = None

    while sat.t < seconds:
        sat.step(dt)
        if cx_at is not None and not sat.cx and sat.t >= cx_at:
            sat.command(logic.build_command("CX_ON"))
        if sat.cx and sat.t >= next_due:
            next_due   = sat.t + 1.0
            sat.packet += 1
            packets.append(sat.telemetry())
        if sat.state == "IMPACT":
            landed_t = landed_t if landed_t is not None else sat.t
            if sat.t - landed_t > 20.0:
                break

    return packets, list(sat.notices)
