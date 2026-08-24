# CanSat Ground Station — Team Kalpana

Ground station software for the IN-SPACe CAN-7USAT 2026 CanSat competition.
Receives live telemetry from the CanSat over an XBee radio, displays it, and
records the graded flight CSV.

Runs on **Windows, macOS and Linux**.

---

## What it does

- Reads telemetry from the XBee on a USB serial port, one packet per second
- Validates every packet and counts anything the radio link dropped
- Displays altitude, velocity, GNSS position, attitude, IMU and power
- Draws a live graph of every telemetry field
- Plots the ground track on a map
- Writes `Flight_<TEAM_ID>.csv` continuously, flushed after every row
- Sends uplink commands (BOOT, SET TIME, CALIBRATE, CX ON/OFF, SIM)
- Switches to a recovery screen automatically when the CanSat lands

---

## 1. Install Python

You need **Python 3.9 or newer**.

**Windows** — download from [python.org/downloads](https://www.python.org/downloads/).
On the first installer screen, tick **"Add python.exe to PATH"**. This matters;
without it the commands below will not be found.

**macOS** — `brew install python3`, or download from python.org.

Check it worked:

```
python --version
```

If Windows says "Python was not found", close and reopen the terminal, or try
`py --version` instead and use `py` wherever this guide says `python`.

---

## 2. Clone the repository

```
git clone https://github.com/The-Atul-Pathak/CanSat.git
cd CanSat
```

---

## 3. Create a virtual environment and install

This keeps the ground station's packages separate from the rest of your system.

**Windows (PowerShell or Command Prompt):**

```
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```

> If PowerShell refuses with *"running scripts is disabled on this system"*, run
> this once and then retry the activate line:
> ```
> Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
> ```

**macOS / Linux:**

```
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

You will know it worked when your prompt starts with `(.venv)`.

**Every time you open a new terminal, activate again** before running the app —
that is the single most common reason for "module not found".

---

## 4. Plug in the XBee

The XBee connects over USB, normally through an XBee Explorer or similar
adapter board.

**Windows may need a driver** for the adapter's USB-to-serial chip. Check
Device Manager → Ports (COM & LPT):

- If you see something like `USB Serial Port (COM3)` — you are ready.
- If you see a yellow warning triangle, or nothing appears, install the driver
  for your adapter's chip:
  - **FTDI** (FT232, FT231X) — [ftdichip.com/drivers/vcp-drivers](https://ftdichip.com/drivers/vcp-drivers/)
  - **CP2102 / CP2104** — [silabs.com CP210x VCP drivers](https://www.silabs.com/developer-tools/usb-to-uart-bridge-vcp-drivers)
  - **CH340 / CH341** — search "CH340 driver" for your Windows version

macOS and Linux normally need no driver. On Linux you may need to add yourself
to the `dialout` group to get permission to open the port:

```
sudo usermod -a -G dialout $USER      # then log out and back in
```

Set the XBee itself to the **same baud rate on both ends**. The default here is
**9600**; change it in the app under ⚙ Settings if your radios use something else.

---

## 5. Check the install

Before touching any hardware, run the self-check:

```
python selftest.py
```

It tests the packet parser, the CSV writer, the port detection, the flight
simulator and the whole window — including one complete simulated mission flown
through the real display — without needing a radio or a CanSat. You should see a
list of `ok` lines ending in:

```
All 98 checks passed. The installation is good.
```

On Windows the radio-link section is skipped automatically (it needs a virtual
serial port that only exists on macOS and Linux), so you will see a slightly
lower count — that is expected.

If this fails, the problem is your installation, not your wiring. Fix it here
before going further.

---

## 6. Run it

```
python ground_station_simple.py
```

No CanSat and no XBee to hand? Fly the whole thing against a simulated one:

```
python ground_station_sim.py
```

See [§11](#11-flight-rehearsal-no-hardware).

---

## 7. Flying a mission

1. **Pick the port** in the dropdown at the bottom left. Only USB devices are
   listed, so the XBee is normally the only entry — something like
   `COM3 — USB Serial Port` or `cu.usbserial-A50285BI — FT231X USB UART`.
   Plug it in at any time and it appears on its own within two seconds.
2. **Press Connect.** The LINK indicator in the header turns green when packets
   start arriving.
3. **Press CX ON.** This commands the CanSat to start transmitting *and* opens
   the flight CSV. Do this on the launch pad, not at liftoff — the file should
   capture the pad wait and calibration too.
4. Fly. Everything updates on its own.
5. When the CanSat reaches **IMPACT**, the recovery screen opens automatically
   with the last known position, the distance and compass bearing to walk, and
   the path of the CSV to hand to the judges.

Keyboard shortcuts: `1` Telemetry · `2` Graphs · `3` Location · `4` Recovery.

### Before a real flight

Under **⚙ Settings**, set **GS Latitude / Longitude** to where the ground
station actually is. The recovery distance and bearing are measured from these,
and they ship set to Chennai.

---

## 8. Files the software writes

All written next to the script, in the project folder:

| File | Contents |
|---|---|
| `Flight_2026-IN-SPACe-CAN-7USAT-056.csv` | The graded flight record. Opens on CX ON, one row per packet, flushed immediately so a laptop crash costs at most one row. |
| `raw_YYYYMMDD_HHMMSS.log` | Every line the radio produced, exactly as it arrived. One per connection. This is the debugging file — see below. |
| `SIM_Flight_2026-IN-SPACe-CAN-7USAT-056.csv` | Written only by `ground_station_sim.py`. Same format, deliberately a different name so a rehearsal can never overwrite the graded file. |

> **`Flight_<TEAM_ID>.csv` is overwritten each time you press CX ON.** Copy it
> somewhere safe after a flight you care about.

---

## 9. When nothing appears on screen

The status line at the bottom right tells you which problem you have. It is the
first thing to read.

| It says | Meaning | Fix |
|---|---|---|
| `No XBee found — check the USB cable` | No USB serial device at all | See "The XBee does not appear" below |
| `no data on the port — check baud rate` | Port opened, zero bytes arrived | Wrong baud rate, CanSat powered off, or the two XBees are not paired |
| `rx 0 · unusable 12 (foreign team id ...)` | Packets arriving, wrong team ID | See below |
| `rx 0 · unusable 12 (only 8 fields, need 19)` | Packets arriving, wrong format | The CanSat is not sending the expected field order |
| `rx 143 · lost 2 · unusable 1` | Working normally | Nothing — a few losses at range are expected |
| `Access is denied` / `Permission denied` on Connect | Port already open | Close Arduino IDE / XCTU / any other serial monitor |

### The XBee does not appear in the dropdown

The list only shows ports that look physically plugged in. It hides the serial
ports a laptop always reports whether or not any hardware exists — the debug
console, and one entry per paired Bluetooth device.

First, check whether the operating system can see it at all:

```
python -m serial.tools.list_ports -v
```

- **The XBee is not listed here either.** The problem is below the software.
  A USB-C cable with nothing on the far end does not create a serial port, so
  an empty list is correct until the adapter itself is connected. Check that
  the cable is a **data** cable and not charge-only, that the adapter board is
  seated, and on Windows that the driver is installed (step 4).
- **It is listed there but not in the dropdown.** Tick the **All** checkbox
  next to the port dropdown. That shows every port with no filtering, so you
  can select it regardless. Please also send that command's output — it means
  the detection missed a case it should handle.

On macOS the XBee appears as `/dev/cu.usbserial-…` or `/dev/cu.usbmodem…`;
on Windows as `COM3` or similar; on Linux as `/dev/ttyUSB0` or `/dev/ttyACM0`.

### Team ID mismatch

The ground station ignores packets from any other team, so it does not draw a
neighbouring CanSat's telemetry by mistake. During testing that usually means
the flight software is still sending an older ID.

Either fix the CanSat firmware, or edit the ID at the top of `gs_logic.py`:

```python
TEAM_ID = "2026-IN-SPACe-CAN-7USAT-056"
```

Note this also names the CSV, so use the real competition ID for real flights.

### Expected packet format

19 comma-separated fields, one line per packet, ending in a newline:

```
TEAM_ID, TIME_STAMPING, PACKET_COUNT, ALTITUDE, PRESSURE, TEMP, VOLTAGE,
GNSS_TIME, GNSS_LATITUDE, GNSS_LONGITUDE, GNSS_ALTITUDE, GNSS_SATS,
ACC_R, ACC_P, ACC_Y, GYRO_R, GYRO_P, GYRO_Y, FLIGHT_SOFTWARE_STATE
```

Three more may follow and are optional — `TVOC, eCO2, GYRO_SPIN_RATE`.

Details that trip people up:

- **PRESSURE is in pascals**, not hPa (the display converts it)
- **TIME_STAMPING** may be seconds (`125.0`) or a clock (`0:02:05`); both work
- **FLIGHT_SOFTWARE_STATE** is the number 0–7, not a name
- Wrapping the line in `<` `>` is accepted and stripped
- A reading far outside physical range (altitude 999999) is treated as a sensor
  fault and the packet is dropped

Example of a valid line:

```
2026-IN-SPACe-CAN-7USAT-056,125.0,7,489.3,95600,23.6,8.35,08:42:10,13.73364,80.18542,491.2,11,0.54,4.22,-0.12,1.10,2.20,3.30,5
```

### Still stuck?

Send the `raw_*.log` file. It contains exactly what came off the radio, which
answers the question immediately — whether the problem is no bytes at all, or
bytes in the wrong shape.

---

## 10. Simulation mode

For demonstrating the software without flying:

1. **SIM EN** — arms simulation mode on the CanSat
2. **SIM ACT** — activates it, and the ground station starts uplinking an
   altitude profile at 1 Hz (ascent to 1000 m, descent through the 600 m
   trigger, landing)
3. The CanSat treats those altitudes as real, changes state and transmits
   telemetry back, which is displayed and recorded exactly like a real flight
4. **SIM DIS** — returns the CanSat to its own sensors

This needs flight software that understands `CMD,SIM,*` and `CMD,SIMP,<alt>`.

---

## 11. Flight rehearsal (no hardware)

> Note the two meanings of "simulation". §10 above is the **competition's**
> simulation mode, where a *real* CanSat flies on pressures you uplink. This
> section is a **hardware** simulation: no CanSat, no XBee, no rocket.

```
python ground_station_sim.py
```

This launches the real ground station — the same window, the same parser, the
same CSV writer — against a virtual CanSat behind a virtual XBee. It connects
and commands CX ON for you, then flies a complete mission in real time:

```
BOOT -> TEST_MODE -> LAUNCH_PAD -> ASCENT -> ROCKET_DEPLOY
     -> DESCENT -> AEROBREAK_RELEASE -> IMPACT -> recovery
```

About three and a half minutes, apex 1000 m, landing roughly half a kilometre
downwind. Everything on screen is driven by generated telemetry: the graphs, the
map track, the attitude indicator, the battery drain, the satellite count, the
packet-loss counter and the recovery screen, which opens itself at impact.

**Every command button really does something to it:**

| Button | Effect on the virtual CanSat |
|---|---|
| **Boot** | Restarts the flight software — clock, packet count and state all reset |
| **Set Time** | Sets the onboard clock; `GNSS_TIME` jumps to match |
| **Calibrate** | Zeroes the barometer (it starts ~2.7 m high) and nulls the gyro bias. Refused in flight |
| **CX ON** | Starts the downlink *and* the launch countdown — nothing flies until the ground station is listening |
| **CX OFF** | Stops the downlink; LINK goes amber after five seconds, as it would |
| **SIM EN → SIM ACT** | The CanSat stops trusting its barometer and flies the altitude profile the ground station uplinks. `SIM ACT` alone is refused, as on real flight software |
| **SIM DIS** | Back onto its own sensors |

The link is deliberately imperfect: about 1 % of packets are lost, a few arrive
corrupted, and the antenna goes through a null while the CanSat tumbles clear of
the rocket — so you see LINK drop to `NO DATA` and recover, and the lost-packet
counter do its job. Signal strength and slant range are shown next to it.

Rehearsals record to **`SIM_Flight_<TEAM_ID>.csv`**, never to the graded
`Flight_<TEAM_ID>.csv`.

### Options

| Flag | Does |
|---|---|
| `--manual` | Do not auto-connect or auto-CX; press the buttons yourself |
| `--speed N` | Run the mission N times faster (`--speed 6` for a quick demo) |
| `--pad-hold S` | Seconds on the pad after CX ON (default 40) |
| `--apex M` | Apex altitude in metres (default 1000) |
| `--wind SPD@BRG` | Wind speed and the bearing it blows *from* (default `4.5@250`) |
| `--loss F` | Fraction of packets the radio loses (default 0.012) |
| `--clean` | A perfect link — no loss, corruption or blackout |
| `--seed N` | Repeat a rehearsal exactly |

---

## 12. Code layout

| File | Contains |
|---|---|
| `ground_station_simple.py` | Entry point for a real flight; just calls `gs_ui.main()` |
| `ground_station_sim.py` | Entry point for a rehearsal; calls the same `main()` with a simulated receiver |
| `gs_logic.py` | Everything that is not drawing — serial link, packet parser, mission data, CSV writer, commands, simulation mode |
| `gs_sim.py` | The virtual CanSat, the virtual XBee and the receiver that ties them together |
| `gs_ui.py` | All the Qt code — widgets, tabs, main window |

`gs_logic.py` and `gs_sim.py` import no Qt, so both can be tested on their own.

The simulator plugs in at exactly one seam: `TelemetryReceiver._open_port()`.
`gs_sim.SimulatedReceiver` overrides it to return a `VirtualXBee` instead of a
`serial.Serial`, and inherits everything else — reader thread, parser, dropped-
packet arithmetic, raw capture, flight CSV, link timeout. That is why a
rehearsal exercises the code that actually flies rather than a copy of it.

Two threads and one queue between them:

```
reader thread    read line -> parse -> write CSV -> put on queue
UI thread        drain queue -> store -> redraw
```

The CSV is written by the reader thread on purpose, so the flight record does
not depend on the display keeping up.

### Things you may want to change

| What | Where |
|---|---|
| Team ID (and therefore the CSV name) | `TEAM_ID` in `gs_logic.py` |
| Default baud rate | `DEFAULT_BAUD` in `gs_logic.py` |
| Secondary deployment altitude | `SECONDARY_TRIGGER_ALT` in `gs_logic.py` |
| Battery full/empty/warning voltages | `VOLTAGE_*` in `gs_logic.py` |
| Packet field order | `_REQUIRED_FIELDS` in `gs_logic.py` |
| Valid sensor ranges | `_LIMITS` in `gs_logic.py` |
| Colours | `THEME` in `gs_ui.py` |
| IMU mounting axis for the attitude indicator | `attitude_from_accel()` in `gs_logic.py` |
| Rehearsal mission profile, sensor models and launch site | `SimConfig` and `VirtualCanSat` in `gs_sim.py` |

---

## Known limitations

- The attitude indicator assumes the **ACC_Y axis runs along the CanSat body**
  (reads about +1 g standing upright on the pad). If your IMU is mounted with a
  different axis vertical, the horizon will be wrong — fix it in
  `attitude_from_accel()`.
- Yaw is shown as a **rate**, not a heading. A gyro alone cannot give an
  absolute heading without a magnetometer.
- The map uses OpenStreetMap tiles and needs internet. Without it, the software
  falls back automatically to an offline grid plot of the same track.
