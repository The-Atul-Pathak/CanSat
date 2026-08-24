#!/usr/bin/env python3
"""
Ground station self-check.

Run this after installing, before connecting real hardware:

    python selftest.py

It exercises the packet parser, the CSV writer, the port filter, the flight
simulator and the whole window — including one complete simulated mission flown
through the real display — so if something is wrong with the installation you
find out here rather than on the launch pad.  No radio and no CanSat needed.

The serial-loopback section needs a virtual serial port, which only exists on
macOS and Linux; it is skipped automatically on Windows.  Everything else runs
everywhere.
"""

import os
import sys
import tempfile
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")   # no window on screen

import gs_logic as L

TEAM = L.TEAM_ID
_checks = 0


def check(condition, label):
    global _checks
    _checks += 1
    if not condition:
        print(f"  FAIL  {label}")
        raise SystemExit(1)
    print(f"  ok    {label}")


def headless_qt():
    """
    The QApplication the two window tests share, with the Leaflet map forced
    onto its offline painter fallback.

    A headless run has neither a GPU nor tile access, and building a second
    QWebEngineView in one offscreen process takes Chromium down with it.  The
    offline map is the same code path the ground station uses at a launch site
    with no signal, so this tests the one that matters more anyway.
    """
    import gs_ui as U
    from PyQt6.QtWidgets import QApplication
    U.HAS_WEBENGINE = False
    return QApplication.instance() or QApplication(["selftest"])


def packet(t=1.0, count=1, alt=250.0, state="ASCENT", volt=7.9, team=TEAM,
           extras=True):
    """Build one well-formed telemetry line."""
    fields = [team, f"{t:.1f}", str(count), f"{alt:.1f}", "95600", "23.6",
              f"{volt:.2f}", "08:42:10", "13.73364", "80.18542", f"{alt - 1.2:.1f}",
              "11", "0.54", "4.22", "-0.12", "1.10", "2.20", "3.30",
              str(L.STATE_NUMBER[state])]
    if extras:
        fields += ["120", "410", "350.0"]
    return ",".join(fields)


# ── 1. the packet parser ──────────────────────────────────────────
def test_parser():
    print("\n[1] packet parser")
    p = L.PacketParser()

    pk = p.parse(packet())
    check(pk is not None, "a good packet is accepted")
    check(abs(pk["pressure"] - 956.0) < 0.01, "pressure converted Pa -> hPa")
    check(pk["state"] == "ASCENT", "state number decoded to a name")
    check(pk["gyro_spin"] == 350.0, "optional trailing fields read")

    pk2 = p.parse(packet(t=2.0, count=2, alt=300.0))
    check(abs(pk2["velocity"] - 50.0) < 1e-6, "velocity derived from altitude")

    p.parse(packet(t=3.0, count=5, alt=320.0))
    check(p.dropped == 2, "gap in PACKET_COUNT counted as dropped")

    bare = packet(count=6, extras=False).split(",")
    bare[1] = "0:01:09"
    pk3 = p.parse("<" + ",".join(bare) + ">")
    check(pk3 is not None, "angle-bracket wrapper stripped")
    check(pk3["t"] == 69.0, "hh:mm:ss timestamp read as seconds")
    check(pk3["tvoc"] == 0.0, "missing optional fields default to zero")

    before = p.accepted
    check(p.parse(packet(count=7, team="2024ASI-023")) is None,
          "another team's packet rejected")
    check("foreign team id" in p.last_error, "  ...with a readable reason")
    check(p.parse(packet(count=7, alt=999999.0)) is None,
          "impossible altitude rejected")
    check(p.parse("2024,1,2,3") is None, "truncated line rejected")
    check(p.parse(packet(count=7).replace(",23.6,", ",abc,", 1)) is None,
          "non-numeric field rejected")
    check(p.accepted == before, "no bad packet was counted as accepted")

    row = L.telemetry_row(pk2)
    check(len(row) == len(L.TELEMETRY_HEADER), "CSV row matches the header width")
    check(row[4] == "95600", "pressure written back to the CSV in pascals")


# ── 2. the CSV writer ─────────────────────────────────────────────
def test_csv():
    print("\n[2] flight CSV")
    p = L.PacketParser()
    with tempfile.TemporaryDirectory() as tmp:
        w = L.CSVWriter(tmp)
        for i in range(3):
            w.write(p.parse(packet(t=float(i), count=i + 1, alt=100.0 * i)))
        check(os.path.exists(w.path), "file created on disk")
        rows = open(w.path).read().strip().splitlines()
        check(len(rows) == 4, "header plus one row per packet")
        check(rows[0] == ",".join(L.TELEMETRY_HEADER), "header is the graded order")
        check(os.path.basename(w.path) == L.CSV_FILENAME,
              f"named {L.CSV_FILENAME}")
        w.close()


# ── 3. USB port filtering ─────────────────────────────────────────
def test_ports():
    print("\n[3] serial port detection")

    class FakePort:
        def __init__(self, device, vid, description):
            self.device, self.vid, self.description = device, vid, description

    real = L.list_ports.comports
    try:
        # A Windows machine with one built-in port and one USB adapter
        L.list_ports.comports = lambda: [
            FakePort("COM1", None, "Communications Port"),
            FakePort("COM3", 0x0403, "USB Serial Port (COM3)"),
        ]
        ports = L.TelemetryReceiver.available_ports()
        check(len(ports) == 1 and ports[0][0] == "COM3",
              "USB adapter found, built-in port hidden")

        # A Mac with the debug console and paired Bluetooth audio
        mac_builtins = [FakePort("/dev/cu.debug-console", None, "n/a"),
                        FakePort("/dev/cu.SomeHeadphones", None, "n/a")]
        L.list_ports.comports = lambda: list(mac_builtins)
        check(L.TelemetryReceiver.available_ports() == [],
              "Bluetooth and debug ports are hidden by default")
        check(len(L.TelemetryReceiver.available_ports(include_all=True)) == 2,
              "  ...but reachable via the All checkbox")

        # The case that matters most: a real adapter whose vendor ID came back
        # empty, which pyserial's macOS backend does sometimes.  The device
        # name has to be enough to keep it selectable.
        L.list_ports.comports = lambda: mac_builtins + [
            FakePort("/dev/cu.usbserial-A50285BI", None, "n/a")]
        ports = L.TelemetryReceiver.available_ports()
        check(len(ports) == 1 and "usbserial" in ports[0][0],
              "USB adapter still found when the vendor ID is missing")

        L.list_ports.comports = lambda: mac_builtins + [
            FakePort("/dev/cu.usbmodem14201", None, "n/a")]
        check(len(L.TelemetryReceiver.available_ports()) == 1,
              "same for a native-USB (usbmodem) board")

        L.list_ports.comports = lambda: [FakePort("/dev/ttyUSB0", None, ""),
                                         FakePort("/dev/ttyACM0", None, "")]
        check(len(L.TelemetryReceiver.available_ports()) == 2,
              "Linux ttyUSB and ttyACM recognised")
    finally:
        L.list_ports.comports = real


# ── 4. the radio link, over a virtual serial port ─────────────────
def test_serial_link():
    print("\n[4] radio link")
    if not hasattr(os, "openpty"):
        print("  skip  (needs a virtual serial port; not available on Windows)")
        return

    import pty
    primary, secondary = pty.openpty()
    rx = L.TelemetryReceiver()
    with tempfile.TemporaryDirectory() as tmp:
        check(rx.connect(os.ttyname(secondary), 9600, log_dir=tmp),
              "port opened")
        rx.start_recording(tmp)

        flight = [(1, 1, 0.0, "LAUNCH_PAD"), (2, 2, 900.0, "ROCKET_DEPLOY"),
                  (3, 4, 595.0, "DESCENT"), (4, 5, 0.5, "IMPACT")]
        for t, n, alt, st in flight:
            os.write(primary, (packet(t, n, alt, st) + "\r\n").encode())
        os.write(primary, b"corrupted&line\r\n")
        time.sleep(1.2)

        data = L.MissionData()
        while not rx.packets.empty():
            data.add(rx.packets.get_nowait())

        check(len(data.packets) == 4, "all four good packets received")
        check(rx.parser.rejected == 1, "the corrupted line was rejected")
        check(rx.parser.dropped == 1, "the missing packet was counted")
        check(data.apex == 900.0, "apex tracked")
        check(data.landed, "landing detected")
        check(rx.link_status() == L.LINK_LIVE, "link reports LIVE")

        rx.send(L.build_command("SIM_PRESSURE", 250))
        time.sleep(0.3)
        check(os.read(primary, 100).decode().strip() == "CMD,SIMP,250",
              "uplink command reached the radio")

        raw = open(rx.raw_log_path).read()
        check("corrupted&line" in raw, "raw log captured even the bad line")

        csv_path = rx.csv_path
        rx.stop_recording()
        rx.disconnect()
        check(len(open(csv_path).read().strip().splitlines()) == 5,
              "CSV holds a header plus four rows")
        check(rx.link_status() == L.LINK_OFFLINE, "link reports OFFLINE after close")


# ── 5. the flight simulator ───────────────────────────────────────
def test_simulator():
    """
    The rehearsal path (ground_station_sim.py).  Checked in two halves: the
    vehicle model on its own, flown headless in a few milliseconds, and then
    the whole virtual radio driving the real receiver in real time.
    """
    print("\n[5] flight simulator")
    import gs_sim as S

    cfg = S.SimConfig(seed=11)
    packets, notices = S.fly(cfg)
    check(len(packets) > 120, "a full mission produces a few minutes of telemetry")

    # Every generated line must survive the same parser the radio feeds.
    parser = L.PacketParser()
    sat    = S.VirtualCanSat(S.SimConfig(seed=5))
    sat.command(L.build_command("CX_ON"))
    for _ in range(40):
        sat.step(0.25)
    sat.packet += 1
    pk = parser.parse(sat.telemetry_line())
    check(pk is not None, f"generated telemetry parses ({parser.last_error})")
    check(pk["team_id"] == TEAM, "carries our team ID")
    check(len(L.telemetry_row(pk)) == len(L.TELEMETRY_HEADER),
          "and fills every graded CSV column")

    # The mission script must visit all eight states, in order.
    order  = [L.STATE_NAME[p["state_num"]] for p in packets]
    walked = [s for i, s in enumerate(order) if i == 0 or s != order[i - 1]]
    check(walked == list(L.STATE_NUMBER), f"states walked in order: {walked}")

    apex = max(p["altitude"] for p in packets)
    check(abs(apex - cfg.apex_m) < cfg.apex_m * 0.10,
          f"apex {apex:.0f} m lands on the {cfg.apex_m:.0f} m target")
    check(packets[-1]["altitude"] < 5.0, "and the CanSat ends up on the ground")

    # Sensors must actually move, and move the right way.
    check(packets[0]["voltage"] > packets[-1]["voltage"] > 6.5,
          "battery drains without falling off a cliff")
    check(max(p["sats"] for p in packets) >= 10, "GNSS acquires a full constellation")
    check(packets[0]["sats"] < 4, "  ...having started with a cold receiver")
    check(min(p["temp"] for p in packets) < packets[0]["temp"] - 3.0,
          "temperature falls with altitude")
    check(max(p["acc"][2] for p in packets) > 5.0, "the boost shows up as a g spike")
    check(max(p["spin"] for p in packets) > 250.0, "the mechanical gyro spins up")
    check(max(p["eco2"] for p in packets) > 1000.0, "exhaust spikes eCO2")
    ground = [p for p in packets if p["altitude"] > 400]
    check(min(p["pressure"] for p in ground) < packets[0]["pressure"],
          "pressure falls as altitude rises")

    # The CanSat has to move across the map, not sit on one pixel.
    dist, _ = L.haversine(packets[0]["lat"], packets[0]["lon"],
                          packets[-1]["lat"], packets[-1]["lon"])
    check(dist * 1000 > 100.0, f"wind carries it {dist * 1000:.0f} m downrange")

    # ── uplink commands have to do something ──────────────────────
    sat = S.VirtualCanSat(S.SimConfig(seed=2))
    for _ in range(30):
        sat.step(1.0)
    check(sat.state == "LAUNCH_PAD", "the CanSat reaches the pad on its own")
    check(abs(sat.telemetry()["altitude"]) > 1.5, "uncalibrated baro reads high")
    sat.command(L.build_command("CALIBRATE"))
    check(abs(sat.telemetry()["altitude"]) < 1.0, "CALIBRATE zeroes it")
    check(all(abs(b) < 0.3 for b in sat.gyro_bias), "  ...and nulls the gyro bias")

    sat.command(L.build_command("SET_TIME", "04:05:06"))
    check(sat.gnss_time == "04:05:06", "SET_TIME sets the GNSS clock")

    check(sat.launch_at is None, "nothing launches while the downlink is off")
    sat.command(L.build_command("CX_ON"))
    check(sat.cx and sat.launch_at is not None, "CX ON arms the launch countdown")

    sat.command(L.build_command("SIM_ACTIVATE"))
    check(not sat.sim_active, "SIM ACT refused before SIM EN")
    sat.command(L.build_command("SIM_ENABLE"))
    sat.command(L.build_command("SIM_ACTIVATE"))
    check(sat.sim_active, "SIM EN then SIM ACT enters simulation mode")
    sat.command(L.build_command("SIM_PRESSURE", 500))
    for _ in range(20):
        sat.step(0.2)
    check(sat.alt > 400.0, "an uplinked altitude really flies the CanSat")
    check(sat.state in ("ASCENT", "ROCKET_DEPLOY"),
          "  ...and the state machine follows it")
    sat.command(L.build_command("SIM_DISABLE"))
    check(not sat.sim_active, "SIM DIS returns it to its own barometer")

    sat.command(L.build_command("BOOT"))
    check(sat.state == "BOOT" and sat.t == 0.0 and not sat.cx,
          "BOOT restarts the flight software")

    # ── the virtual radio driving the real receiver ───────────────
    rx = S.SimulatedReceiver(S.SimConfig(seed=4, speed=25.0, pad_hold_s=4.0))
    check(rx.unavailable_reason() == "", "the virtual link needs no pyserial")
    ports = rx.available_ports()
    check(len(ports) == 1 and "XBee" in ports[0][1],
          f"one believable radio in the port list: {ports[0][1]}")

    with tempfile.TemporaryDirectory() as tmp:
        check(rx.connect(ports[0][0], 9600, log_dir=tmp), "virtual port opens")
        check(rx.csv_filename.startswith("SIM_"),
              "a rehearsal cannot overwrite the graded flight CSV")
        rx.start_recording(tmp)
        check("downlink OFF" in rx.link_note(), "the dock explains the silence "
                                                "before CX ON")
        rx.send(L.build_command("CX_ON"))

        data     = L.MissionData()
        deadline = time.time() + 40
        while time.time() < deadline and not data.landed:
            time.sleep(0.1)
            while not rx.packets.empty():
                data.add(rx.packets.get_nowait())

        check(data.landed, "the mission flies to IMPACT over the virtual radio")
        check(rx.link_status() == L.LINK_LIVE, "link reported LIVE throughout")
        check("dBm" in rx.link_note(), f"signal strength reported: {rx.link_note()}")
        check(rx.parser.accepted > 100, f"{rx.parser.accepted} packets received")
        check(rx.parser.dropped > 0, f"{rx.parser.dropped} lost to the radio, "
                                     f"and counted")
        seen = {p["state"] for p in data.packets}
        check(seen == set(L.STATE_NUMBER), "every state reached the ground station")

        csv_path = rx.csv_path
        events   = [m for _, _, m in data.events]
        # Stop the reader first, so the parser count and the file agree — the
        # reader thread would otherwise keep parsing after the writer closed.
        rx.disconnect()
        rx.stop_recording()

        rows = open(csv_path).read().strip().splitlines()
        check(len(rows) == rx.parser.accepted + 1, "every packet reached the CSV")
        check(any("AEROBREAK" in e for e in events),
              "the aerobrake release was logged")
    check(rx.link_status() == L.LINK_OFFLINE, "link closes cleanly")


# ── 6. the window itself ──────────────────────────────────────────
def test_ui():
    print("\n[6] user interface")
    import gs_ui as U

    app = headless_qt()
    U.RECEIVER  = L.TelemetryReceiver()
    U.DATA      = L.MissionData()
    U.SIMULATOR = L.SimulationSender(U.RECEIVER)
    U.FEED      = U.TelemetryFeed(U.RECEIVER, U.DATA)

    win = U.MainWindow()
    win.show()
    win._on_feed(False)
    check(True, "window builds with no telemetry yet")

    flight = [(1, 1, 0.0, "LAUNCH_PAD"), (2, 2, 200.0, "ASCENT"),
              (3, 3, 900.0, "ROCKET_DEPLOY"), (4, 4, 700.0, "DESCENT"),
              (5, 5, 595.0, "AEROBREAK_RELEASE"), (6, 6, 0.5, "IMPACT")]
    for t, n, alt, st in flight:
        U.DATA.add(U.RECEIVER.parser.parse(packet(t, n, alt, st)))

    for tab in ["telemetry", "graphs", "location", "recovery"]:
        win._switch_tab(tab)
        win._on_feed(True)
        app.processEvents()
        check(True, f"{tab} tab renders live telemetry")

    for key in L.COMMANDS:
        win._on_command(key)
    check(True, "every command button is safe to press with no link open")

    win.close()


# ── 7. the window flown by the simulator ──────────────────────────
def test_sim_ui():
    """
    The whole rehearsal, assembled the way ground_station_sim.py assembles it:
    simulated receiver behind the real window, autostarted, then flown until
    the recovery screen comes up by itself.
    """
    print("\n[7] simulated flight through the window")
    import gs_sim as S
    import gs_ui as U

    app = headless_qt()
    U.RECEIVER  = S.SimulatedReceiver(S.SimConfig(seed=9, speed=30.0,
                                                  pad_hold_s=3.0))
    U.DATA      = L.MissionData()
    U.SIMULATOR = L.SimulationSender(U.RECEIVER)
    U.FEED      = U.TelemetryFeed(U.RECEIVER, U.DATA)

    win = U.MainWindow()
    win.show()
    app.processEvents()
    check("XBee" in win._dock._port_box.currentText(),
          "the simulated radio appears in the port dropdown")

    win.autostart(cx=True)
    check(win._link_open, "Connect opened the virtual link")
    check(U.RECEIVER.recording, "CX ON started the rehearsal recording")

    deadline = time.time() + 40
    while time.time() < deadline and not U.DATA.landed:
        app.processEvents()
        time.sleep(0.02)
        for severity, message in U.RECEIVER.drain_notices():
            U.DATA.log(severity, message)

    check(U.DATA.landed, "the window flew a whole mission")
    check(win._stack.currentIndex() == win._tab_index["recovery"],
          "the recovery screen opened itself at IMPACT")
    check(len(U.DATA.events) > 8, f"{len(U.DATA.events)} mission events logged")

    for tab in ["telemetry", "graphs", "location", "recovery"]:
        win._switch_tab(tab)
        win._on_feed(True)
        app.processEvents()
        check(True, f"{tab} tab renders the flown mission")

    win._disconnect_link()
    check(not win._link_open, "Disconnect closes the virtual link")
    win.close()

    # Rehearsal output, not a deliverable — do not leave it in the repo.
    for path in (os.path.join(U.APP_DIR, U.RECEIVER.csv_filename),
                 U.RECEIVER.raw_log_path):
        if path and os.path.exists(path):
            os.remove(path)


def main():
    print(f"CanSat Ground Station self-check")
    print(f"python {sys.version.split()[0]} on {sys.platform}")
    print(f"pyserial {'found' if L.HAS_SERIAL else 'MISSING — pip install pyserial'}")

    if not L.HAS_SERIAL:
        raise SystemExit("\npyserial is required. Run: pip install -r requirements.txt")

    test_parser()
    test_csv()
    test_ports()
    test_serial_link()
    test_simulator()
    test_ui()
    test_sim_ui()

    print(f"\nAll {_checks} checks passed. The installation is good.")
    print("Plug in the XBee, run:  python ground_station_simple.py")
    print("No hardware to hand?     python ground_station_sim.py")


if __name__ == "__main__":
    main()
