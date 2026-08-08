#!/usr/bin/env python3
"""
Ground station self-check.

Run this after installing, before connecting real hardware:

    python selftest.py

It exercises the packet parser, the CSV writer, the port filter and the whole
window, so if something is wrong with the installation you find out here rather
than on the launch pad.  No radio and no CanSat needed.

The serial-loopback section needs a virtual serial port, which only exists on
macOS and Linux; it is skipped automatically on Windows.  Everything else runs
everywhere.
"""

import os
import sys
import tempfile

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
        L.list_ports.comports = lambda: [
            FakePort("/dev/cu.debug-console", None, "n/a"),
            FakePort("/dev/cu.SomeHeadphones", None, "n/a"),
        ]
        check(L.TelemetryReceiver.available_ports() == [],
              "Bluetooth and debug ports are never offered")
    finally:
        L.list_ports.comports = real


# ── 4. the radio link, over a virtual serial port ─────────────────
def test_serial_link():
    print("\n[4] radio link")
    if not hasattr(os, "openpty"):
        print("  skip  (needs a virtual serial port; not available on Windows)")
        return

    import pty, time
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


# ── 5. the window itself ──────────────────────────────────────────
def test_ui():
    print("\n[5] user interface")
    import gs_ui as U
    from PyQt6.QtWidgets import QApplication

    app = QApplication.instance() or QApplication(["selftest"])
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
    test_ui()

    print(f"\nAll {_checks} checks passed. The installation is good.")
    print("Plug in the XBee, run:  python ground_station_simple.py")


if __name__ == "__main__":
    main()
