#!/usr/bin/env python3
"""
CanSat Ground Station — flight rehearsal.

Run:
    python3 ground_station_sim.py

Launches the real ground station against a virtual CanSat and a virtual XBee.
The port dropdown, Connect, the command buttons, the graphs, the map, the
flight CSV and the recovery screen all behave exactly as they do on launch day;
the only thing that is not real is the hardware at the other end of the radio.

Use ground_station_simple.py for an actual flight.  This file never touches a
serial port and never writes the graded Flight_<TEAM_ID>.csv — its recording
goes to SIM_Flight_<TEAM_ID>.csv so a rehearsal cannot destroy a real one.

Mission
-------
The virtual CanSat powers up on its own and walks the full state machine:

    BOOT → TEST_MODE → LAUNCH_PAD → ASCENT → ROCKET_DEPLOY
         → DESCENT → AEROBREAK_RELEASE → IMPACT → recovery

It sits on the pad until CX ON tells it the ground station is listening, then
counts down and flies.  Every command button does something real to it — see
the mission event log for what.

Options
-------
    --manual              do not auto-connect or auto-CX; press the buttons
    --speed N             run the mission N times faster (default 1)
    --pad-hold S          seconds on the pad after CX ON (default 40)
    --apex M              apex altitude in metres (default 1000)
    --wind SPD@BRG        wind, m/s and the bearing it blows from (default 4.5@250)
    --loss F              fraction of packets the radio loses (default 0.012)
    --clean               a perfect link: no loss, no corruption, no blackout
    --seed N              repeat a rehearsal exactly
"""

import argparse
import sys

import gs_sim
import gs_ui


def _wind(text: str):
    """Parse a --wind value of the form SPEED@BEARING, e.g. 4.5@250."""
    try:
        speed, bearing = text.split("@")
        return float(speed), float(bearing)
    except ValueError:
        raise argparse.ArgumentTypeError("wind must look like 4.5@250 "
                                         "(m/s @ bearing it blows from)")


def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description="Fly the CanSat ground station against a simulated CanSat.")
    ap.add_argument("--manual", action="store_true",
                    help="do not auto-connect or auto-CX")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="mission time scale (2 = twice as fast)")
    ap.add_argument("--pad-hold", type=float, default=40.0,
                    help="seconds on the pad after CX ON")
    ap.add_argument("--apex", type=float, default=1000.0,
                    help="apex altitude above the pad, metres")
    ap.add_argument("--wind", type=_wind, default=(4.5, 250.0),
                    help="wind as SPEED@BEARING, e.g. 4.5@250")
    ap.add_argument("--loss", type=float, default=0.012,
                    help="fraction of packets the link loses")
    ap.add_argument("--clean", action="store_true",
                    help="perfect link — no loss, corruption or blackout")
    ap.add_argument("--seed", type=int, default=None,
                    help="RNG seed, for a repeatable rehearsal")
    return ap.parse_args(argv)


def build_config(args) -> gs_sim.SimConfig:
    cfg = gs_sim.SimConfig(
        speed=max(0.1, args.speed),
        pad_hold_s=max(0.0, args.pad_hold),
        apex_m=max(50.0, args.apex),
        wind_mps=args.wind[0],
        wind_from_deg=args.wind[1],
        loss=max(0.0, min(0.5, args.loss)),
        seed=args.seed,
    )
    if args.clean:
        cfg.loss = cfg.corrupt = cfg.blackout_s = 0.0
    return cfg


def briefing(cfg: gs_sim.SimConfig, auto: bool):
    """Print the flight card, so the console says what is about to happen."""
    device, label = gs_sim._fake_port()
    pad_to_launch = cfg.boot_s + cfg.test_s + cfg.pad_hold_s
    print("─" * 68)
    print("CanSat Ground Station — FLIGHT REHEARSAL (no hardware)")
    print("─" * 68)
    print(f"  radio       {label}")
    print(f"  recording   {gs_sim.SimulatedReceiver.csv_filename}")
    print(f"  apex        {cfg.apex_m:.0f} m      descent "
          f"{cfg.descent_mps:.0f} → {cfg.aerobrake_mps:.1f} m/s "
          f"at {gs_sim.AEROBRAKE_ALT:.0f} m")
    print(f"  wind        {cfg.wind_mps:.1f} m/s from {cfg.wind_from_deg:.0f}°")
    print(f"  link        {cfg.loss * 100:.1f}% loss · "
          f"{cfg.corrupt * 100:.1f}% corrupt · "
          f"{cfg.blackout_s:.1f} s null at separation")
    print(f"  timeline    liftoff ~T+{pad_to_launch:.0f} s after CX ON, "
          f"{cfg.speed:g}× real time")
    if auto:
        print("  start       connecting and commanding CX ON automatically")
    else:
        print("  start       press Connect, then CX ON")
    print("─" * 68)


def main(argv=None):
    args = parse_args(argv)
    cfg  = build_config(args)
    briefing(cfg, auto=not args.manual)

    receiver = gs_sim.SimulatedReceiver(cfg)

    def on_ready(win):
        """Attach the rehearsal to the live window, then let it fly."""
        win.setWindowTitle(win.windowTitle() + "   ·   SIMULATION")

        # The virtual CanSat talks back — bring its log lines into the mission
        # event log the same way a real crew would call them out.
        pump = gs_ui.QTimer(win)
        pump.setInterval(200)
        pump.timeout.connect(
            lambda: [gs_ui.DATA.log(sev, msg)
                     for sev, msg in receiver.drain_notices()])
        pump.start()
        win._sim_notice_pump = pump      # keep it alive with the window

        if not args.manual:
            # Once the event loop is running, so the window is already up when
            # the link opens and the operator sees it happen.
            gs_ui.QTimer.singleShot(
                400, lambda: win.autostart(gs_sim._fake_port()[0], cx=True))

    # Qt parses argv itself and does not know our flags.
    sys.argv = sys.argv[:1]
    gs_ui.main(receiver_factory=lambda: receiver, on_ready=on_ready)


if __name__ == "__main__":
    main()
