#!/usr/bin/env python3
"""
CanSat Ground Station — Team Kalpana

Run:
    python3 ground_station_simple.py

Then pick the serial port the XBee is plugged into, press Connect, and press
CX ON to command the downlink and start recording Flight_<TEAM_ID>.csv.

Dependencies:
    pip install -r requirements.txt

No CanSat or XBee to hand?  ground_station_sim.py runs this same window against
a simulated one — see README §11.

How the code is organised:
    gs_logic.py  —  pure Python, no Qt
                    serial link, packet parser, mission data, CSV writer,
                    commands, simulation profile, haversine, utilities
    gs_sim.py    —  pure Python, no Qt
                    virtual CanSat, virtual XBee, simulated receiver
    gs_ui.py     —  all Qt code
                    widgets, tabs, main window, telemetry feed
    ground_station_simple.py  (this file)
                    just calls gs_ui.main()
    ground_station_sim.py
                    calls the same main() with a simulated receiver
"""

#Fix this
#> **`Flight_<TEAM_ID>.csv` is overwritten each time you press CX ON.** Copy it
#> somewhere safe after a flight you care about.


from gs_ui import main

if __name__ == "__main__":
    main()
