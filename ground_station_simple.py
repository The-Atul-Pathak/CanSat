#!/usr/bin/env python3
"""
CanSat Ground Station — Team Kalpana

Run:
    python3 ground_station_simple.py

Dependencies:
    pip install PyQt6 pyqtgraph numpy

How the code is organised:
    gs_logic.py  —  pure Python, no Qt
                    simulation engine, CSV loader, haversine, utilities
    gs_ui.py     —  all Qt code
                    widgets, tabs, main window, playback engine
    ground_station_simple.py  (this file)
                    just calls gs_ui.main()
"""

from gs_ui import main

if __name__ == "__main__":
    main()
