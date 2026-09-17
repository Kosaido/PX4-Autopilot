#!/usr/bin/env python3
"""
Launcher for DRONE 0 (the reference/leader drone).

This is the file you put on drone 0's Raspberry Pi. It just calls
into the shared controller logic in formation_control_core.py with
this drone's id hardcoded -- no command-line arguments needed at
boot time, which matters once this runs from a systemd service or
a startup script rather than a terminal you're typing into.

REQUIRES: formation_control_core.py must be present on this Pi, in
the same folder (or importable on the Python path).
"""

from formation_control_core import run

if __name__ == "__main__":
    run(drone_id=0)
