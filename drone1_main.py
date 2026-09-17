#!/usr/bin/env python3
"""
Launcher for DRONE 1 (follower).

This is the file you put on drone 1's Raspberry Pi. See
drone0_main.py for the full explanation -- this file only differs
by the hardcoded drone_id.

REQUIRES: formation_control_core.py must be present on this Pi, in
the same folder (or importable on the Python path).
"""

from formation_control_core import run

if __name__ == "__main__":
    run(drone_id=1)
