#!/bin/bash
# Stops everything started by start_all.sh: all PX4 instances, the
# MicroXRCEAgent, and all 3 formation controller scripts.
#
# USAGE:
#   chmod +x stop_all.sh
#   ./stop_all.sh

echo "Stopping formation controllers..."
pkill -f "drone0_main.py" 2>/dev/null
pkill -f "drone1_main.py" 2>/dev/null
pkill -f "drone2_main.py" 2>/dev/null

echo "Stopping PX4 instances..."
pkill -f "bin/px4 -i 0" 2>/dev/null
pkill -f "bin/px4 -i 1" 2>/dev/null
pkill -f "bin/px4 -i 2" 2>/dev/null

echo "Stopping MicroXRCEAgent..."
pkill -f "MicroXRCEAgent" 2>/dev/null

echo "Done. (Gazebo's own window may need to be closed manually if it's still open.)"
