#!/bin/bash
# Launches everything needed for the 3-drone formation simulation:
# the Micro XRCE-DDS agent, all 3 PX4 instances (+ Gazebo), and all
# 3 formation controller scripts. Everything except Gazebo's own
# GUI window runs in the background, logging to /tmp/formation_logs.
#
# USAGE:
#   chmod +x start_all.sh
#   ./start_all.sh
#
# Then watch everything at once with:
#   tail -f /tmp/formation_logs/*.log
#
# To stop everything, run stop_all.sh (in the same folder).

set -e

WORKDIR="/workspace/PX4-Autopilot"
LOGDIR="/tmp/formation_logs"
mkdir -p "$LOGDIR"

echo "[1/4] Starting MicroXRCEAgent..."
MicroXRCEAgent udp4 -p 8888 > "$LOGDIR/agent.log" 2>&1 &
sleep 2

echo "[2/4] Starting PX4 instance 0 (this one launches Gazebo itself -- window should appear shortly)..."
cd "$WORKDIR"
PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE="0,0" \
    ./build/px4_sitl_default/bin/px4 -i 0 > "$LOGDIR/px4_0.log" 2>&1 &

# Gazebo needs real time to finish starting before other instances
# can attach as standalone clients -- 8s is a safe margin, increase
# if instance 1/2 fail to spawn (check px4_1.log / px4_2.log for
# "waiting for Gazebo" errors if so).
sleep 8

echo "[2/4] Starting PX4 instance 1..."
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE="5,0" \
    ./build/px4_sitl_default/bin/px4 -i 1 > "$LOGDIR/px4_1.log" 2>&1 &
sleep 3

echo "[2/4] Starting PX4 instance 2..."
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE="-5,0" \
    ./build/px4_sitl_default/bin/px4 -i 2 > "$LOGDIR/px4_2.log" 2>&1 &
sleep 5

echo "[3/4] Sourcing ROS 2..."
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash

echo "[4/4] Starting formation controllers..."
python3 "$WORKDIR/drone0_main.py" > "$LOGDIR/drone0.log" 2>&1 &
python3 "$WORKDIR/drone1_main.py" > "$LOGDIR/drone1.log" 2>&1 &
python3 "$WORKDIR/drone2_main.py" > "$LOGDIR/drone2.log" 2>&1 &

echo ""
echo "All processes launched."
echo "Watch everything with:  tail -f $LOGDIR/*.log"
echo "Stop everything with:   ./stop_all.sh"
echo ""

# Keeps this script (and therefore its background jobs' parent
# shell) alive until you Ctrl+C -- without this, closing the
# terminal that ran this script could kill the background jobs too,
# depending on your shell's job control settings.
wait
