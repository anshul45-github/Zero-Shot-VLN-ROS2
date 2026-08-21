#!/bin/bash
# ============================================================
# Terminal 3: DRL Environment
# ============================================================
# Processes sensor data, computes rewards, handles state.
# Run AFTER Terminal 1 and 2 are running.

set -e
source /opt/ros/humble/setup.bash
source ~/dl_hackathon/install/setup.bash
export DRLNAV_BASE_PATH=~/dl_hackathon
export TURTLEBOT3_MODEL=waffle
export CYCLONEDDS_URI=file://$HOME/dl_hackathon/cyclonedds.xml

echo "🌍 Starting DRL Environment..."
ros2 run turtlebot3_drl environment
