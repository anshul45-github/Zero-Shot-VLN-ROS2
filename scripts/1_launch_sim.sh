#!/bin/bash
# ============================================================
# Terminal 1: Launch Gazebo Harmonic + TurtleBot3 in DRL World
# ============================================================
# Run this FIRST before other scripts.
#
# USAGE:
#   ./1_launch_sim.sh           # Stage 4 (default, medium)
#   ./1_launch_sim.sh 1         # Stage 1 (empty arena)
#   ./1_launch_sim.sh 9         # Stage 9 (hard, dense)

set -e
source /opt/ros/humble/setup.bash
source ~/dl_hackathon/install/setup.bash
export TURTLEBOT3_MODEL=waffle
export DRLNAV_BASE_PATH=~/dl_hackathon
export CYCLONEDDS_URI=file://$HOME/dl_hackathon/cyclonedds.xml

STAGE=${1:-4}

echo "🖥️  Launching Gazebo Harmonic — DRL Stage $STAGE (with GUI)"
echo "   Available stages: 1 (easy), 4 (medium), 9 (hard)"
echo ""
ros2 launch turtlebot3_gazebo gz_drl_stage.launch.py stage:=$STAGE
