#!/bin/bash
# ============================================================
# Terminal 4: DRL Training Agent
# ============================================================
# Starts or resumes training. Run AFTER all other terminals.
#
# USAGE:
#   Fresh training:     ./4_train.sh td3
#   Resume training:    ./4_train.sh td3 "td3_0" 500
#   Test a model:       ./4_train.sh td3 "td3_0" 500 --test
#
# ALGORITHMS: td3, ddpg, dqn
#
# CHECKPOINTS:
#   Models auto-save every 50 episodes to:
#   ~/dl_hackathon/src/turtlebot3_drl/model/<HOSTNAME>/<MODEL_NAME>/
#
#   To resume, specify the model name and episode number.

set -e
source /opt/ros/humble/setup.bash
source ~/dl_hackathon/install/setup.bash
export DRLNAV_BASE_PATH=~/dl_hackathon
export TURTLEBOT3_MODEL=waffle
export CYCLONEDDS_URI=file://$HOME/dl_hackathon/cyclonedds.xml

# Parse arguments
ALGORITHM=${1:-td3}
MODEL_NAME=${2:-}
EPISODE=${3:-}
MODE="train"

for arg in "$@"; do
  if [ "$arg" == "--test" ]; then
    MODE="test"
  fi
done

echo "🤖 DRL Agent Configuration:"
echo "   Algorithm:  $ALGORITHM"
echo "   Mode:       ${MODE}ing"

if [ -n "$MODEL_NAME" ] && [ -n "$EPISODE" ]; then
  echo "   Loading:    $MODEL_NAME @ episode $EPISODE"
  echo ""
  ros2 run turtlebot3_drl ${MODE}_agent "$ALGORITHM" "$MODEL_NAME" "$EPISODE"
else
  echo "   Starting:   Fresh training"
  echo ""
  ros2 run turtlebot3_drl ${MODE}_agent "$ALGORITHM"
fi
