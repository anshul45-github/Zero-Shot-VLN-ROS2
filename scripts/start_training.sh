#!/bin/bash
# ============================================================
# 🚀 MASTER LAUNCHER: Sleep-Proof DRL Training via tmux
# ============================================================
# Launches ALL 4 DRL processes in a single tmux session.
# Training survives terminal close and SSH disconnect.
#
# USAGE:
#   Fresh training (stage 4):     ./start_training.sh td3
#   Fresh training (stage 1):     ./start_training.sh td3 --stage 1
#   Resume training:              ./start_training.sh td3 --resume "td3_0" 500
#   Resume on different stage:    ./start_training.sh td3 --stage 9 --resume "td3_0" 500
#
# MONITOR:   tmux attach -t drl_training
# STOP:      tmux kill-session -t drl_training
#
# PREVENT SLEEP:
#   sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
# RE-ENABLE SLEEP:
#   sudo systemctl unmask sleep.target suspend.target hibernate.target hybrid-sleep.target
# ============================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION="drl_training"
ALGORITHM=${1:-td3}
STAGE=4
MODEL_NAME=""
EPISODE=""

# Parse arguments
shift 2>/dev/null || true
while [[ $# -gt 0 ]]; do
  case $1 in
    --stage)
      STAGE="$2"
      shift 2
      ;;
    --resume)
      MODEL_NAME="$2"
      EPISODE="$3"
      shift 3
      ;;
    *)
      shift
      ;;
  esac
done

# Kill existing session
tmux kill-session -t $SESSION 2>/dev/null || true

echo "============================================================"
echo "🚀 DRL Training Session"
echo "============================================================"
echo "   Algorithm:    $ALGORITHM"
echo "   Stage:        $STAGE"
if [ -n "$MODEL_NAME" ]; then
  echo "   Resuming:     $MODEL_NAME @ episode $EPISODE"
else
  echo "   Mode:         Fresh training"
fi
echo ""
echo "   Monitor:      tmux attach -t $SESSION"
echo "   Stop:         tmux kill-session -t $SESSION"
echo "============================================================"
echo ""

# Create tmux session
tmux new-session -d -s $SESSION -n training

# Pane 0: Simulation
tmux send-keys -t $SESSION "bash $SCRIPT_DIR/1_launch_sim.sh $STAGE" Enter

echo "⏳ Waiting 20s for simulation to load..."
sleep 20

# Split into 2x2 grid
tmux split-window -h -t $SESSION
tmux split-window -v -t $SESSION:0.0
tmux split-window -v -t $SESSION:0.1

# Pane 2: Goal Manager
tmux send-keys -t $SESSION:0.2 "bash $SCRIPT_DIR/2_goal_manager.sh" Enter
sleep 3

# Pane 1: Environment
tmux send-keys -t $SESSION:0.1 "bash $SCRIPT_DIR/3_environment.sh" Enter
sleep 3

# Pane 3: Training Agent
if [ -n "$MODEL_NAME" ] && [ -n "$EPISODE" ]; then
  tmux send-keys -t $SESSION:0.3 "bash $SCRIPT_DIR/4_train.sh $ALGORITHM \"$MODEL_NAME\" $EPISODE" Enter
else
  tmux send-keys -t $SESSION:0.3 "bash $SCRIPT_DIR/4_train.sh $ALGORITHM" Enter
fi

echo ""
echo "✅ All 4 processes launched in tmux session '$SESSION'"
echo ""
echo "📺 To monitor:  tmux attach -t $SESSION"
echo "🛑 To stop:     tmux kill-session -t $SESSION"
echo "📂 Checkpoints: ~/dl_hackathon/src/turtlebot3_drl/model/$(hostname)/"
echo ""
