# TurtleBot3 DRL Navigation Training

Autonomous navigation via Deep Reinforcement Learning in Gazebo Harmonic.
Robot learns to reach goals and avoid obstacles using only LiDAR + odometry — no map, no path planning.

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│  Terminal 1: Gazebo Sim          (world + robot + sensors)  │
│  Terminal 2: gazebo_goals        (spawn/track goal markers) │
│  Terminal 3: environment         (state/reward/step logic)  │
│  Terminal 4: train_agent <algo>  (neural network + training)│
└─────────────────────────────────────────────────────────────┘
```

All 4 nodes communicate via ROS2 services (`step_comm`, `goal_comm`, `task_succeed`, `task_fail`).

---

## State Space

Each step the agent observes a vector of size **`NUM_SCAN_SAMPLES + 4`**:

| Index | Value |
|---|---|
| `0 … N-1` | LiDAR scan ranges (N samples, capped at 3.5 m) |
| `N` | Distance to goal (m) |
| `N+1` | Angle to goal (rad) |
| `N+2` | Last linear velocity |
| `N+3` | Last angular velocity |

`NUM_SCAN_SAMPLES` is auto-read from the waffle SDF (default **120**).
Total state size = **124**.

---

## Action Space

| Algorithm | Actions | Type |
|---|---|---|
| TD3, DDPG | `[linear_vel, angular_vel]` | Continuous ∈ [-1, 1], scaled to ±0.22 m/s / ±2.0 rad/s |
| DQN | 5 discrete velocity pairs | Discrete index 0–4 |

---

## Reward Function (A)

```python
r = r_yaw          # [-π, 0]     penalize heading error
  + r_distance     # [-1, 1]     progress toward goal
  + r_obstacle     # -20 or 0    obstacle < 0.22 m
  + r_vlinear      # [-4.84, 0]  penalize slow speed
  + r_vangular     # [-4, 0]     penalize spinning
  - 1              # step cost

# Terminal bonuses
SUCCESS           → +2500
COLLISION         → -2000
```

---

## Algorithms

### TD3 — Twin Delayed Deep Deterministic Policy Gradient (recommended)
- **Actor**: 3-layer MLP (state→512→512→action), tanh output
- **Critic**: 2× Q-networks with separate state/action encoders
- Delayed actor update every 2 critic steps (`POLICY_UPDATE_FREQUENCY=2`)
- Target policy smoothing: noise σ=0.2, clipped ±0.5
- OUNoise exploration (σ=0.1)
- Best for: continuous control, stable training, highest final performance

### DDPG — Deep Deterministic Policy Gradient
- Same architecture as TD3 but single critic
- Faster per-step but higher variance
- Good for: quick experiments, less stable than TD3

### DQN — Deep Q-Network
- Discrete action space (5 velocity pairs)
- ε-greedy exploration (decay 0.9995, min 0.05)
- Target network hard update every 1000 steps
- Good for: simple stages, fast iteration, interpretable actions

---

## Hyperparameters (`settings.py`)

| Param | Value | Meaning |
|---|---|---|
| `HIDDEN_SIZE` | 512 | Neurons per hidden layer |
| `BATCH_SIZE` | 128 | Samples per training step |
| `BUFFER_SIZE` | 1,000,000 | Replay buffer capacity |
| `DISCOUNT_FACTOR` | 0.99 | Future reward discount γ |
| `LEARNING_RATE` | 0.003 | Adam optimizer LR |
| `TAU` | 0.003 | Soft target update rate |
| `OBSERVE_STEPS` | 25,000 | Random exploration before training starts |
| `EPISODE_TIMEOUT` | 50 s | Max episode length |
| `MODEL_STORE_INTERVAL` | 50 | Save weights every N episodes |

---

## Training Stages

10 stages of increasing difficulty. Stage = obstacle count + layout complexity.

| Stage | Obstacles | Description |
|---|---|---|
| 1 | 0 | Empty arena — pure goal reaching |
| 2 | 2 | 2 static obstacles |
| 3 | 4 | 4 static obstacles |
| **4** | **4** | **4 obstacles + tighter layout (default)** |
| 5 | 4 | Narrow corridors |
| 6 | 5 | Mixed static/dynamic obstacles |
| 7 | 5 | Dense static obstacles |
| 8 | 6 | Dense + moving obstacles |
| 9 | 6 | Hard — dense + narrow passages |
| 10 | 6 | Expert — maximum density |

Arena: 4.2 × 4.2 m. Goal radius: 0.20 m. Collision threshold: 0.13 m.

**Curriculum strategy**: train stage 1→4→7→10, loading previous model at each step.

---

## How to Run

### Prerequisites
```bash
cd ~/dl_hackathon
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
export DRLNAV_BASE_PATH=~/dl_hackathon
export CYCLONEDDS_URI=file://$HOME/dl_hackathon/cyclonedds.xml
```

Or use the provided scripts (they export everything automatically).

---

### Fresh Training (4 terminals)

**Terminal 1 — Gazebo sim:**
```bash
./scripts/1_launch_sim.sh 4        # stage 4 (default)
./scripts/1_launch_sim.sh 1        # stage 1 (easy start)
```

**Terminal 2 — Goal manager:**
```bash
./scripts/2_goal_manager.sh
```

**Terminal 3 — Environment node:**
```bash
./scripts/3_environment.sh
```

**Terminal 4 — Train:**
```bash
./scripts/4_train.sh td3           # TD3 (recommended)
./scripts/4_train.sh ddpg          # DDPG
./scripts/4_train.sh dqn           # DQN
```

---

### Resume Training
```bash
# ./4_train.sh <algo> <model_name> <episode>
./scripts/4_train.sh td3 "td3_5_stage_4" 500
```
Model name = folder name under `src/turtlebot3_drl/model/<hostname>/`.

---

### Test a Model (no training)
```bash
./scripts/4_train.sh td3 "td3_5_stage_4" 500 --test
```

---

### All-in-one convenience script
```bash
./scripts/start_training.sh
```

---

## Training Flow

```
Episode start
  │
  ├─ OBSERVE PHASE (steps < 25,000)
  │    └─ random actions → fill replay buffer
  │
  └─ TRAINING PHASE (steps ≥ 25,000)
       ├─ get_action(state) → actor network + OUNoise
       ├─ step → next_state, reward, done
       ├─ add to replay buffer
       ├─ sample batch → train critic (every step)
       ├─ train actor + soft-update targets (every 2 steps for TD3)
       └─ episode done → log, save model every 50 episodes
```

Episode ends on: **SUCCESS** (goal reached) | **COLLISION** (wall/obstacle) | **TUMBLE** (robot falls) | **TIMEOUT** (50 s)

---

## Model Storage

```
src/turtlebot3_drl/model/
└── <hostname>/
    └── <algo>_<N>_stage_<S>/
        ├── td3_actor_episode_50.pt
        ├── td3_critic_episode_50.pt
        ├── td3_target_actor_episode_50.pt
        ├── td3_target_critic_episode_50.pt
        ├── stage4_latest_buffer.pkl    ← replay buffer (resume training)
        ├── graphdata.pkl               ← reward/loss history
        └── log.txt                     ← per-episode CSV log
```

---

## Monitoring Training

Episode output format:
```
Epi: 120  R: -847    outcome: SUCCESS      steps: 312   steps_total: 84521  time: 18.43
Epi: 121  R: -2847   outcome: COLL_WALL    steps: 45    steps_total: 84566  time: 6.12
```

Graph (reward + loss curves) auto-saved every 10 episodes to model directory.

**Success rate target**: ~70%+ sustained over 50 episodes before moving to harder stage.

---

## Curriculum Transfer (stage-to-stage)

```bash
# Train stage 1 → get model at episode 300
./scripts/4_train.sh td3

# Kill sim, relaunch on stage 4
./scripts/1_launch_sim.sh 4

# Resume model on new stage
./scripts/4_train.sh td3 "td3_1_stage_1" 300
```

The model dir name encodes the original stage — the model trains further on the new stage and saves to the same directory.

---

## Key Files

| File | Purpose |
|---|---|
| `src/turtlebot3_drl/turtlebot3_drl/common/settings.py` | All hyperparameters |
| `src/turtlebot3_drl/turtlebot3_drl/drl_environment/reward.py` | Reward function |
| `src/turtlebot3_drl/turtlebot3_drl/drl_agent/td3.py` | TD3 network + training |
| `src/turtlebot3_drl/turtlebot3_drl/drl_agent/drl_agent.py` | Main training loop |
| `src/turtlebot3_drl/turtlebot3_drl/drl_environment/drl_environment.py` | State/reward/done logic |
| `src/turtlebot3_drl/turtlebot3_drl/drl_gazebo/drl_gazebo.py` | Goal spawning |
| `cyclonedds.xml` | CycloneDDS config (max 50 participants — required) |
