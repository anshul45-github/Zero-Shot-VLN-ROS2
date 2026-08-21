# Vision-Language Navigation Robot

![VLN Robot Control Demo](demo.gif)

TurtleBot3 Waffle navigates to objects described in natural language inside a Gazebo environment it has never seen before. No pre-built map. No hand-coded waypoints.

```
"Go to the chair, move past the trash can, stop at the bed."
```

You tell the robot where to go in plain English. Gemini parses the instruction into an ordered list of goals. The robot finds each object using its camera, builds a map as it moves, and navigates to each one — exploring autonomously if a target hasn't been spotted yet.

---

## Demo

Type or say **"go to the chair"**. The robot:

1. Sends the instruction to Gemini 2.5 Flash
2. Gemini returns a structured goal list: `["chair"]`
3. The robot scans its camera feed — YOLO detects objects, CLIP matches them to the goal label
4. If the chair is visible, Nav2 navigates directly to it
5. If not yet seen, explore_lite runs frontier exploration until the chair appears in frame
6. Robot arrives at the chair, moves to the next goal if any

That's the whole loop. No map pre-built. No waypoints hand-coded. One sentence in, robot moves.

---

## Why This Problem

Most VLN systems assume a pre-built map or require environment-specific training. Real deployments face environments the robot has never seen. This system works zero-shot:

- No pre-built semantic map — perception runs live on the camera feed
- No hand-coded waypoints — goals come entirely from natural language
- No environment-specific training — YOLO, CLIP, and Gemini are used zero-shot
- Fully autonomous exploration when a target is not yet visible

---

## Pipeline

```
Voice / Text
     │
     ▼
Gemini 2.5 Flash  ──►  Goal list  ["chair", "bed"]
                                        │
                    ┌───────────────────▼──────────────────────┐
                    │           For each goal:                  │
                    │                                           │
                    │  Camera RGB-D ──► YOLO detect objects     │
                    │                       │                   │
                    │               CLIP match to goal label    │
                    │                       │                   │
                    │          Object found? ──No──► explore_lite frontier BFS
                    │                │ Yes                      │
                    │                ▼                          │
                    │         Nav2 navigate to object           │
                    │         (Cartographer SLAM + A* + DWB)   │
                    └──────────────────────────────────────────┘
```

### Perception

Each camera frame runs through:

- **YOLOv8x-World v2** — open-vocabulary object detection (conf=0.2). No fixed class list; it detects whatever label you give it.
- **MobileSAM + FastSAM-x** — instance segmentation to get precise object masks from YOLO bounding boxes
- **CLIP ViT-L-14** — 768-dim visual-semantic embeddings. Computes cosine similarity between the camera crop and the goal label to confirm identity

When CLIP similarity exceeds threshold against the current goal, the object's 3D position is computed from the depth image (unprojected using camera intrinsics) and passed to Nav2 as a navigation target.

### Language Parsing

Gemini 2.5 Flash receives the raw instruction and returns a structured Pydantic schema:

```python
class GoalStack(BaseModel):
    goals: list[str]           # ordered list of object labels
    stop_after_each: bool      # pause at each goal or chain them
```

Input: `"go to the chair, then stop at the bed"`
Output: `GoalStack(goals=["chair", "bed"], stop_after_each=True)`

Gemini handles ambiguity, paraphrasing, and multi-goal sequences. The robot only ever sees the clean label list.

### Navigation

- **Cartographer** — real-time Graph SLAM from LiDAR, builds a 2D occupancy map as the robot moves
- **Nav2** — NavFn A* global planner + DWB local planner (7 critics), 1.0 rad/s angular velocity cap to keep SLAM stable
- **explore_lite** — Wavefront BFS over the Nav2 costmap for frontier exploration when the target is not yet visible

---

## RL Navigation (Optional)

A TD3 policy is also available as a drop-in replacement for Nav2's local planner. It reads LiDAR directly and outputs continuous velocity commands at 10 Hz — no replan loop, so it reacts to moving obstacles immediately.

### Reward Function

```
r(t) = r_distance + r_collision + r_goal
```

- **r_distance** — positive reward for closing distance to goal each step
- **r_collision** — −200 when any LiDAR reading falls below safe threshold
- **r_goal** — +200 sparse reward on arrival within success radius

### Twin Critics (TD3)

Two independent critics, Q₁ and Q₂. Target uses `min(Q₁', Q₂')` to prevent Q-value overestimation. Actor updated every 2 critic steps (delayed policy update). Target policy smoothing adds clipped Gaussian noise to target actions.

### Why RL Is Not Used in the Main Pipeline

The TD3 policy uses 2.0 rad/s max angular velocity — twice what Nav2 allows. At that speed, the LiDAR completes a full sweep while the heading is still changing. Cartographer receives a scan acquired across multiple orientations and treats it as a single-pose snapshot — scan-to-submap matching fails, submaps misalign, and the global map drifts. The entire perception pipeline breaks because 3D object positions depend on accurate odometry.

The main launch uses Nav2 with a 1.0 rad/s cap. **Fix:** retrain with angular velocity ≤1.0 rad/s, or add LiDAR motion distortion correction before Cartographer. Not completed due to time constraints.

### Training

Two stages in Gazebo:

- **Stage 1 — static obstacles**: robot learns basic goal-reaching and collision avoidance
- **Stage 2 — dynamic obstacles**: moving obstacles added, robot learns to react to objects crossing its path

Architecture: 44→512→512→2 MLP, Tanh output. Loaded by `rl_nav_node.py` via `model_path` ROS parameter.

---

## Stack

| Layer | Tech |
|---|---|
| Simulation | Gazebo Harmonic · small_house.world |
| Robot | TurtleBot3 Waffle |
| Middleware | ROS 2 Humble |
| SLAM | Cartographer (Graph SLAM) |
| Navigation | Nav2 · NavFn A* · DWB (7 critics) |
| RL nav | TD3 · 44→512→512→2 MLP · 2 training stages |
| Exploration | explore_lite (Wavefront BFS) |
| Detection | YOLOv8x-World v2 (conf=0.2) |
| Segmentation | MobileSAM + FastSAM-x (conf=0.35) |
| Features | CLIP ViT-L-14 · 768-dim |
| Language | Gemini 2.5 Flash · Pydantic structured output |
| STT | Whisper (local HuggingFace) |
| Web GUI | Next.js · rosbridge WebSocket · WebSocket STT server |

---

## Installation

### Prerequisites

- ROS 2 Humble
- Gazebo Harmonic
- conda (for perception Python environment)
- Node.js ≥18 (for web GUI only)
- CUDA-capable GPU (for YOLO, SAM, CLIP inference)

```bash
sudo apt install ros-humble-cartographer-ros ros-humble-nav2-bringup \
                 ros-humble-rosbridge-server ros-humble-rviz2
```

### 1. Clone and Build ROS 2 Workspace

```bash
cd ~/dl_hackathon
colcon build --symlink-install
source install/setup.bash
```

### 2. Set Up Conda Environment

```bash
cd ~/dl_hackathon/src/hackathon_updated/hackathon/scratch/DualMap
conda env create -f environment.yml
conda activate dualmap
```

Installs: `torch`, `torchvision`, `open_clip_torch`, `ultralytics==8.3.103`, `open3d`, `hydra-core`, `supervision==0.25.1`, `rerun-sdk==0.22.1`, `faiss-cpu`.

### 3. Download Model Weights

Place inside `src/hackathon_updated/hackathon/scratch/DualMap/model/`:

| File | Source |
|---|---|
| `yolov8x-worldv2.pt` | Ultralytics (auto-downloads on first run) |
| `mobile_sam.pt` | MobileSAM (Zhang et al., 2023) |
| `FastSAM-x.pt` | FastSAM (Zhao et al., 2023) |
| CLIP ViT-L-14 | Auto-downloaded by open_clip on first run |

### 4. Set Gemini API Key

```bash
export GEMINI_API_KEY="your_key_here"
```

Get a key at [aistudio.google.com](https://aistudio.google.com).

### 5. Set TurtleBot3 Model

```bash
export TURTLEBOT3_MODEL=waffle
```

### 6. Install Web GUI Dependencies (GUI Mode Only)

```bash
cd ~/dl_hackathon/VLN-Robot-Control/my-app
npm install

cd ~/dl_hackathon/VLN-Robot-Control
pip install -r requirements.txt
sudo apt install portaudio19-dev
python -m spacy download en_core_web_sm
```

---

## Running

### Option A — Topic Mode (No GUI)

```bash
source ~/dl_hackathon/install/setup.bash
export TURTLEBOT3_MODEL=waffle
export GEMINI_API_KEY="your_key_here"

# Start full pipeline
ros2 launch turtlebot3_gazebo gz_vln_exploration.launch.py

# Send a command
ros2 topic pub --once /vln/instruction std_msgs/String "data: 'go to the chair'"
```

### Option B — Web GUI (Voice + Text in Browser)

```bash
# Terminal 1 — full pipeline with rosbridge
source ~/dl_hackathon/install/setup.bash
export TURTLEBOT3_MODEL=waffle
export GEMINI_API_KEY="your_key_here"
ros2 launch turtlebot3_gazebo gz_vln_gui.launch.py

# Terminal 2 — Next.js frontend
cd ~/dl_hackathon/VLN-Robot-Control/my-app
npm run dev

# Open http://localhost:3000
```

**GUI flow:**
1. Browser connects to rosbridge at `ws://localhost:9090`
2. Click mic → Float32 PCM audio streams to STT server at `ws://localhost:8765`
3. Whisper transcribes → Gemini parses → goal list sent back to browser
4. Browser publishes goal to `/vln/goal_stack` via rosbridge
5. Robot navigates

Type directly in the text box to skip Whisper.

### Launch Startup Sequence

| Time | What Starts |
|---|---|
| 0 s | Gazebo, robot spawn, Cartographer, Nav2 (5 lifecycle nodes), map_odom_bridge, rviz2 |
| 0 s (GUI only) | rosbridge (port 9090), STT server (port 8765) |
| 25 s | explore_lite, perception subprocess, vln_parser, navigation controller |

25-second delay: Nav2 costmaps need LiDAR data to initialize before explore_lite and perception attach.

---

## Repository Layout

```
src/
├── vln_pipeline/                          # Main ROS 2 package
│   ├── vln_pipeline/
│   │   ├── arbitrator_node.py             # Navigation controller: search → explore → navigate
│   │   ├── vln_parser_node.py             # /vln/instruction → Gemini → /vln/goal_stack
│   │   ├── structured_out.py              # Pydantic GoalStack schema + Gemini client
│   │   ├── map_odom_bridge.py             # Cartographer TF → /odom_map at 10 Hz
│   │   └── utils.py                       # Shared helpers
│   ├── stt/
│   │   ├── stt_server.py                  # WebSocket: audio → Whisper → Gemini → GoalStack JSON
│   │   └── speech_to_text.py              # Whisper wrapper
│   └── config/
│       └── vln_params.yaml                # ROS params
│
├── hackathon_updated/hackathon/scratch/DualMap/   # Perception pipeline
│   ├── applications/
│   │   └── runner_ros.py                  # Entry point — ROS subscriber at 15 Hz
│   ├── dualmap/
│   │   ├── perception.py                  # YOLO + MobileSAM + FastSAM + CLIP + DBSCAN
│   │   └── mobility.py                    # CLIP cosine similarity object classifier
│   ├── config/
│   │   └── runner_ros.yaml                # ros_rate=15, use_fastsam, etc.
│   ├── model/                             # Model weight files
│   └── environment.yml
│
├── turtlebot3_simulations/
│   └── turtlebot3_gazebo/launch/
│       ├── gz_vln_exploration.launch.py   # Main entry point
│       ├── gz_vln_gui.launch.py           # GUI variant (rosbridge + STT)
│       └── gz_rl_exploration.launch.py    # RL variant (TD3 replaces Nav2 local planner)
│
├── rl_nav/rl_nav/
│   └── rl_nav_node.py                     # NavigateToPose backed by TD3 actor MLP
│
├── turtlebot3_drl/                        # TD3 training
│   └── turtlebot3_drl/
│       ├── drl_agent/                     # TD3 actor/critic, replay buffer, training loop
│       ├── drl_environment/               # Gazebo reward, reset, step
│       └── drl_gazebo/                    # Gazebo plugin bridge
│
├── m-explore-ros2/explore_lite/           # Frontier exploration (Wavefront BFS)
├── turtlebot3/                            # TurtleBot3 ROS 2 packages
├── turtlebot3_msgs/                       # TurtleBot3 message definitions
└── aws-robomaker-small-house-world/       # small_house.world

VLN-Robot-Control/
└── my-app/                                # Next.js web GUI
    ├── app/
    ├── components/                        # Voice control, status display
    └── hooks/                             # rosbridge connection, STT stream
```

---

## References

1. ROBOTIS. *TurtleBot3.* https://github.com/ROBOTIS-GIT/turtlebot3, 2023.
2. S. Macenski et al. *The Marathon 2: A Navigation System.* IEEE/RSJ IROS, 2020.
3. S. Fujimoto, H. van Hoof, D. Meger. *Addressing Function Approximation Error in Actor-Critic Methods.* ICML, 2018.
