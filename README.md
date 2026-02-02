# ROS2 Feedback Planner


A ROS2 package for LLM-based robot planning with visual feedback. The planner uses Large Language Models (OpenAI GPT, Google Gemini, or Hugging Face models) to generate and adapt navigation and manipulation plans based on real-time camera feedback and execution results.


## Table of Contents

- [Prerequisites](#prerequisites)
- [Installation](#installation)
  - [Native Installation](#native-installation)
  - [Docker Installation](#docker-installation)
- [Configuration](#configuration)
- [Usage](#usage)
  - [Navigation Demo](#navigation-demo)
  - [Manipulation Demo](#manipulation-demo)
- [Architecture](#architecture)
- [API Keys](#api-keys)
- [Troubleshooting](#troubleshooting)
- [Citation](#citation)

## Prerequisites

### System Requirements
- **OS**: Ubuntu 24.04 (Noble)
- **ROS2**: Jazzy Jalisco
- **Python**: 3.12+
- **GPU**: Recommended for local model inference (optional)
- **RAM**: 16GB minimum, 32GB recommended

### Required Software
- ROS2 Jazzy Desktop Full
- Gazebo Harmonic
- Python 3.12+
- Git
- vcstool

## Installation

### Native Installation

#### 1. Install ROS2 Jazzy

Follow the [official ROS2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation.html):

```bash
# Add ROS2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy
sudo apt update
sudo apt install ros-jazzy-desktop-full
```

#### 2. Create Workspace and Clone Repository

```bash
# Create workspace
mkdir -p ~/ros2_feedback_ws/src
cd ~/ros2_feedback_ws/src

# Clone the main repository
git clone https://github.com/juandpenan/ros2_feedback_planner.git

# Import third-party dependencies
vcs import < ros2_feedback_planner/thirdparty.repos
```

#### 3. Install Dependencies

```bash
cd ~/ros2_feedback_ws

# Install ROS dependencies
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Create Python virtual environment
python3 -m venv venv
source venv/bin/activate

# Install Python dependencies
pip install --upgrade pip
pip install -r src/ros2_feedback_planner/requirements.txt
```

#### 4. Build the Workspace

```bash
cd ~/ros2_feedback_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

#### 5. Source the Workspace

```bash
source ~/ros2_feedback_ws/install/setup.bash
source ~/ros2_feedback_ws/venv/bin/activate
```

Add to your `~/.bashrc` for convenience:
```bash
echo "source ~/ros2_feedback_ws/install/setup.bash" >> ~/.bashrc
echo "source ~/ros2_feedback_ws/venv/bin/activate" >> ~/.bashrc
```

---

### Docker Installation

#### 1. Build the Docker Image

```bash
cd ~/ros2_feedback_ws/src/ros2_feedback_planner
docker build -t ros2_feedback_planner:jazzy .
```

#### 2. Run the Container

```bash
# Basic run
docker run -it --rm \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ros2_feedback_planner:jazzy

# With GPU support (NVIDIA)
docker run -it --rm \
  --network host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ros2_feedback_planner:jazzy

# With API keys (pass directly)
docker run -it --rm \
  --network host \
  -e DISPLAY=$DISPLAY \
  -e OPENAI_API_KEY="your_openai_key_here" \
  -e GOOGLE_API_KEY="your_gemini_key_here" \
  -e HF_TOKEN="your_hf_token_here" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ros2_feedback_planner:jazzy

# With API keys (source from host environment)
docker run -it --rm \
  --network host \
  -e DISPLAY=$DISPLAY \
  -e OPENAI_API_KEY \
  -e GOOGLE_API_KEY \
  -e HF_TOKEN \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ros2_feedback_planner:jazzy

# With API keys from .env file
docker run -it --rm \
  --network host \
  -e DISPLAY=$DISPLAY \
  --env-file .env \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ros2_feedback_planner:jazzy
```

---

## Configuration

### API Keys

The planner supports multiple LLM providers. Set up API keys for your preferred provider:

#### OpenAI (GPT Models)
```bash
export OPENAI_API_KEY="sk-your-api-key-here"
```

#### Google Gemini
```bash
export GOOGLE_API_KEY="your-gemini-api-key-here"
```

#### Hugging Face
```bash
export HF_TOKEN="your-hf-token-here"
```

### Planning Configuration

Edit `ros2_feedback_planner/config/navigation_planner_config.yaml`:

```yaml
planner_node:
  ros__parameters:
    llm_client:
      vendor: 'gemini'  # Options: 'openai', 'gemini', 'huggingface', 'local'
      model_name: 'gemini-flash-latest'
      temperature: 0.5
      max_tokens: 6000
    
    planner_type: 'forecast'  # Options: 'forecast', 'monologue', 'doremi'
    
    forecast:
      feedback_mode: 'continious'  # Options: 'once', 'continious'
      use_image: True
      image_topic: '/head_front_camera/image'
```

### Feedback Configuration

Configure visual feedback analysis in the same YAML file:

```yaml
feedback_node:
  ros__parameters:
    llm_client:
      vendor: 'gemini'
      model_name: 'gemini-flash-lite-latest'
      temperature: 1.0
    
    feedback_type: 'forecast'
    
    forecast:
      probability_threshold: neutral  # Risk threshold for replanning
      image_topic: '/head_front_camera/image'
```

---

## Usage

### Navigation Demo

Launch the navigation demo with TIAGo robot in the Plasys House environment:

```bash
# Terminal 1: Launch Gazebo simulation and navigation
ros2 launch ros2_feedback_planner navigation_demo.launch.py

# Terminal 2: Send a navigation goal
ros2 topic pub /goal_topic std_msgs/String "data: 'go to the kitchen'" --once
```

The robot will:
1. Generate a navigation plan using the configured LLM
2. Execute the plan using Nav2
3. Monitor visual feedback from the camera
4. Replan if obstacles or humans are detected

### Manipulation Demo

Launch the dual manipulator demo for pick-and-place tasks:

```bash
# Terminal 1: Launch manipulation simulation
ros2 launch ros2_feedback_planner manipulation_demo.launch.py

# Terminal 2: Trigger manipulation sequence
ros2 service call /start_manipulation std_srvs/srv/Trigger
```

### Running Individual Nodes

```bash
# Planner node only
ros2 run ros2_feedback_planner planner_node --ros-args --params-file config/navigation_planner_config.yaml

# Feedback node only
ros2 run ros2_feedback_planner feedback_node --ros-args --params-file config/navigation_planner_config.yaml

# Metrics manager
ros2 run ros2_feedback_planner metrics_manager_node --ros-args --params-file config/navigation_planner_config.yaml
```

---

## Architecture

### System Components

```
┌─────────────────────────────────────────────────────┐
│                  ROS2 Feedback Planner              │
├─────────────────────────────────────────────────────┤
│                                                     │
│  ┌──────────────┐      ┌──────────────┐           │
│  │   Planner    │◄────►│   Feedback   │           │
│  │     Node     │      │     Node     │           │
│  └──────┬───────┘      └──────▲───────┘           │
│         │                      │                    │
│         │                      │                    │
│         ▼                      │                    │
│  ┌──────────────┐      ┌──────┴───────┐           │
│  │  Action      │      │   Camera     │           │
│  │  Executor    │      │   Input      │           │
│  └──────┬───────┘      └──────────────┘           │
│         │                                           │
│         ▼                                           │
│  ┌──────────────────────────────────┐             │
│  │  Nav2 / MoveIt2 / Robot Control  │             │
│  └──────────────────────────────────┘             │
│                                                     │
└─────────────────────────────────────────────────────┘
         │                           ▲
         │                           │
         ▼                           │
    ┌────────────────────────────────┴────┐
    │      Gazebo Simulation              │
    │  (TIAGo / Panda Robot + Environment)│
    └─────────────────────────────────────┘
```

### Key Nodes

- **planner_node**: Generates and manages robot plans using LLMs
- **feedback_node**: Analyzes camera images to assess preconditions and trigger replanning
- **metrics_manager_node**: Tracks performance metrics and experimental data
- **robot_controller_node**: Low-level robot control interface

### Planning Strategies

#### Forecast
Predicts future preconditions and fallback actions before execution. Continuously monitors the environment and replans proactively when preconditions are at risk.

#### Monologue
Uses inner-monologue reasoning to determine the next best action. Analyzes feedback after each action to decide the next step adaptively.

#### DoReMi
Dynamic observation and re-evaluation of preconditions. Checks current precondition violations and generates reactive plans.

---

## Troubleshooting

### Common Issues

#### 1. API Key Not Found
```
ERROR: API key not set for vendor 'gemini'
```
**Solution**: Export the required API key:
```bash
export GOOGLE_API_KEY="your-api-key"
```

#### 2. Import Error: No module named 'openai'
```
ModuleNotFoundError: No module named 'openai'
```
**Solution**: Activate venv and reinstall requirements:
```bash
source ~/ros2_feedback_ws/venv/bin/activate
pip install -r src/ros2_feedback_planner/requirements.txt
```

#### 3. Gazebo Fails to Start
```
[gazebo-1] process has died
```
**Solution**: Check GPU drivers and Gazebo installation:
```bash
# Test Gazebo
gz sim --version

# Reinstall if needed
sudo apt install ros-jazzy-ros-gz
```

#### 4. Package 'tiago_gazebo' not found
```
Package 'tiago_gazebo' not found
```
**Solution**: Ensure all dependencies were imported:
```bash
cd ~/ros2_feedback_ws/src
vcs import < ros2_feedback_planner/thirdparty.repos
cd ..
colcon build --symlink-install
```

#### 5. Camera topic not receiving data
```
WARN: No image received on topic /head_front_camera/image
```
**Solution**: Verify Gazebo simulation is running and camera is active:
```bash
ros2 topic list | grep image
ros2 topic hz /head_front_camera/image
```

#### 6. Nav2 navigation fails
```
ERROR: Could not get current robot pose
```
**Solution**: Ensure TIAGo navigation is properly launched:
```bash
# Check if nav2 nodes are running
ros2 node list | grep nav2

# Check TF tree
ros2 run tf2_tools view_frames
```

---

## Package Structure

```
ros2_feedback_planner/
├── config/
│   └── navigation_planner_config.yaml    # Configuration file
├── launch/
│   ├── navigation_demo.launch.py         # Navigation demo
│   ├── manipulation_demo.launch.py       # Manipulation demo
│   └── dual_manipulator_multiprocess.launch.py
├── ros2_feedback_planner/
│   ├── __init__.py
│   ├── executor.py                       # Plan execution
│   ├── metrics_manager.py                # Metrics tracking
│   ├── robot_controller_node.py          # Robot control
│   ├── scenario_coordinator.py           # Scenario management
│   ├── utils.py                          # Utility functions
│   ├── feedback/
│   │   └── feedback_server.py            # Feedback generation
│   ├── models/
│   │   └── client_base.py                # LLM client abstraction
│   └── planning/
│       ├── actions.py                    # Robot actions (Nav2, MoveIt)
│       ├── planner_online.py             # Online LLM planning
│       ├── planning_output_formats.py    # Plan formats
│       └── simple_planner.py             # Main planner node
├── feedback_planner_interfaces/          # Custom messages/services
├── requirements.txt                      # Python dependencies
├── thirdparty.repos                      # Third-party ROS packages
├── Dockerfile                            # Docker build configuration
├── docker-entrypoint.sh                  # Docker entrypoint script
├── package.xml                           # ROS package manifest
└── README.md                             # This file
```

---

## Development

### Running Tests

```bash
cd ~/ros2_feedback_ws
colcon test --packages-select ros2_feedback_planner
colcon test-result --verbose
```

### Code Style

The project follows PEP 8 style guidelines. Format code with:

```bash
source ~/ros2_feedback_ws/venv/bin/activate
black ros2_feedback_planner/
flake8 ros2_feedback_planner/
```

### Adding New Planning Strategies

1. Create a new planner class in `ros2_feedback_planner/planning/`
2. Implement the planning logic
3. Add configuration in `navigation_planner_config.yaml`
4. Register the planner in `simple_planner.py`

---

## Citation

If you use this package in your research, please cite:


todo
---

## License

This project is licensed under the MIT License - see the LICENSE file for details.

---

## Acknowledgments

- TIAGo robot platform by PAL Robotics
- ROS2 Navigation Stack (Nav2)
- MoveIt2 motion planning framework
- OpenAI, Google, and Hugging Face for LLM APIs

---

## Contact
todo
---

## Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

---

## Known Limitations

- Visual feedback requires sufficient GPU resources for real-time processing
- LLM API calls may introduce latency (use local models for faster response)
- Navigation accuracy depends on map quality and localization
- Multi-robot coordination is experimental

---

## Roadmap

- [ ] Support for additional LLM providers (Claude, Llama 3)
- [ ] Real robot deployment (tested on simulation only)
- [ ] Multi-agent coordination improvements
- [ ] ROS2 Iron and Humble backports
- [ ] Web-based monitoring dashboard
- [ ] Improved error recovery strategies

---
