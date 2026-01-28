# ROS2 Feedback Planner

Repo with the implementation of paper : todo

A ROS2 package for LLM-based robot planning with visual feedback. The planner uses Large Language Models (OpenAI GPT, Google Gemini, or Hugging Face models) to generate and adapt navigation and manipulation plans based on real-time camera feedback and execution results.

## Demo Videos

Check out the system in action:

### Navigation Demo


https://github.com/user-attachments/assets/78dc06fd-d787-4ac8-b137-8a1e5047cb67


### Manipulation Demo

https://github.com/user-attachments/assets/98bf1bb0-04ce-4a7b-ac15-30977852f2cd


## Features

- **Multi-modal LLM Planning**: Support for OpenAI, Google Gemini, Hugging Face, and local models
- **Visual Feedback**: Real-time image analysis for adaptive replanning
- **Multiple Planning Strategies**: 
  - **Forecast**: Predicts future preconditions and proactively replans
  - **Monologue**: Inner-monologue style reasoning for next-best actions
  - **DoReMi**: Dynamic observation and re-evaluation
- **Dual Manipulation Support**: Coordinate multiple robot arms
- **Navigation Integration**: Nav2 integration for mobile robot navigation
- **Metrics & Evaluation**: Built-in performance tracking and experiment management

## Table of Contents

- [Prerequisites](#prerequisites)
- [Installation](#installation)
  - [Native Installation](#native-installation)
  - [Docker Installation](#docker-installation)
- [Configuration](#configuration)
- [Usage](#usage)
  - [Navigation Demo](#navigation-demo)
  - [Manipulation Demo](#manipulation-demo)
- [API Keys](#api-keys)
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
# Terminal 1: Launch Gazebo simulation
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True world_name:=plasys_house x:=-5.5 y:=-3.8 Y:=1.708

# Terminal 2: Launch the navigation demo
ros2 launch ros2_feedback_planner navigation_demo.launch.py

# Terminal 3: Configure metrics manager
ros2 lifecycle set /metrics_manager configure

# Terminal 4: Send a navigation goal
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
# Terminal 1: Launch Panda simulation
ros2 launch panda multipanda_gz.launch.py

# Terminal 2: Launch manipulation demo
ros2 launch ros2_feedback_planner manipulation_demo.launch.py

# Terminal 3: Configure metrics manager
ros2 lifecycle set /metrics_manager configure

# Terminal 4: Trigger manipulation sequence
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

## Planning Strategies

#### Forecast
Predicts future preconditions and fallback actions before execution. Continuously monitors the environment and replans proactively when preconditions are at risk.

#### Monologue
Uses inner-monologue reasoning to determine the next best action. Analyzes feedback after each action to decide the next step adaptively.

#### DoReMi
Dynamic observation and re-evaluation of preconditions. Checks current precondition violations and generates reactive plans.

---

## Citation

To be added.

---

## License

This project is licensed under the MIT License - see the LICENSE file for details.

---
