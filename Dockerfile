# ROS2 Feedback Planner - Docker Container
# Based on ROS2 Jazzy with TIAGo simulation and navigation support

FROM osrf/ros:jazzy-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV WORKSPACE=/workspace

# Add Gazebo repository for python3-gz-transport13
RUN apt-get update && apt-get install -y wget gnupg lsb-release && \
    wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update

# Install system dependencies
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    python3-vcstool \
    wget \
    curl \
    vim \
    tmux \
    libopencv-dev \
    python3-opencv \
    # Gazebo/Ignition dependencies
    ros-${ROS_DISTRO}-ros-gz \
    # Navigation dependencies
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    # MoveIt2 dependencies
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-py \
    ros-${ROS_DISTRO}-moveit-ros-planning-interface \
    # Additional ROS2 packages
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR ${WORKSPACE}
RUN mkdir -p ${WORKSPACE}/src

# Copy the ros2_feedback_planner package from build context
COPY . ${WORKSPACE}/src/ros2_feedback_planner/

# Import third-party dependencies using vcstool
RUN cd ${WORKSPACE}/src && \
    vcs import < ros2_feedback_planner/thirdparty.repos

# Install ROS dependencies using rosdep
RUN cd ${WORKSPACE} && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Install gz-python for Python bindings (not included in ROS2 Jazzy base image)
RUN apt-get update && \
    apt-get install -y python3-gz-transport13 || \
    echo "Warning: python3-gz-transport13 not available, gz.transport13 may not work" && \
    rm -rf /var/lib/apt/lists/*

# Create Python virtual environment and install Python dependencies
# Using --system-site-packages to access gz.transport13 and other system Python packages
RUN python3 -m venv --system-site-packages ${WORKSPACE}/venv && \
    . ${WORKSPACE}/venv/bin/activate && \
    pip install --upgrade pip && \
    pip install -r ${WORKSPACE}/src/ros2_feedback_planner/requirements.txt

# Build the workspace in two stages (using bash shell for ROS setup script)
SHELL ["/bin/bash", "-c"]
# Stage 1: Build third-party packages with --symlink-install (needed for panda_description scripts)
RUN cd ${WORKSPACE} && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release" \
    --packages-skip ros2_feedback_planner

# Stage 2: Build ros2_feedback_planner packages without --symlink-install (for Python venv compatibility)
RUN cd ${WORKSPACE} && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source ${WORKSPACE}/install/setup.bash && \
    colcon build --merge-install --cmake-args "-DCMAKE_BUILD_TYPE=Release" \
    --packages-select ros2_feedback_planner

# Setup entrypoint
COPY ./docker-entrypoint.sh /
RUN chmod +x /docker-entrypoint.sh

# Environment setup for runtime
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source ${WORKSPACE}/install/setup.bash" >> ~/.bashrc && \
    echo "source ${WORKSPACE}/venv/bin/activate" >> ~/.bashrc

# Expose common ROS/Gazebo ports
EXPOSE 11345 11346

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]
