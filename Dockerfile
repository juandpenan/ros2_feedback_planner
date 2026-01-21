# ROS2 Feedback Planner - Docker Container
# Based on ROS2 Jazzy with TIAGo simulation and navigation support

FROM osrf/ros:jazzy-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV WORKSPACE=/workspace

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
    # Gazebo dependencies
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-ros-gz \
    # Navigation dependencies
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    # MoveIt2 dependencies
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-ros-planning-interface \
    # Additional ROS2 packages
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR ${WORKSPACE}
RUN mkdir -p ${WORKSPACE}/src

# Clone the ros2_feedback_planner repository
RUN cd ${WORKSPACE}/src && \
    git clone https://github.com/juandpenan/ros2_feedback_planner.git

# Import third-party dependencies using vcstool
RUN cd ${WORKSPACE}/src && \
    vcs import < ros2_feedback_planner/thirdparty.repos

# Install ROS dependencies using rosdep
RUN cd ${WORKSPACE} && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Create Python virtual environment and install Python dependencies
RUN python3 -m venv ${WORKSPACE}/venv && \
    . ${WORKSPACE}/venv/bin/activate && \
    pip install --upgrade pip && \
    pip install -r ${WORKSPACE}/src/ros2_feedback_planner/requirements.txt

# Build the workspace
RUN cd ${WORKSPACE} && \
    . /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

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
