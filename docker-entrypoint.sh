#!/bin/bash
set -e

# Source ROS2 environment
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source workspace if built
if [ -f "${WORKSPACE}/install/setup.bash" ]; then
    source ${WORKSPACE}/install/setup.bash
fi

# Activate Python virtual environment
if [ -d "${WORKSPACE}/venv" ]; then
    source ${WORKSPACE}/venv/bin/activate
fi

# Execute the command passed to docker run
exec "$@"
