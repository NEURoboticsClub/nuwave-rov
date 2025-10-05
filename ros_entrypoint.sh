#!/bin/bash
set -e

# Source the base ROS 2 installation
source "/opt/ros/$ROS_DISTRO/setup.bash"

# If install/setup.bash does not exist, build the workspace
if [ ! -f "/workspace/install/setup.bash" ]; then
    echo "--- No install found. Building workspace... ---"
    cd /ros2_ws
    colcon build --symlink-install --merge-install
fi

# Source the workspace's installation
source "/workspace/install/setup.bash"

# Environment setup, ready to start
echo "--- Starting nodes ---"

exec "$@"
