#!/usr/bin/env bash
set -e

# Source ROS 2 underlay
source /opt/ros/humble/setup.bash

# Source overlay if it exists
if [ -f "/workspace/install/setup.bash" ]; then
  source /workspace/install/setup.bash
fi

exec "$@"
