#!/usr/bin/env bash
set -e

# Source ROS 2 underlay
source /opt/ros/humble/setup.bash

# Source overlay if it exists
if [ -f "/root/nuwave-rov/install/setup.bash" ]; then
  source /root/nuwave-rov/install/setup.bash
fi

exec "$@"
