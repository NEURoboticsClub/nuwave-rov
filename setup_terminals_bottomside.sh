#!/bin/bash

# Runs the following commands in seperate terminals:
# - ros2 launch thruster_pkg multi_thruster.launch.py
# - ros2 launch thruster_pkg multi_arm_motor.launch.py
# - ros2 launch power_monitor_pkg multi_power_monitor.launch.py
# - ros2 run rov_depth_sensor depth_sensor_node
# - ros2 run camera_pkg camera_publisher --ros-args -p camera_id:=0
# - ros2 run imu_pkg imu_pub

# Usage: setup_terminals_bottomside.sh [-d] [-s]
#   -d: skip colcon build step
#   -s: run commands over SSH on the ROV


# Local workspace (used when running without -s)
WS="$HOME/nuwave-rov"

# Remote settings
REMOTE_HOST="${ROV_HOST:-rov}"
REMOTE_WS="${ROV_REMOTE_WS:-/home/nuwave-rov/nuwave-rov}"

SKIP_BUILD=false
USE_SSH=false

usage() {
    cat <<EOF
Usage: ./$(basename "$0") [-d] [-s] [-h]

Spawns all ROS2 nodes for bottomside (ROV) in a terminator window.

Panes:
  Multi-Thruster   ros2 launch thruster_pkg multi_thruster.launch.py
  Multi-Arm        ros2 launch thruster_pkg multi_arm_motor.launch.py
  Power Monitor    ros2 launch power_monitor_pkg multi_power_monitor.launch.py
  Depth Sensor     ros2 run rov_depth_sensor depth_sensor_node
  IMU              ros2 run imu_pkg imu_pub
  Camera           ros2 run camera_pkg camera_publisher --ros-args -p camera_id:=0

Options:
  -d    Skip colcon build step (faster if already compiled)
  -s    Run nodes over SSH on the ROV instead of locally
        Requires an SSH alias 'rov' in ~/.ssh/config, or override with:
          ROV_HOST=user@host ROV_REMOTE_WS=/path/to/ws ./$(basename "$0") -s
  -h    Show this help menu

Examples:
  ./$(basename "$0")          # Local: build and launch
  ./$(basename "$0") -d       # Local: skip build, just launch
  ./$(basename "$0") -s       # Remote: build on ROV and launch over SSH
  ./$(basename "$0") -d -s    # Remote: skip build, launch over SSH
EOF
}

while getopts "dsh" opt; do
    case $opt in
        d) SKIP_BUILD=true ;;
        s) USE_SSH=true ;;
        h) usage; exit 0 ;;
        \?) usage; exit 1 ;;
    esac
done

if [ "$USE_SSH" = true ]; then
    echo "Starting bottomside setup over SSH ($REMOTE_HOST)..."

    if [ "$SKIP_BUILD" = false ]; then
        echo "Running colcon build on $REMOTE_HOST..."
        ssh $SSH_OPTS "$REMOTE_HOST" \
            "cd $REMOTE_WS && source venv/bin/activate && colcon build" \
            || { echo "colcon build failed on remote!"; exit 1; }
    fi

    SETUP="source /opt/ros/humble/setup.bash && cd $REMOTE_WS && source $REMOTE_WS/venv/bin/activate && source $REMOTE_WS/install/setup.bash"

    wrap_ssh() { echo "ssh -t $REMOTE_HOST \"$1\""; }

    MULTI_THRUSTER_LAUNCH=$(wrap_ssh "$SETUP && ros2 launch thruster_pkg multi_thruster.launch.py")
    MULTI_ARM_LAUNCH=$(wrap_ssh      "$SETUP && ros2 launch thruster_pkg multi_arm_motor.launch.py")
    POWER_MONITOR_LAUNCH=$(wrap_ssh  "$SETUP && ros2 launch power_monitor_pkg multi_power_monitor.launch.py")
    DEPTH_SENSOR_LAUNCH=$(wrap_ssh   "$SETUP && ros2 run rov_depth_sensor depth_sensor_node")
    CAMERA_LAUNCH=$(wrap_ssh         "$SETUP && ros2 run camera_pkg camera_publisher --ros-args -p camera_id:=0")
    IMU_LAUNCH=$(wrap_ssh            "$SETUP && ros2 run imu_pkg imu_pub")
else
    echo "Starting bottomside setup locally..."
    echo "WS is $WS"
    cd "$WS" && source "$WS/venv/bin/activate"

    if [ "$SKIP_BUILD" = false ]; then
        echo "Running colcon build..."
        cd "$WS" && colcon build
        if [ $? -ne 0 ]; then
            echo "colcon build failed!"
            exit 1
        fi
    fi

    SETUP="cd $WS && source $WS/venv/bin/activate && source $WS/install/setup.bash"

    MULTI_THRUSTER_LAUNCH="$SETUP && ros2 launch thruster_pkg multi_thruster.launch.py"
    MULTI_ARM_LAUNCH="$SETUP && ros2 launch thruster_pkg multi_arm_motor.launch.py"
    POWER_MONITOR_LAUNCH="$SETUP && ros2 launch power_monitor_pkg multi_power_monitor.launch.py"
    DEPTH_SENSOR_LAUNCH="$SETUP && ros2 run rov_depth_sensor depth_sensor_node"
    CAMERA_LAUNCH="$SETUP && ros2 run camera_pkg camera_publisher --ros-args -p camera_id:=0"
    IMU_LAUNCH="$SETUP && ros2 run imu_pkg imu_pub"
fi

LAYOUT_FILE=$(mktemp /tmp/terminator_ros2_setup_bottomside_XXXX.conf)

cat > "$LAYOUT_FILE" <<EOF
[global_config]
[keybindings]
[profiles]
  [[default]]
[layouts]
  [[ros2]]
    [[[window0]]]
      type = Window
      parent = ""
    [[[hpane]]]
      type = HPaned
      parent = window0
    [[[vpane_left_top]]]
      type = VPaned
      parent = hpane
    [[[vpane_left_bottom]]]
      type = VPaned
      parent = vpane_left_top
    [[[vpane_right_top]]]
      type = VPaned
      parent = hpane
    [[[vpane_right_bottom]]]
      type = VPaned
      parent = vpane_right_top
    [[[top-left]]]
      type = Terminal
      parent = vpane_left_top
      title = Multi-Thruster
      command = bash -c '$MULTI_THRUSTER_LAUNCH; exec bash'
    [[[middle-left]]]
      type = Terminal
      parent = vpane_left_bottom
      title = Power Monitor
      command = bash -c '$POWER_MONITOR_LAUNCH; exec bash'
    [[[bottom-left]]]
      type = Terminal
      parent = vpane_left_bottom
      title = Depth Sensor
      command = bash -c '$DEPTH_SENSOR_LAUNCH; exec bash'
    [[[top-right]]]
      type = Terminal
      parent = vpane_right_top
      title = Multi-Arm
      command = bash -c '$MULTI_ARM_LAUNCH; exec bash'
    [[[middle-right]]]
      type = Terminal
      parent = vpane_right_bottom
      title = IMU
      command = bash -c '$IMU_LAUNCH; exec bash'
    [[[bottom-right]]]
      type = Terminal
      parent = vpane_right_bottom
      title = Camera
      command = bash -c '$CAMERA_LAUNCH; exec bash'
[plugins]
EOF

terminator --no-dbus -g "$LAYOUT_FILE" -l ros2 -m
sleep 1
rm -f "$LAYOUT_FILE"

# Tear down the master connection
if [ "$USE_SSH" = true ]; then
    ssh -O exit -o ControlPath="$CTRL_PATH" "$REMOTE_HOST" 2>/dev/null
fi