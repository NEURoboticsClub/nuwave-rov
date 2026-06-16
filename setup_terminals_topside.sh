#!/bin/bash

# Runs the following commands in separate terminals:
# - ros2 launch houston_pkg joystick_launch.launch.py
# - ros2 run houston_pkg houston
# - ros2 run controller thruster_controller_node
# - ros2 run arm_controller arm_controller_node
# - ros2 run web_gui bridge_node
# - ros2 run crab_recognition_pkg crab_recognition

# Usage: setup_terminals_topside.sh [-d] [-h]
#   -d: skip colcon build step
#   -h: show help

WS="$HOME/nuwave-rov"

usage() {
    cat <<EOF
Usage: ./$(basename "$0") [-d] [-h]

Spawns all ROS2 nodes for topside (laptop) in a terminator window.

Panes:
  Joysticks            ros2 launch houston_pkg joystick_launch.launch.py
  Houston              ros2 run houston_pkg houston
  Thruster Controller  ros2 run controller thruster_controller_node
  Arm Controller       ros2 run arm_controller arm_controller_node
  Web GUI              ros2 run web_gui bridge_node
  Crab Recognition     ros2 run crab_recognition_pkg crab_recognition
  Topside Shell        sourced shell for any additional commands

Options:
  -d    Skip colcon build step (faster if already compiled)
  -h    Show this help menu

Examples:
  ./$(basename "$0")          # Build and launch
  ./$(basename "$0") -d       # Skip build, just launch
EOF
}

# Parse flags
SKIP_BUILD=false
while getopts "dh" opt; do
    case $opt in
        d) SKIP_BUILD=true ;;
        h) usage; exit 0 ;;
        \?) usage; exit 1 ;;
    esac
done

# Build first
echo "Starting topside setup..."
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

JOYSTICK_LAUNCH="$SETUP && ros2 launch houston_pkg joystick_launch.launch.py"
START_HOUSTON="$SETUP && ros2 run houston_pkg houston"
START_THRUSTER_CONTROLLER="$SETUP && ros2 run controller thruster_controller_node"
START_ARM_CONTROLLER="$SETUP && ros2 run arm_controller arm_controller_node"
START_WEB_GUI="$SETUP && ros2 run web_gui bridge_node"
START_CRAB_RECOGNITION="$SETUP && ros2 run crab_recognition_pkg crab_recognition"

LAYOUT_FILE=$(mktemp /tmp/terminator_ros2_setup_topside_XXXX.conf)

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
      title = Joysticks
      command = bash -c '$JOYSTICK_LAUNCH; exec bash'
    [[[middle-left]]]
      type = Terminal
      parent = vpane_left_bottom
      title = Thruster Controller
      command = bash -c '$START_THRUSTER_CONTROLLER; exec bash'
    [[[bottom-left]]]
      type = Terminal
      parent = vpane_left_bottom
      title = Topside Shell
      command = bash -c '$SETUP; exec bash'
    [[[top-right]]]
      type = Terminal
      parent = vpane_right_top
      title = Houston
      command = bash -c '$START_HOUSTON; exec bash'
    [[[middle-right]]]
      type = Terminal
      parent = vpane_right_bottom
      title = Arm Controller
      command = bash -c '$START_ARM_CONTROLLER; exec bash'
    [[[bottom-right]]]
      type = Terminal
      parent = vpane_right_bottom
      title = Web GUI
      command = bash -c '$START_WEB_GUI; exec bash'
    [[[bottom-middle]]]
      type = Terminal
      parent = vpane_left_bottom
      title = Crab Recognition
      command = bash -c '$START_CRAB_RECOGNITION; exec bash'
[plugins]
EOF

terminator --no-dbus -g "$LAYOUT_FILE" -l ros2 -m
sleep 1
rm -f "$LAYOUT_FILE"