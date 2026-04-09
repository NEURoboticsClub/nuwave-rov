#!/bin/bash

WS="$HOME/nuwave-rov"

# Build first
echo "Starting topside setup..."
echo WS is $WS
cd "$WS" && source $WS/venv/bin/activate

# Parse flags
SKIP_BUILD=false
while getopts "d" opt; do
    case $opt in
        d) SKIP_BUILD=true ;;
    esac
done

if [ "$SKIP_BUILD" = false ]; then
    echo "Running colcon build..."
    cd "$WS" && colcon build
    if [ $? -ne 0 ]; then
        echo "colcon build failed!"
        exit 1
    fi
fi


SETUP="cd $WS && source $WS/venv/bin/activate && source $WS/install/setup.bash"


# Runs the following commands in seperate terminals:
# - ros2 launch houston_pkg joystick_launch.launch.py
# - ros2 run houston_pkg houston
# - ros2 run controller thruster_controller_node
# - ros2 run arm_controller arm_controller_node
# - ros2 run web_gui bridge_node


JOYSTICK_LAUNCH="$SETUP && ros2 launch houston_pkg joystick_launch.launch.py"
START_HOUSTON="$SETUP && ros2 run houston_pkg houston"
START_THRUSTER_CONTROLLER="$SETUP && ros2 run controller thruster_controller_node"
START_ARM_CONTROLLER="$SETUP && ros2 run arm_controller arm_controller_node"
START_WEB_GUI="$SETUP && ros2 run web_gui bridge_node"

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
[plugins]
EOF

terminator --no-dbus -g "$LAYOUT_FILE" -l ros2 -m
sleep 1
rm -f "$LAYOUT_FILE"