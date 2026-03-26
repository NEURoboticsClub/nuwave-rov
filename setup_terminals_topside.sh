#!/bin/bash

WS="$HOME/nuwave-rov"

# Build first
echo "Running colcon build..."
cd "$WS" && colcon build
if [ $? -ne 0 ]; then
    echo "colcon build failed!"
    exit 1
fi

SETUP="cd $WS && source ~/nuwave-rov/venv/bin/activate && source install/setup.bash"


# Runs the following commands in seperate terminals:
# - ros2 launch houston_pkg joystick_launch.launch.py
# - ros2 run houston_pkg houston
# - ros2 run controller thruster_controller_node
# - ros2 run arm_controller arm_controller_node


JOYSTICK_LAUNCH="$SETUP && ros2 launch houston_pkg joystick_launch.launch.py"
START_HOUSTON="$SETUP && ros2 run houston_pkg houston"
START_THRUSTER_CONTROLLER="$SETUP && ros2 run controller thruster_controller_node"
START_ARM_CONTROLLER="$SETUP && ros2 run arm_controller arm_controller_node"

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
    [[[vpane_left]]]
      type = VPaned
      parent = hpane
    [[[vpane_right]]]
      type = VPaned
      parent = hpane
    [[[top-left]]]
      type = Terminal
      parent = vpane_left
      title = Joysticks
      command = bash -c '$JOYSTICK_LAUNCH; exec bash'
    [[[bottom-left]]]
      type = Terminal
      parent = vpane_left
      title = Thruster Controller
      command = bash -c '$START_THRUSTER_CONTROLLER; exec bash'
    [[[top-right]]]
      type = Terminal
      parent = vpane_right
      title = Houston
      command = bash -c '$START_HOUSTON; exec bash'
    [[[bottom-right]]]
      type = Terminal
      parent = vpane_right
      title = Arm Controller
      command = bash -c '$START_ARM_CONTROLLER; exec bash'
[plugins]
EOF

terminator --no-dbus -g "$LAYOUT_FILE" -l ros2 -m
sleep 1
rm -f "$LAYOUT_FILE"