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

CMD_TL="$SETUP && ros2 launch houston_pkg joystick_launch.launch.py"
CMD_TR="$SETUP && ros2 run houston_pkg houston"
CMD_BL="$SETUP && ros2 run controller thruster_controller_node"
CMD_BR="$SETUP && ros2 run arm_controller arm_controller_node"

LAYOUT_FILE=$(mktemp /tmp/terminator_ros2_XXXX.conf)

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
      command = bash -c '$CMD_TL; exec bash'
    [[[bottom-left]]]
      type = Terminal
      parent = vpane_left
      command = bash -c '$CMD_BL; exec bash'
    [[[top-right]]]
      type = Terminal
      parent = vpane_right
      command = bash -c '$CMD_TR; exec bash'
    [[[bottom-right]]]
      type = Terminal
      parent = vpane_right
      command = bash -c '$CMD_BR; exec bash'
[plugins]
EOF

terminator -g "$LAYOUT_FILE" -l ros2 -m
rm -f "$LAYOUT_FILE"