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
# - ros2 launch thruster_pkg multi_thruster.launch.py
# - ros2 launch thruster_pkg multi_arm_motor.launch.py
# - ros2 launch power_monitor_pkg multi_power_monitor.launch.py


MULTI_THRUSTER_LAUNCH="$SETUP && ros2 launch thruster_pkg multi_thruster.launch.py"
MULTI_ARM_LAUNCH="$SETUP && ros2 launch thruster_pkg multi_arm_motor.launch.py"
POWER_MONITOR_LAUNCH="$SETUP && ros2 launch power_monitor_pkg multi_power_monitor.launch.py"
BONUS_TERMINAL="$SETUP"

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
    [[[vpane_left]]]
      type = VPaned
      parent = hpane
    [[[vpane_right]]]
      type = VPaned
      parent = hpane
    [[[top-left]]]
      type = Terminal
      parent = vpane_left
      title = Multi-Thruster
      command = bash -c '$MULTI_THRUSTER_LAUNCH; exec bash'
    [[[bottom-left]]]
      type = Terminal
      parent = vpane_left
      title = Power Monitor
      command = bash -c '$POWER_MONITOR_LAUNCH; exec bash'
    [[[top-right]]]
      type = Terminal
      parent = vpane_right
      title = Multi-Arm
      command = bash -c '$MULTI_ARM_LAUNCH; exec bash'
    [[[bottom-right]]]
      type = Terminal
      parent = vpane_right
      command = bash -c '$BONUS_TERMINAL; exec bash'
[plugins]
EOF

terminator --no-dbus -g "$LAYOUT_FILE" -l ros2 -m
sleep 1
rm -f "$LAYOUT_FILE"