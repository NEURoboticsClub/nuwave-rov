#!/bin/bash

WS="$HOME/nuwave-rov"

# Build first
echo "Starting bottomside setup..."
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
# - ros2 launch thruster_pkg multi_thruster.launch.py
# - ros2 launch thruster_pkg multi_arm_motor.launch.py
# - ros2 launch power_monitor_pkg multi_power_monitor.launch.py
# - ros2 run rov_depth_sensor depth_sensor_node
# - ros2 run camera_pkg camera_publisher --ros-args -p camera_id:=0
# - ros2 run imu_pkg imu_pub


MULTI_THRUSTER_LAUNCH="$SETUP && ros2 launch thruster_pkg multi_thruster.launch.py"
MULTI_ARM_LAUNCH="$SETUP && ros2 launch thruster_pkg multi_arm_motor.launch.py"
POWER_MONITOR_LAUNCH="$SETUP && ros2 launch power_monitor_pkg multi_power_monitor.launch.py"
DEPTH_SENSOR_LAUNCH="$SETUP && ros2 run rov_depth_sensor depth_sensor_node"
CAMERA_LAUNCH="$SETUP && ros2 run camera_pkg camera_publisher --ros-args -p camera_id:=0"
IMU_LAUNCH="$SETUP && ros2 run imu_pkg imu_pub"

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