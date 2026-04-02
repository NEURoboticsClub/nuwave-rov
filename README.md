# neu-underwater-robotics
Repo for the Northeastern underwater robotics team.

### For setting up this repo for the first time, please see the [software setup guide](software_setup.md)
### For best practices for software development, please see the [best practices guide](best_practices.md)
### For best Git Practices, please see [so you want to git](so_you_want_to_git.md)


## Software Overview:
To be written...

## Running instructions

### How to run EVERYTHING:
- colcon build (on both remote and nano)
- Every terminal:
    - srcvenv
        - source ~/nuwave-rov/venv/bin/activate
    - source install/setup.bash
- Unique terminals on remote laptop:
    - ros2 launch houston_pkg joystick_launch.launch.py
        - Launches 2 joystick nodes
    - ros2 run houston_pkg houston
    - ros2 run thruster_commander thruster_commander_node
    - ros2 run arm_commander arm_commander_node
- Unique terminals on the nano:
    - ros2 launch thruster_pkg multi_thruster.launch.py
    - ros2 launch thruster_pkg multi_arm_motor.launch.py
    - ros2 launch power_monitor_pkg multi_power_monitor.launch.py
