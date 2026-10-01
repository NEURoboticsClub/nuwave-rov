# Thruster Controller

## Purpose

The `imu_pub` node takes and stores data from an IMU on the rov and publishes it to the /imu topic for other nodes to use. 



## How to run
### Running with default configs
```
ros2 run imu_pkg imu_pub
```

## Published Topics

The node publishes the \imu topic, type `std_msgs/msg/Imu`


## Parameters

| Parameter            |             Default                | Description                                              |
| -------------------- | ---------------------------------: | -------------------------------------------------------- |
| `addr`               |                                `7` | I2C address of the IMU                                   |
| `sampling_rate`      |                              `100` | Time between publishing to topic                         |
| `q_mount`            |`[0.0, 0.0, -0.7071068, 0.7071068]` | Array used to determine bodies, orientation, etc         |

## Methods

| Method                                 | Description                                                                                                        |
| -------------------------------------- | ------------------------------------------------------------------------------------------------------------------ |
| `__init__()`                     | Initializes the node, parameters, imu, publisher, and timer. |
| `_quat_mul()`                    | Multiplies two quaternions (I believe)                                     |
| `_rotate_vec`                    | Rotates a vector by a quaternion                     |
| `timer_callback`                 |  Reads data from IMU and publishes it to the /imu topic based on whatever control loop freq is provided.      |
| `main()`                         | Initializes rclpy, starts the node, and runs the rclpy spin method on the node.              |

## Dependencies

### ROS 2 Packages

* `rclpy`
* `sensor_msgs`
* `imu_pkg`
* `geometry_msgs`

### Python Packages

* `numpy`
* `time`