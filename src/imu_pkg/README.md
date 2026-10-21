# IMU

## Purpose

The `imu` node takes and stores data from an IMU on the rov and publishes it to the /imu topic for other nodes to use.



## How to run
### Running with default configs
```
ros2 run imu_pkg imu_node
```

## Subscribed Topics

This node doesn't subscribe to any topics.

## Published Topics

| Topic  | Type                  | Description                      |
| ------ | --------------------- | -------------------------------- |
| `/imu` | `sensor_msgs/msg/Imu` | IMU data for other nodes to use. |


## Parameters

| Parameter            |             Default                | Description                                              |
| -------------------- | ---------------------------------: | -------------------------------------------------------- |
| `addr`               |                                `7` | I2C bus number of the IMU                                |
| `sampling_rate`      |                              `100` | Rate of publishing to topic, in Hz                       |
| `q_mount`            |`[0.0, 0.0, -0.7071068, 0.7071068]` | Array used to determine body orientation, etc            |

## Configuration

This node has no config file. It is configured only through the parameters above.

## Methods

| Method                                 | Description                                                                                                        |
| -------------------------------------- | ------------------------------------------------------------------------------------------------------------------ |
| `__init__()`                     | Initializes the node, parameters, imu, publisher, and timer. |
| `_quat_mul()`                    | Multiplies two quaternions                                     |
| `_rotate_vec()`                  | Rotates a vector by a quaternion                     |
| `timer_callback()`               |  Reads data from IMU and publishes it to the /imu topic based on whatever control loop freq is provided.      |
| `main()`                         | Initializes rclpy, starts the node, and runs the rclpy spin method on the node.              |

## Dependencies

### ROS 2 Packages

* `rclpy`
* `sensor_msgs`

### Python Packages

* `numpy`
* `adafruit-circuitpython-bno08x`
* `adafruit-extended-bus`

### Project Packages

* `imu_pkg` — the IMU driver (`imu_pkg.imu_driver`) is in this same package.

## External Documentation

* [sensor_msgs/msg/Imu](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Imu.html)
* [Adafruit BNO085 guide](https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085)
