# SPDX-FileCopyrightText: 2020 Bryan Siepert, written for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense
import time
import board
import busio
import json
import socket
from adafruit_extended_bus import ExtendedI2C as I2C
# import serial

from adafruit_bno08x import (
    BNO_REPORT_LINEAR_ACCELERATION,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_GAME_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C
# from adafruit_bno08x.uart import BNO08X_UART

from common import utils

class ImuDriver:
    def __init__(self,addr,control_loop_freq):
        self.addr = addr
        self.control_loop_freq = control_loop_freq
        self.init_IMU(self)    

        # velocities
        self.vel_x = 0
        self.vel_y = 0
        self.vel_z = 0

        # accelerations
        self.acc_x = 0
        self.acc_y = 0
        self.acc_z = 0

    def init_IMU(self):
        try:
            i2c = I2C(self.addr) # The argument here was 8, if it doesnt work
            bno = BNO08X_I2C(i2c)

            # uart = busio.UART(board.TX, board.RX, baudrate=3000000, receiver_buffer_size=2048)
            # uart = serial.Serial("/dev/serial0", 115200)
            # bno = BNO08X_UART(uart)

            bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)
            bno.enable_feature(BNO_REPORT_GYROSCOPE)
            bno.enable_feature(BNO_REPORT_MAGNETOMETER)
            bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
        except Exception as e:
            raise RuntimeError("Could not initialize IMU")



    """
    Reads data from the IMU sensor and returns as a dictionary.

    Returns:
        dict: a dictionary of dictionaries, each one containing a set of data from the IMU
            (accelerometer, gyroscope, magnetometer, or quaternion)

    TODO: Make it return linear acceleration and angular vel
          Like this: https://docs.ros2.org/foxy/api/sensor_msgs/msg/Imu.html    
    """
    def read_data(self) -> dict:
        data = {}
        # acceleration
        self.accel_x, self.accel_y, self.accel_z = self.bno.linear_acceleration  # pylint:disable=no-member
        data["acceleration"] = utils.make_xyz_dict(self.accel_x, self.accel_y, self.accel_z)

        # velocity
        self.vel_x = self.vel_x + self.accel_x / self.control_loop_freq
        self.vel_y = self.vel_y + self.accel_y / self.control_loop_freq
        self.vel_z = self.vel_z + self.accel_z / self.control_loop_freq
        data["velocity"] = utils.make_xyz_dict(self.vel_x, self.vel_y, self.vel_z)

        # gyro
        # gyro_x, gyro_y, gyro_z = bno.gyro  # pylint:disable=no-member
        # data["gyroscope"] = utils.make_xyz_dict(gyro_x, gyro_y, gyro_z)

        # magnetometer
        mag_x, mag_y, mag_z = self.bno.magnetic  # pylint:disable=no-member
        self.magnetometer = utils.make_xyz_dict(mag_x, mag_y, mag_z)
        data["magnetometer"] = self.magnetometer

        # quaternion
        quat_i, quat_j, quat_k, quat_real = self.bno.game_quaternion  # pylint:disable=no-member
        self.quaternion = {}
        self.quaternion["i"] = quat_i
        self.quaternion["j"] = quat_j
        self.quaternion["k"] = quat_k
        self.quaternion["real"] = quat_real
        data["game_quaternion"] = self.quaternion

        return data