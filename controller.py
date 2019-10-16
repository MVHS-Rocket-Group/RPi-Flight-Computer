import time
import csv
import os
import smbus
import enum
import math
import datetime
import subprocess
import picamera as picam
import BMP280 as barometer
import RPi.GPIO as io
import mean_filter as m_filter
import IMU
from dataclasses import dataclass

# Conversion from encoded value to m/s^2
IMU_ACC_COEFF = 0.244 / 1000.0 * 9.81
# Conversion from encoded value to deg/s
IMU_GYRO_COEFF = 0.070  # Gyro deg/s per LSB.
# Time of script start
START_TIME = datetime.datetime.now()

# I2C bus for BMP280
i2c_bus = smbus.SMBus(1)


class FlightState(enum.Enum):
    # Python Enum type: https://www.geeksforgeeks.org/enum-in-python
    ON_PAD = 1
    LAUNCHED = 2
    IN_FREEFALL = 3
    LANDED = 4


@dataclass
class Timestep:
    # https://stackoverflow.com/a/45426493/3339274

    # Units:
    # - time: seconds
    # - acc: [x, y, z] m/s^2
    # - gyro: [x, y, z] deg/s
    # - mag: [x, y, z] µTesla?
    # - baro_p: hPa
    # - baro_temp: deg C

    time: float
    flight_state: FlightState

    # Raw sensor values.
    acc: float[3]
    gyro: float[3]
    mag: float[3]
    baro_p: float
    baro_temp: float

    # Filtered sensor values.
    acc_f: float[3]
    gyro_f: float[3]
    mag_f: float[3]
    baro_p_f: float
    baro_temp_f: float

    @staticmethod
    def new():
        # https://stackoverflow.com/a/735978/3339274

        time = (datetime.datetime.now() - START_TIME).total_seconds()

        acc = [IMU.readACCx() * IMU_ACC_COEFF,
               -IMU.readACCy() * IMU_ACC_COEFF,
               -IMU.readACCz() * IMU_ACC_COEFF]
        # Compensate for gravitational acceleration.
        # TODO: Do we want to do some angle calculations to figure out how much to subtract from x, y, and z?
        acc[0] += 9.81

        gyro = [IMU.readGYRx() * IMU_GYRO_COEFF,
                IMU.readGYRy() * IMU_GYRO_COEFF,
                IMU.readGYRz() * IMU_GYRO_COEFF]

        # TODO: Conversion needed?
        mag = [IMU.readMAGx(), IMU.readMAGy(), IMU.readMAGz()]

        baro_tuple = barometer.get_baro_values(i2c_bus)
        baro_p = baro_tuple[0]
        baro_temp = baro_tuple[1]

        return Timestep(time=time, acc=acc, gyro=gyro, mag=mag, baro_p=baro_p, baro_temp=baro_temp)


class LoopController:
    def __init__(self, filter_buffer_size: int, log_file: csv.DictWriter):
        self.filter = m_filter.IMUFilter(filter_buffer_size)
        self.log = log_file

        self.log.writerow(["time (s)", "flight state", "accX", "accY", "accZ", "gyroX", "gyroY", "gyroZ",
                           "magX", "magY", "magZ", "accX Filter", "accY Filter", "accZ Filter", "gyroX Filter", "gyroY Filter",
                           "gyroZ Filter", "magX Filter", "magY Filter", "magZ Filter", "baroPressure", "baroTemp", "baroPressure Filter", "baroTemp Filter"])

    def run_iteration(self):
        if(self.flight_state == None):
            self.flight_state = FlightState.ON_PAD

        self.read_sensors()

        self.data.flight_state = self.flight_state
        self.log.writerow([self.data.time, self.data.flight_state.name,
                           self.data.acc[0], self.data.acc[1], self.data.acc[2],
                           self.data.gyro[0], self.data.gyro[1], self.data.gyro[2],
                           self.data.mag[0], self.data.mag[1], self.data.mag[2],
                           self.data.acc_f[0], self.data.acc_f[1], self.data.acc_f[2],
                           self.data.gyro_f[0], self.data.gyro_f[1], self.data.gyro_f[2],
                           self.data.mag_f[0], self.data.mag_f[1], self.data.mag_f[2],
                           self.data.baro_p, self.data.baro_temp,
                           self.data.baro_p_f, self.data.baro_temp_f])

    def read_sensors(self):
        time = (datetime.datetime.now() - START_TIME).total_seconds()

        acc = [IMU.readACCx() * IMU_ACC_COEFF,
               -IMU.readACCy() * IMU_ACC_COEFF,
               -IMU.readACCz() * IMU_ACC_COEFF]
        # Compensate for gravitational acceleration.
        # TODO: Do we want to do some angle calculations to figure out how much to subtract from x, y, and z?
        acc[0] += 9.81

        gyro = [IMU.readGYRx() * IMU_GYRO_COEFF,
                IMU.readGYRy() * IMU_GYRO_COEFF,
                IMU.readGYRz() * IMU_GYRO_COEFF]

        # TODO: Conversion needed?
        mag = [IMU.readMAGx(), IMU.readMAGy(), IMU.readMAGz()]

        self.filter.add_data(acc, gyro, mag)
        filtered = self.filter.update_filter()

        baro_tuple = barometer.get_baro_values(i2c_bus)
        baro_p = baro_tuple[0]
        baro_temp = baro_tuple[1]

        self.data = Timestep(time=time, acc=acc, gyro=gyro, mag=mag,
                             acc_f=filtered[0], gyro_f=filtered[1], mag_f=filtered[2], baro_p=baro_p, baro_temp=baro_temp)
