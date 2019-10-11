import time
import csv
import os
import smbus
import datetime
import math
import IMU
import BMP280 as baro
import mean_filter
DEL = ", "  # Data item delimiter.
# Conversion from encoded value to m/s^2
IMU_ACC_COEFF = 0.244 / 1000.0 * 9.81
# Conversion from encoded value to deg/s
IMU_GYRO_COEFF = 0.070  # Gyro deg/s/ per LSB.
# Magnetometer min/max calibrated values.
MAG_CALIBRATION = [[None, None], [None, None], [None, None]]
# Loop period (sec)
LOOP_PERIOD = 1.0 / 50.0

# I2C bus for BMP280
bus = smbus.SMBus(1)


def form(decimal, digits=3):
    # https://stackoverflow.com/questions/783897/truncating-floats-in-python
    return str(math.floor(decimal * 10**digits) / 10**digits)


folder_path = "imu_raw_dump/"
file_root = "imu_dump"
file_suffix = 0
if not os.path.exists(folder_path):
    os.makedirs(folder_path)
while os.path.isfile(folder_path + file_root + str(file_suffix) + ".csv"):
    file_suffix += 1

file = csv.writer(open(folder_path + file_root + str(file_suffix) + ".csv", 'w'),
                  delimiter=',')
file.writerow(["time (s)", "accX", "accY", "accZ", "gyroX", "gyroY", "gyroZ",
               "magX", "magY", "magZ", "baroTemp", "baroPressure", "accX Filter",
               "accY Filter", "accZ Filter", "gyroX Filter", "gyroY Filter",
               "gyroZ Filter", "magX Filter", "magY Filter", "magZ Filter"])

IMU.detectIMU()     # Detect if BerryIMUv1 or BerryIMUv2 is connected.
IMU.initIMU()       # Initialise the accelerometer, gyroscope and compass

try:
    filter = mean_filter.IMUFilter(10)
    start_time = datetime.datetime.now()
    while True:
        baroValues = baro.get_baro_values(bus)
        # Units: g's
        acc = [IMU.readACCx() * IMU_ACC_COEFF,
               -IMU.readACCy() * IMU_ACC_COEFF,
               -IMU.readACCz() * IMU_ACC_COEFF]
        # Units: deg/s
        gyro = [IMU.readGYRx() * IMU_GYRO_COEFF,
                IMU.readGYRy() * IMU_GYRO_COEFF,
                IMU.readGYRz() * IMU_GYRO_COEFF]
        # Units: ?
        mag = [IMU.readMAGx(), IMU.readMAGy(), IMU.readMAGz()]

        filter.add_data(acc, gyro, mag)
        clean = filter.update_filter()

        print(form((datetime.datetime.now() - start_time).total_seconds(), 4) + DEL +
              form(acc[0]) + DEL + form(acc[1]) + DEL + form(acc[2]) + "\t\t" +
              form(gyro[0]) + DEL + form(gyro[1]) + DEL + form(gyro[2]) + "\t\t" +
              form(clean[0][0]) + DEL + form(clean[0][1]) + DEL + form(clean[0][2]) + DEL +
              form(clean[1][0]) + DEL + form(clean[1][1]) + DEL + form(clean[1][2]))

        file.writerow([(datetime.datetime.now() - start_time).total_seconds(),
                       acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2],
                       mag[0], mag[1], mag[2], baroValues[0], baroValues[1],
                       str(clean[0][0]), str(clean[0][1]), str(clean[0][2]),
                       str(clean[1][0]), str(clean[1][1]), str(clean[1][2]),
                       str(clean[2][0]), str(clean[2][1]), str(clean[2][2])])

        time.sleep(LOOP_PERIOD)
except KeyboardInterrupt:
    pass
