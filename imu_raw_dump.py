import time
import csv
import os
import smbus
import datetime
import IMU
import BMP280 as baro
import mean_filter
DEL = ","  # Data item delimiter.
GYRO_GAIN = 0.070  # Gyro deg/s/ per LSB.
# Magnetometer min/max calibrated values.
MAG_CALIBRATION = [[None, None], [None, None], [None, None]]

# Get I2C bus
bus = smbus.SMBus(1)


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
        acc = [IMU.readACCx() * 0.244 / 1000, -IMU.readACCy() * 0.244 /
               1000, -IMU.readACCz() * 0.244 / 1000]
        # Units: deg/s
        gyro = [IMU.readGYRx() * GYRO_GAIN, IMU.readGYRy() *
                GYRO_GAIN, IMU.readGYRz() * GYRO_GAIN]
        # Units: ?
        mag = [IMU.readMAGx(), IMU.readMAGy(), IMU.readMAGz()]

        filter.add_data(acc, gyro, mag)
        clean = filter.update_filter()

        print(str(datetime.datetime.now() - start_time) +
              str(acc[0]) + DEL + str(acc[1]) + DEL + str(acc[2]) + "\t\t" +
              str(gyro[0]) + DEL + str(gyro[1]) + DEL + str(gyro[2]) + "\t\t" +
              str(clean[0][0]) + DEL + str(clean[0][1]) + DEL + str(clean[0][2]) + DEL +
              str(clean[1][0]) + DEL + str(clean[1][1]) + DEL + str(clean[1][2]))

        file.writerow([datetime.datetime.now() - start_time, acc[0], acc[1], acc[2],
                       gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2], baroValues[0], baroValues[1],
                       str(clean[0][0]), str(clean[0][1]), str(clean[0][2]),
                       str(clean[1][0]), str(clean[1][1]), str(clean[1][2]),
                       str(clean[2][0]), str(clean[2][1]), str(clean[2][2])])

except KeyboardInterrupt:
    pass
