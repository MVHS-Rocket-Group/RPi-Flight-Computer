import time
import csv
import os
from imu_utils import IMU
DEL = ", "  # Data item delimiter

file_suffix = 0
while os.path.isfile("imu_dump" + str(file_suffix) + ".csv"):
    file_suffix += 1

file = csv.writer(open("imu_dump" + str(file_suffix) + ".csv", 'w'),
                  delimiter=DEL, quotechar='|', quoting=csv.QUOTE_MINIMAL)

IMU.detectIMU()     # Detect if BerryIMUv1 or BerryIMUv2 is connected.
IMU.initIMU()       # Initialise the accelerometer, gyroscope and compass

try:
    while True:
        print(str(IMU.readACCx()) + DEL + str(IMU.readACCy()) + DEL
              + str(IMU.readACCz()) + "\t" + str(IMU.readGYRx()) + DEL
              + str(IMU.readGYRy()) + DEL + str(IMU.readGYRz()) + "\t"
              + str(IMU.readMAGx()) + DEL + str(IMU.readMAGy()) + DEL
              + str(IMU.readMAGz()) + "\t")

        csvLine = (str(IMU.readACCx()) + DEL + str(IMU.readACCy()) + DEL
                   + str(IMU.readACCz()) + DEL + str(IMU.readGYRx()) + DEL
                   + str(IMU.readGYRy()) + DEL + str(IMU.readGYRz()) + DEL
                   + str(IMU.readMAGx()) + DEL + str(IMU.readMAGy()) + DEL
                   + str(IMU.readMAGz()))
        file.writerow(csvLine)
        time.sleep(0.1)

except KeyboardInterrupt:
    pass
