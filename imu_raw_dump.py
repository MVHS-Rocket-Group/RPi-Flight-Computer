import time
from imu_utils import IMU
DEL = ". "

while True:
  print(str(IMU.readACCx()) + ", " + str(IMU.readACCy()) + ", " + str(IMU.readACCz()) + "\t"
        + str(IMU.readGYRx()) + ", " + str(IMU.readGYRy()) + ", " + str(IMU.readGYRz()) + "\t"
        + str(IMU.readMAGx()) + ", " + str(IMU.readMAGy()) + ", " + str(IMU.readMAGz()) + "\t"
        )
  time.sleep(0.1)