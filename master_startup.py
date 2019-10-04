import time
import csv
import os
import smbus
import enum
import math
import datetime
import RPi.GPIO as io
from imu_utils import IMU

esc_pwm_pin = 15
# frequency (Hz) = 1 / period (sec)
esc_pwm_freq = 1 / 0.02
# Duty cycle percentage when firewalling the throttle.
esc_max_duty = 100 * 2.0 / 20.0
# Duty cycle percentage when at idle throttle.
esc_min_duty = 100 * 1.0 / 20.0
# Handle for control over the ESC PWM pin. Setup later in `try` block.
esc_pwm = None

# Conversion from encoded value to m/s^2
IMU_ACC_COEFF = 0.244 / 1000.0 * 9.81
# Conversion from encoded value to deg/s
IMU_GYRO_COEFF = 0.070  # Gyro deg/s/ per LSB.

# Loop period (sec)
loop_period = 1.0 / 10.0


class FlightState(enum):
    # Python Enum type: https://www.geeksforgeeks.org/enum-in-python
    ON_PAD = 1
    LAUNCHED = 2
    IN_FREEFALL = 3
    LANDED = 4


class IMUData:
    # https://stackoverflow.com/questions/68645/are-static-class-variables-possible-in-python
    start_time = None
    # acc = [None, None, None]
    # gyro = [None, None, None]
    # mag = [None, None, None]
    # baroPressure?
    # baroTemp?
    # events = []

    def __init__(self, flight_state, time=None, acc=None, gyro=None, mag=None):
        # TODO: Add Barometer recording
        self.flight_state = flight_state

        if(time == None):
            if(start_time == None):
                start_time == datetime.datetime.now()
            self.time = datetime.datetime.now() - start_time
        else:
            self.time = time

        if(acc == None):
            self.acc = [IMU.readACCy() * IMU_ACC_COEFF, IMU.readACCx()
                        * IMU_ACC_COEFF, -IMU.readACCz() * IMU_ACC_COEFF]
        else:
            self.acc = acc

        if(gyro == None):
            self.gyro = [-IMU.readGYRy() * IMU_GYRO_COEFF, IMU.readGYRx()
                         * IMU_GYRO_COEFF,  IMU.readGYRz() * IMU_GYRO_COEFF]
        else:
            self.gyro = gyro

        if(mag == None):
            # DODO: Conversion needed?
            self.mag = [IMU.readMAGx(), IMU.readMAGy(), IMU.readMAGz()]
        else:
            self.mag = mag

    def add_event(self, event):
        self.events.append(event)

    def get_acc_magnitude(self):
        return math.sqrt(math.pow(acc[0], 2) + math.pow(acc[1], 2) + math.pow(acc[2], 2))

    def get_gyro_magnitude(self):
        return math.sqrt(math.pow(gyro[0], 2) + math.pow(gyro[1], 2) + math.pow(gyro[2], 2))

    def get_mag_magnitude(self):
        return math.sqrt(math.pow(mag[0], 2) + math.pow(mag[1], 2) + math.pow(mag[2], 2))

    def formatted_for_log(self):
        events_as_string = ""
        for event in self.events:
            events_as_string += event + ", "

        return [time, flight_state.name, acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2],
                mag[0], mag[1], mag[2], events_as_string]


# Stores which state of flight the rocket is currently in.
current_flight_state = FlightState.ON_PAD

try:
    # Init GPIO PWM output for ESC
    io.setwarnings(False)
    io.setmode(io.BOARD)
    io.setup(esc_pwm_pin, io.OUT)

    # Setup the PWM pin and set it to min command.
    esc_pwm = io.PWM(esc_pwm_pin, esc_pwm_freq)
    esc_pwm.start(esc_min_duty)

    # TODO: Spin up both fans for a second or two.

    # Init IMU
    IMU.detectIMU()     # Detect if BerryIMUv1 or BerryIMUv2 is connected.
    IMU.initIMU()       # Initialise the accelerometer, gyroscope and compass

    # Start log file
    folder_path = "flight_logs/"
    file_root = "flight_log"
    file_suffix = 0
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
    while os.path.isfile(folder_path + file_root + str(file_suffix) + ".csv"):
        file_suffix += 1

    log_file = csv.writer(open(folder_path + file_root + str(file_suffix) + ".csv", 'w'),
                          delimiter=',')
    log_file.writerow(["elapsed time", "flight state", "accX", "accY", "accZ", "gyroX",
                       "gyroY", "gyroZ", "magX", "magY", "magZ", "baroTemp", "baroPressure", "events"])

    # TODO: Start camera recording (do we want to do this later?)

    # Main Loop
    while True:
        # Read in motion data from IMU.
        imu_data = IMUData(current_flight_state)

        # TODO: Compute and send target to ESC
        if current_flight_state == FlightState.ON_PAD:
            # Idle until we detect a launch.
            esc_pwm.ChangeDutyCycle(esc_min_duty)

            # If we're seeing >2g's, then we've almost certainly launched.
            if(imu_data.get_acc_magnitude() > 2 * 9.81):
                current_flight_state = FlightState.LAUNCHED
                imu_data.add_event("launched")
                # TODO: Do we need to do more now?
        elif current_flight_state == FlightState.LAUNCHED:
            # If we're seeing <2g's, then we've almost certainly reached apogee.
            if(imu_data.get_acc_magnitude() > 2 * 9.81):
                current_flight_state = FlightState.IN_FREEFALL
                imu_data.add_event("in freefall")
        elif current_flight_state == FlightState.IN_FREEFALL:
            pass
        elif current_flight_state == FlightState.LANDED:
            pass

        # TODO: Did any important events get triggered?
        # TODO: Did the flight state change? e.g. did we just launch, just land?
        # TODO: Has it been >1 munute since landing? If so, shut off the camera recording and shut down RPi

        # Log current system state to file
        log_file.writerow(imu_data.formatted_for_log())

        time.sleep(loop_period)

except KeyboardInterrupt:
    imu_data = IMUData(current_flight_state)
    imu_data.add_event("manually terminated by SIGTERM")
    log_file.writerow(imu_data.formatted_for_log())
    pass

# TODO: Stop camera recording
esc_pwm.ChangeDutyCycle(esc_min_duty)
time.sleep(1)
esc_pwm.stop()
gpio.cleanup()
