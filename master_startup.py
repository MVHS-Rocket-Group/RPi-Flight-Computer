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


# https://pinout.xyz/#
ESC_PWM_PIN = 15
ARMING_SW_PIN = 8
# frequency (Hz) = 1 / period (sec)
ESC_PWM_FREQ = 1 / 0.02
# Duty cycle percentage when firewalling the throttle.
ESC_MAX_DUTY = 100 * 2.0 / 20.0
# Duty cycle percentage when at idle throttle.
ESC_MIN_DUTY = 100 * 1.0 / 20.0
# Handle for control over the ESC PWM pin. Initialized later in `try` block.
esc_pwm = None
# I2C bus for BMP280
i2c_bus = smbus.SMBus(1)

# Conversion from encoded value to m/s^2
IMU_ACC_COEFF = 0.244 / 1000.0 * 9.81
# Conversion from encoded value to deg/s
IMU_GYRO_COEFF = 0.070  # Gyro deg/s per LSB.

# Loop period (sec)
LOOP_PERIOD = 1.0 / 50.0


class FlightState(enum.Enum):
    # Python Enum type: https://www.geeksforgeeks.org/enum-in-python
    DISARMED = 1
    ON_PAD = 2
    LAUNCHED = 3
    IN_FREEFALL = 4
    LANDED = 5


class IMUData:
    # Units:
    # - time: seconds
    # - acc: [x, y, z] m/s^2
    # - gyro: [x, y, z] deg/s
    # - mag: [x, y, z] µTesla?
    # - baro: [degC, pressure] deg celsius, hPa

    # https://stackoverflow.com/questions/68645/are-static-class-variables-possible-in-python
    start_time = None

    def __init__(self, flight_state, time=None, acc=None, gyro=None, mag=None, baro=None):
        self.flight_state = flight_state

        if(time == None):
            if(IMUData.start_time == None):
                IMUData.start_time = datetime.datetime.now()
            self.time = (datetime.datetime.now() -
                         IMUData.start_time).total_seconds()
        else:
            self.time = time

        if(acc == None):
            self.acc = [IMU.readACCx() * IMU_ACC_COEFF,
                        -IMU.readACCy() * IMU_ACC_COEFF,
                        -IMU.readACCz() * IMU_ACC_COEFF]
            # Compensate for gravitational acceleration.
            # TODO: Do we want to do some angle calculations to figure out how much to subtract from x, y, and z?
            self.acc[0] += 9.81
        else:
            self.acc = acc

        if(gyro == None):
            self.gyro = [IMU.readGYRx() * IMU_GYRO_COEFF,
                         IMU.readGYRy() * IMU_GYRO_COEFF,
                         IMU.readGYRz() * IMU_GYRO_COEFF]
        else:
            self.gyro = gyro

        if(mag == None):
            # TODO: Conversion needed?
            self.mag = [IMU.readMAGx(), IMU.readMAGy(), IMU.readMAGz()]
        else:
            self.mag = mag

        if(baro == None):
            tuple = barometer.get_baro_values(i2c_bus)
            self.baro = [tuple[0], tuple[1]]
        else:
            self.baro = baro

        # Start events list off as empty.
        self.events = []

    def add_event(self, event):
        self.events.append(event)

    def get_acc_magnitude(self):
        return math.sqrt(math.pow(self.acc[0], 2) + math.pow(self.acc[1], 2) + math.pow(self.acc[2], 2))

    def get_gyro_magnitude(self):
        return math.sqrt(math.pow(self.gyro[0], 2) + math.pow(self.gyro[1], 2) + math.pow(self.gyro[2], 2))

    def get_mag_magnitude(self):
        return math.sqrt(math.pow(self.mag[0], 2) + math.pow(self.mag[1], 2) + math.pow(self.mag[2], 2))

    def formatted_for_log(self):
        events_as_string = ""
        for event in self.events:
            events_as_string += event + ", "

        return [self.time, self.flight_state.name,
                self.acc[0], self.acc[1], self.acc[2],
                self.gyro[0], self.gyro[1], self.gyro[2],
                self.mag[0], self.mag[1], self.mag[2],
                self.baro[0], self.baro[1], events_as_string]

    def get_events_list(self):
        return (self.time, self.events)


# Stores which state of flight the rocket is currently in.
current_flight_state = FlightState.DISARMED

# Placeholder for datetime object.
launch_time = None
landed_time = None

cam = picam.PiCamera()
# cam.resolution = (640, 480)
cam.resolution = (1280, 720)


try:
    # Init GPIO PWM output for ESC
    io.setwarnings(False)
    io.setmode(io.BOARD)
    io.setup(ESC_PWM_PIN, io.OUT)
    io.setup(ARMING_SW_PIN, io.IN)

    # Setup the PWM pin and set it to min command.
    esc_pwm = io.PWM(ESC_PWM_PIN, ESC_PWM_FREQ)
    esc_pwm.start(ESC_MIN_DUTY)
    # Let the ESCs sample the duty cycle of ESC_MIN_DUTY.
    time.sleep(3)
    print("ESC MIN_DUTY calibration performed")

    # Init IMU
    IMU.detectIMU()     # Detect if BerryIMUv1 or BerryIMUv2 is connected.
    IMU.initIMU()       # Initialise the accelerometer, gyroscope and compass
    print("IMU Initialized")

    # Start log file
    log_folder = "flight_logs/"
    log_file = "flight_log"
    log_file_suffix = 0
    if not os.path.exists(log_folder):
        os.makedirs(log_folder)
    while os.path.isfile(log_folder + log_file + str(log_file_suffix) + ".csv"):
        log_file_suffix += 1

    log_writer = csv.writer(open(log_folder + log_file + str(log_file_suffix) + ".csv", 'w'),
                            delimiter=',')
    log_writer.writerow(["elapsed time", "flight state", "accX", "accY", "accZ", "gyroX",
                         "gyroY", "gyroZ", "magX", "magY", "magZ", "baroTemp", "baroPressure", "events"])
    print("Log file started: " + log_folder +
          log_file + str(log_file_suffix) + ".csv")

    video_folder = "recordings/"
    video_file = "payload_recording"
    video_file_suffix = 0
    if not os.path.exists(video_folder):
        os.makedirs(video_folder)
    while os.path.isfile(video_folder + video_file + str(video_file_suffix) + ".h264"):
        video_file_suffix += 1

    # Main Loop
    while True:
        # Read in motion data from IMU.
        # TODO: Implement mean_filter.
        imu_data = IMUData(current_flight_state)

        # If we're armed and the arming switch is turned off...
        if current_flight_state != FlightState.DISARMED and io.input(ARMING_SW_PIN):
            imu_data.add_event("FTS manual trigger via arming switch")
            current_flight_state = FlightState.DISARMED
            esc_pwm.ChangeDutyCycle(ESC_MIN_DUTY)
            log_writer.writerow(imu_data.formatted_for_log())
            break

        if current_flight_state == FlightState.DISARMED:
            # Idle the motors because we aren't armed.
            esc_pwm.ChangeDutyCycle(ESC_MIN_DUTY)

            # If we are indeed armed by the 3v3 switch line...
            if io.input(ARMING_SW_PIN):
                current_flight_state = FlightState.ON_PAD
                imu_data.add_event("armed")

                # Spin up both fans for a test.
                esc_pwm.ChangeDutyCycle(ESC_MAX_DUTY)
                time.sleep(1.5)
                esc_pwm.ChangeDutyCycle(ESC_MIN_DUTY)
                print("ESC self-test performed")

                # Start the camera recording.
                # https://picamera.readthedocs.io/en/release-1.13/api_camera.html#picamera.PiCamera.start_recording
                cam.start_recording(video_folder + video_file +
                                    str(video_file_suffix) + ".h264", format="h264")
                print("Recording started: " + video_folder +
                      video_file + str(video_file_suffix) + ".h264 ...")
        if current_flight_state == FlightState.ON_PAD:
            # Idle until we detect a launch.
            esc_pwm.ChangeDutyCycle(ESC_MIN_DUTY)

            # If we're seeing >2g's, then we've almost certainly launched.
            if(imu_data.get_acc_magnitude() > 2 * 9.81):
                current_flight_state = FlightState.LAUNCHED
                launch_time = datetime.datetime.now()
                imu_data.add_event("launched")
                esc_pwm.ChangeDutyCycle(ESC_MAX_DUTY)
        elif current_flight_state == FlightState.LAUNCHED:
            # Set full power.
            esc_pwm.ChangeDutyCycle(ESC_MAX_DUTY)

            # If we're seeing <2g's, then we've almost certainly lost thrust due to SRM burnout.
            if imu_data.get_acc_magnitude() < 2 * 9.81:
                current_flight_state = FlightState.IN_FREEFALL
                imu_data.add_event("in freefall")
        elif current_flight_state == FlightState.IN_FREEFALL:
            # Set full power.
            esc_pwm.ChangeDutyCycle(ESC_MAX_DUTY)

            # If the acceleration magnitude is next to nothing following the freefall, set state to LANDED
            # TODO: Fix this. What we're actually looking for is a spike in acceleration as we hit the ground.
            if abs(imu_data.get_acc_magnitude()) < (0.5 * 9.81):  # Accounting for random noise
                landed_time = datetime.datetime.now()
                current_flight_state = FlightState.LANDED
                imu_data.add_event("landed")
        elif current_flight_state == FlightState.LANDED:
            # Set zero power.
            esc_pwm.ChangeDutyCycle(ESC_MIN_DUTY)

            if datetime.datetime.now() > landed_time + datetime.timedelta(minutes=1):
                # Begin shutdown procedure
                imu_data.add_event("FTS automatic trigger: 60s from landing")
                log_writer.writerow(imu_data.formatted_for_log())
                break

        # 3-minute deadman timer to ensure a proper shutdown.
        if launch_time != None:
            if datetime.datetime.now() > launch_time + datetime.timedelta(minutes=3):
                # Begin shutdown procedure
                imu_data.add_event("FTS automatic trigger: 3m from launch")
                log_writer.writerow(imu_data.formatted_for_log())
                break

        # Log current system state to file
        log_writer.writerow(imu_data.formatted_for_log())

        # REMOVE FOR FLIGHT!
        events = imu_data.get_events_list()
        if not events[1] == []:
            print("Events produced @ t=" + str(events[0]))
            for event in events[1]:
                print(event)

        time.sleep(1.0 if current_flight_state == FlightState.DISARMED else LOOP_PERIOD)

except KeyboardInterrupt:
    imu_data = IMUData(current_flight_state)
    imu_data.add_event("manually terminated by SIGTERM")
    log_writer.writerow(imu_data.formatted_for_log())

cam.stop_recording()
esc_pwm.ChangeDutyCycle(ESC_MIN_DUTY)
time.sleep(1)
esc_pwm.stop()
io.cleanup()


print("Shutting down...")
# ENABLE FOR FLIGHT:
# subprocess.call(["sudo", "shutdown", "-h", "now"])
