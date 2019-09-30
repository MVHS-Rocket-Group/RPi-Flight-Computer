import time
import csv
import os
import smbus
import enum
import math
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

# Loop period (sec)
loop_period = 1.0 / 10.0


class FlightState(enum):
    # Python Enum type: https://www.geeksforgeeks.org/enum-in-python
    ON_PAD = 1
    LAUNCHED = 2
    LANDED = 3


class IMUData:
    time = None
    acc = [None, None, None]
    gyro = [None, None, None]
    mag = [None, None, None]
    # baroPressure?
    # baroTemp?
    events = []

    def __init__(self, time, acc, gyro, mag):
        self.time = time
        self.acc = acc
        self.gyro = gyro
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
        output = self.time + ", "
        output += ", ".join(self.acc)
        output += ", ".join(self.gyro)
        output += ", ".join(self.mag)
        output += ", ".join(self.events_list)

        return output


# Stores which state of flight the rocket is currently in.
flight_state = FlightState.ON_PAD

try:
    # Init GPIO PWM output for ESC
    io.setwarnings(False)
    io.setmode(io.BOARD)
    io.setup(esc_pwm_pin, io.OUT)

    # Setup the PWM pin and set it to min command.
    esc_pwm = io.PWM(esc_pwm_pin, esc_pwm_freq)
    esc_pwm.start(esc_min_duty)

    # TODO: Start log file
    # TODO: Start camera recording (do we want to do this later?)

    # Main Loop
    while True:
        # TODO: Read in motion data from IMU
        # TODO: Compute and send target to ESC
        # TODO: Did any important events get triggered?
        # TODO: Did the flight state change? e.g. did we just launch, just land?
        # TODO: Has it been >1 munute since landing? If so, shut off the camera recording and shut down RPi
        # TODO: Log current system state to file (IMUData and flight_state)
        time.sleep(loop_period)

except KeyboardInterrupt:
    pass

# TODO: Stop camera recording
esc_pwm.stop()
gpio.cleanup()
