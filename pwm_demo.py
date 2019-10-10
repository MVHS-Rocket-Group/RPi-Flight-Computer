import time
import sys
import RPi.GPIO as io

ESC_PWM_PIN = 15
# frequency (Hz) = 1 / period (sec)
ESC_PWM_FREQ = 1 / 0.02
# Duty cycle percentage when firewalling the throttle.
ESC_MAX_DUTY = 100 * 2.0 / 20.0
# Duty cycle percentage when idling.
ESC_MIN_DUTY = 100 * 1.0 / 20.0
# Handle for control over the ESC PWM pin. Setup later in `try` block.
esc_pwm = None

try:
    io.setwarnings(False)
    io.setmode(io.BOARD)
    io.setup(ESC_PWM_PIN, io.OUT)

    # Setup the PWM pin and set it to min command.
    esc_pwm = io.PWM(ESC_PWM_PIN, ESC_PWM_FREQ)
    esc_pwm.start(ESC_MIN_DUTY)

    print("Setting quarter power")
    esc_pwm.ChangeDutyCycle(
        0.25 * (ESC_MAX_DUTY - ESC_MIN_DUTY) + ESC_MIN_DUTY)
    for count in range(5):
        time.sleep(1)
        print(str(count) + "...")

    print("Setting minimum power")
    esc_pwm.ChangeDutyCycle(ESC_MIN_DUTY)

    time.sleep(2)

except KeyboardInterrupt:
    pass

esc_pwm.stop()
io.cleanup()
