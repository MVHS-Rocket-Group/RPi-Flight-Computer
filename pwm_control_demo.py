import time
import sys

# RC PWM has a "window" period of 20ms (milliseconds), with a pulse ranging in
# width from 1ms to 2ms, where 1ms is ~0% command and 2ms is ~100% command. Duty
# cycle, a percentage, is a ratio of on-time to off-time.
#
# Cit: https://upload.wikimedia.org/wikipedia/commons/b/b7/Sinais_controle_servomotor.JPG
# Cit: https://sourceforge.net/p/raspberry-gpio-python/wiki/PWM/
#
# Therefore:
# - 0% command --> 5% duty cycle
# - 100% command --> 10% duty cycle

esc_pwm_pin = 15
# frequency (Hz) = 1 / period (sec)
esc_pwm_freq = 1 / 0.02
# Duty cycle percentage when firewalling the throttle.
esc_max_duty = 100 * 2.0 / 20.0
# Duty cycle percentage when idling.
esc_min_duty = 100 * 1.0 / 20.0
# Handle for control over the ESC PWM pin. Setup later in `try` block.
esc_pwm = None

try:
    try:
        import RPi.GPIO as gpio
        gpio.setwarnings(False)
        gpio.setmode(gpio.BOARD)
        gpio.setup(esc_pwm_pin, gpio.OUT)

        # Setup the PWM pin and set it to min command.
        esc_pwm = gpio.PWM(esc_pwm_pin, esc_pwm_freq)
        esc_pwm.start(esc_min_duty)
    except RuntimeError:
        print("Platform not supported, must be Raspberry Pi")
        sys.exit()

    # while True:
    # Set the throttle to full power for 5s.
    print("Setting maximum power")
    esc_pwm.ChangeDutyCycle(esc_max_duty)
    for count in range(5):
        time.sleep(1)
        print(str(count) + "...")

    print("Setting minimum power")
    esc_pwm.ChangeDutyCycle(esc_min_duty)

except KeyboardInterrupt:
    pass

esc_pwm.stop()
gpio.cleanup()
