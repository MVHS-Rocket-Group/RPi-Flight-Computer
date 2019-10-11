import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)

GPIO.setup(12,GPIO.OUT)
GPIO.setup(18,GPIO.OUT)

p1 = GPIO.PWM(12,50)
p2 = GPIO.PWM(18,50)
p1.start(7.5)
p2.start(7.5)

try:
	while True:
		p1.ChangeDutyCycle(7.5)
		p2.ChangeDutyCycle(7.5)
		time.sleep(1)
		p1.ChangeDutyCycle(2.5)
		p2.ChangeDutyCycle(2.5)
		time.sleep(1)
		p1.ChangeDutyCycle(12.5)
		p2.ChangeDutyCycle(12.5)
		time.sleep(1)
except KeyboardInterrupt:
	p1.stop()
	p2.stop()
	GPIO.cleanup()
