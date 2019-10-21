# RPi-Flight-Computer

## Repository Structure

- `BerryIMU/*`: Example and library code for the BerryIMU from Ozzymaker.
- `old_software/*`: Software from initial September 5, 2019 dump of the `/home/pi` directory.
- `IMU.py`, `LSM9DS0.py`, `LSM9DS1.py`: IMU library.
- `BMP280.py`: Barometer library.
- `master_startup.py`: Script to launch at RPi startup.
- `mean_filter.py`: Time-averaged (*rolling average*) IMU filter.
- `picamera_demo.py`: Demo for recording video from the Pi's Camera Module.
- `pwm_demo.py`: Demo for interacting with PWM ESCs over GPIO.
- `RPi-Flight-Controller.code-workspace`: VSCode workspace file.

## Principal control flow

- Embedded RPi Python script functions
  - Flight controller (*PWM Output*)
  - [Flight state data logger](http://ozzmaker.com/product/berryimu-accelerometer-gyroscope-magnetometer-barometricaltitude-sensor)
    - Important flight events: e.g. Launch detection, arming of different systems, control loop decisions, deployments (*detected via accelerometer edges?*)
  - Camera recorder
  - Landing buzzer control?
  - Automatic safe shutdown 1 minute after landing detected (*upon impact with the ground*)

- Logger file format
  - CSV text lines: flight state with additional column for events
  - TODO: Look into graphing tools to visualize flight data (*maybe [plot.ly Python `Dash` library](https://dash.plot.ly/)?*)

## Background on PWM control for Servo Motors and ESCs

RC PWM has a "window" period of 20ms (milliseconds), with a pulse ranging in width from 1ms to 2ms, where 1ms is ~0% command and 2ms is ~100% command. Duty cycle, a percentage, is a ratio of on-time to off-time.

![ESC PWM Diagram](https://upload.wikimedia.org/wikipedia/commons/b/b7/Sinais_controle_servomotor.JPG)

Therefore:

- 0% throttle command --> 5% duty cycle
- 100% throttle command --> 10% duty cycle

[Documentation from RPi.GPIO library](https://sourceforge.net/p/raspberry-gpio-python/wiki/PWM)

### Rocket IMU Axes

From perspective of a cockpit at the nose cone:

![Originally defined like a fighter plane due to how early spacecraft were flight cockpits plopped on the top of rocket boosters.](https://qph.fs.quoracdn.net/main-qimg-67b906f1ec6e62819e16134e76b8830f-c)

| Vehicle Axis: | Axis Description: | IMU Measurement Axis: |
|--------------:|-------------------|:----------------------|
| X | *roll - vertical axis through center of rocket* | +X (*acc*), +X (*gyro*) |
| Y | *pitch - horizontal axis* | -Y (*acc*), +Y (*gyro*) |
| Z | *yaw - horizontal axis* | -Z (*acc*), +Z (*gyro*) |

## Helpful Resources

- ESC
  - [ESC Specs](https://hobbyking.com/en_us/turnigy-monster-2000-200a-4-12s-brushless-esc.html)
    - [ESC manual](https://cdn-global-hk.hobbyking.com/media/file/969150300X462171X21.pdf)
    - [ESC programming card](https://hobbyking.com/en_us/turnigy-monster-2000-esc-programming-card.html)

- BerryIMU
  - ~~[C++ resources](http://ozzmaker.com/product/berryimu-accelerometer-gyroscope-magnetometer-barometricaltitude-sensor/#Guides%20and%20Tutorials)~~
  - [Kalman Filter](http://ozzmaker.com/guide-interfacing-gyro-accelerometer-raspberry-pi-kalman-filter)
  - [Implementing Positioning Algorithms Using Accelerometers](https://www.nxp.com/docs/en/application-note/AN3397.pdf)
  - [Interpreting raw values](http://ozzmaker.com/accelerometer-to-g)
- ~~[RaspiCam C++](https://www.uco.es/investiga/grupos/ava/node/40)~~
- ~~[WiringPi C++](https://www.youtube.com/watch?v=J6KsTz6hjfU)~~
