# Flight-Computer

## Repository Structure

- `BerryIMU/*`: Example and library code for the BerryIMU from Ozzymaker.
- `Old Software/*`: Software from initial September 5, 2019 dump of the `/home/pi` directory.

## Principal control flow

- Embedded RPi Python script functions
  - Flight Controller (*PWM Output*)
  - IMU & Event data logger
    - [IMU](http://ozzmaker.com/product/berryimu-accelerometer-gyroscope-magnetometer-barometricaltitude-sensor) data recording (*filtered through Kalman filter*)
    - Important flight events: e.g. Launch detection, arming of different systems, control loop decisions, deployments (*detected via accelerometer edges?*)
    - Flight duration
  - Camera recorder
  - Landing buzzer control?
  - Automatic safe shutdown after landing detected
    - 1 minute of zero acceleration after flight?

- Logger file format
  - CSV text lines: flight state with additional column for events
  - TODO: Look into graphing tools to visualize flight data (*maybe [plot.ly Python `Dash` library](https://dash.plot.ly/)?*)

## Background on PWM control for Servo Motors and ESCs

RC PWM has a "window" period of 20ms (milliseconds), with a pulse ranging in width from 1ms to 2ms, where 1ms is ~0% command and 2ms is ~100% command. Duty cycle, a percentage, is a ratio of on-time to off-time.

![ESC PWM Diagram](https://upload.wikimedia.org/wikipedia/commons/b/b7/Sinais_controle_servomotor.JPG)

### Rocket IMU Axes

From perspective of a cockpit at the nose cone:

![Originally defined like a fighter plane due to how early spacecraft were flight cockpits plopped on the top of rocket boosters.](https://qph.fs.quoracdn.net/main-qimg-67b906f1ec6e62819e16134e76b8830f-c)

| Vehicle Axis: | Axis Description: | IMU Measurement Axis: |
|--------------:|-------------------|:----------------------|
| X | *roll - vertical axis through center of rocket* | +Y (*acc*), -Y (*gyro*) |
| Y | *pitch - horizontal axis* | +X (*acc*), +X (*gyro*) |
| Z | *yaw - horizontal axis* | -Z (*acc*), +Z (*gyro*) |

[Documentation from RPi.GPIO library](https://sourceforge.net/p/raspberry-gpio-python/wiki/PWM)

Therefore:
- 0% throttle command --> 5% duty cycle
- 100% throttle command --> 10% duty cycle

## Helpful Resources

- ESC
  - [RPi GPIO PWM](https://www.electronicshub.org/raspberry-pi-servo-motor-interface-tutorial)
  - [ESC Specs](https://hobbyking.com/en_us/turnigy-monster-2000-200a-4-12s-brushless-esc.html)
    - [ESC manual](https://cdn-global-hk.hobbyking.com/media/file/969150300X462171X21.pdf)
    - [ESC programming card](https://hobbyking.com/en_us/turnigy-monster-2000-esc-programming-card.html)

- BerryIMU
  - ~~[C++ API resources](http://ozzmaker.com/product/berryimu-accelerometer-gyroscope-magnetometer-barometricaltitude-sensor/#Guides%20and%20Tutorials)~~
  - [Kalman Filter](http://ozzmaker.com/guide-interfacing-gyro-accelerometer-raspberry-pi-kalman-filter)
  - [Interpreting values](http://ozzmaker.com/accelerometer-to-g)
- ~~[RaspiCam C++ API](https://www.uco.es/investiga/grupos/ava/node/40)~~
- ~~[WiringPi C++ API](https://www.youtube.com/watch?v=J6KsTz6hjfU)~~
