# Flight-Computer

## Repository Structure

- `BerryIMU/*`: Example and library code for the BerryIMU from Ozzymaker.
- `Old Software/*`: Software from initial September 5, 2019 dump of the `/home/pi` directory.

## Principal control flow

- Embedded RPi Python script functions
  - Flight Controller (*PWM Output*)
  - IMU & Event data logger
    - [3-axis accelerometer, gyroscope, magnetometer](http://ozzmaker.com/product/berryimu-accelerometer-gyroscope-magnetometer-barometricaltitude-sensor) recording (*200hz?, filtered through Kalman filter*)
    - Important flight events: e.g. Launch detection, arming of different systems, control loop decisions, deployments (*detected via accelerometer edges?*)
    - Flight duration
  - Camera recorder & lighting control
  - Landing buzzer control?
  - Automatic safe shutdown after landing detected
    - 1 minute of zero acceleration after flight?

- Logger file format
  - ASCII text lines: `msgType`s (`IMU`, `Filtered`, `Event`)
  - Separate Python script to parse out IMU or Filtered IMU data for processing (*maybe to a `csv` file?*)
  - TODO: Look into graphing tools to visualize flight data (*maybe [plot.ly Python `Dash` library](https://dash.plot.ly/)?*)

## Helpful Resources

- ESC
  - [RPi GPIO PWM](https://www.electronicshub.org/raspberry-pi-servo-motor-interface-tutorial)
  - [ESC Specs](https://hobbyking.com/en_us/turnigy-monster-2000-200a-4-12s-brushless-esc.html)
    - [ESC manual](https://cdn-global-hk.hobbyking.com/media/file/969150300X462171X21.pdf)
    - [ESC programming card](https://hobbyking.com/en_us/turnigy-monster-2000-esc-programming-card.html)
- [Rocket coordinate system reference](https://qph.fs.quoracdn.net/main-qimg-67b906f1ec6e62819e16134e76b8830f-c)

- BerryIMU
  - ~~[C++ API resources](http://ozzmaker.com/product/berryimu-accelerometer-gyroscope-magnetometer-barometricaltitude-sensor/#Guides%20and%20Tutorials)~~
  - [Kalman Filter](http://ozzmaker.com/guide-interfacing-gyro-accelerometer-raspberry-pi-kalman-filter)
  - [Interpreting values](http://ozzmaker.com/accelerometer-to-g)
- ~~[RaspiCam C++ API](https://www.uco.es/investiga/grupos/ava/node/40)~~
- ~~[WiringPi C++ API](https://www.youtube.com/watch?v=J6KsTz6hjfU)~~
