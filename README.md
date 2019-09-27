# Flight-Computer

## Repository Structure

- `Old Software`: Software from initial September 5, 2019 dump of the `/home/pi` directory.

## Helpful Resources

- [RPi PWM Servo control](https://www.electronicshub.org/raspberry-pi-servo-motor-interface-tutorial/)
- [ESC Specs page](https://hobbyking.com/en_us/turnigy-monster-2000-200a-4-12s-brushless-esc.html)
  - [ESC manual](https://cdn-global-hk.hobbyking.com/media/file/969150300X462171X21.pdf)
  - [Optional programming card](https://hobbyking.com/en_us/turnigy-monster-2000-esc-programming-card.html)

- ~~[BerryIMU C++ resources](http://ozzmaker.com/product/berryimu-accelerometer-gyroscope-magnetometer-barometricaltitude-sensor/#Guides%20and%20Tutorials)~~
- ~~[RaspiCam C++ resources](https://www.uco.es/investiga/grupos/ava/node/40)~~
- ~~[WiringPi C++ resource](https://www.youtube.com/watch?v=J6KsTz6hjfU)~~

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
