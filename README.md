# Aura Sensors

This is arduino code written for the teensy (arm) line of embedded
processor boards.  It turns the teensy into a sensor
collector/aggregater, communications hub, and servo controller.  It is
not a full fledged autopilot itself, but designed to pair with a linux
board (pi, gumstix, beaglebone, etc) for all the higher level AP
functions.  It supports the mpu9250 imu, ublox8 gps, bme280/bmp180
pressure sensor, sbus receiver, and attopilot volt/amp sensor.
Supports an external airdata system via the i2c bus.

This is one component of a research grade autopilot system that anyone
can assemble with basic soldering skills.

When paired with a beaglebone, or raspberry pi caliber computer
running the Aura core software, the result is a very high quality and
very capable autopilot system at a very inexpensive price point.

# Preliminary working features

* MPU9250 via spi (i2c is supported in the library.)
* MPU9250 DMP, interrupt generatation, scaling
* Gyro zeroing (calibration) automatically on startup if unit is still enough.
* SBUS input (direct) with support for 16 channels.
* UBLOX8 support
* BME280 support
* Eeprom support for saving/loading config as well as assigning a serial #.
* 8 channel PWM output support
* Onboard 3-axis stability (simple dampening) system
* "Smart reciever" capability.  Handles major mixing and modes on the
  airplane side allowing flight with a 'dumb' radio.
* i2c airdata system (BFS)
* Full 2-way serial communication with any host computer enabling a 
  "big processor/little processor" architecture.  Hard real time tasks run on
  the "little" processor.  High level functions run on the big processor (like
  EKF, PID's, navigation, logging, communication, mission, etc.)

# Pending

* real flight test

  
