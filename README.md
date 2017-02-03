# Caution: experimental

This is an experiment in porting my apm2-sensor system which ran on the old
atmega2560 apm2 hardware.  The target system will be a teensy 3.2 connected
to an mpu9250 breakout board.  Hopefully there will be an onboard static 
pressure sensor, sbus input support, pwm servo out support, and maybe gps
(ublox 8+) support.

The goal is for a hobby grade system that anyone could put together with 
meager soldering skills (i.e. you can solder 0.1" headers onto a board.)

# Preliminary working features

* MPU9250 via i2c (and probably spi.)
* MPU9250 DMP, interrupt generatation, scaling
* Gyro zeroing (calibration) automatically on startup if unit is still enough.
* SBUS input (direct) with support for 16 channels.
* UBLOX8 support
* Eeprom support for saving/loading config as well as assigning a serial #.

# Pending

* PWM output support
* Onboard roll, pitch, yaw damening direct with gyros. (Simple stabilization.)
* "Smart reciever" features which can move all the mixing and modes to the
  airplane side allowing flight with a 'dumb' radio.
* Full 2-way serial communication with any host computer enabling a 
  "big processor/little processor" architecture.  Hard real time tasks run on
  the "little" processor.  High level functions run on the big processor (like
  EKF, PID's, navigation, logging, communication, mission, etc.)
  
