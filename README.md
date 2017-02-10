# Caution: (still slightly) experimental

This is a port (and overhaul) of my apm2-sensor system which ran on
the old atmega2560 apm2 hardware.  The new system is teensy 3.2 based.
It expects to be connected to an mpu9250 breakout board, ublox8 gps,
and sbus receiver.  Supports an external airdata system via the i2c
bus.

This result is a hobby grade autopilot hardware system that anyone
could put together with basic hand soldering skills (i.e. you can solder
0.1" headers onto a board.)

When paired with a beaglebone, or raspberry pi caliber computer
running the Aura core software, the result is a very high quality and
very capable autopilot system for very low cost.

# Preliminary working features

* MPU9250 via i2c (and probably spi.)
* MPU9250 DMP, interrupt generatation, scaling
* Gyro zeroing (calibration) automatically on startup if unit is still enough.
* SBUS input (direct) with support for 16 channels.
* UBLOX8 support
* Eeprom support for saving/loading config as well as assigning a serial #.
* PWM output support
* Onboard 3-axis stability (simple dampening) system
* "Smart reciever" capability.  Handles major mixing and modes on the
  airplane side allowing flight with a 'dumb' radio.
* i2c airdata system (BFS)
* Full 2-way serial communication with any host computer enabling a 
  "big processor/little processor" architecture.  Hard real time tasks run on
  the "little" processor.  High level functions run on the big processor (like
  EKF, PID's, navigation, logging, communication, mission, etc.)

# Pending

* finalize end-to-end testing.
* flight test

  
