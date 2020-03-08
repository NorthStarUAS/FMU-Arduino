# Aura Sensors

This is a teensy 3.x/4.x (teensyduino) sketch.  Aura-sensors turns an
inexpensive teensy board into a sensor collector + attitude
determination system + communications hub + servo controller.  It is
not yet a full fledged autopilot itself, but designed to pair with a
host linux board (such as a raspberry pi, beaglebone, gumstix, etc)
for all the higher level AP functions.  It supports the mpu9250 imu,
ublox8 gps, bme280/bmp280 pressure sensors, sbus receiver, and
attopilot volt/amp sensor.  Supports an external airdata systems via
the i2c bus.

As of version 4xx, a high accurancy 15-state EKF has been added for
precision attitude and lcoation estimate. It is currently inertial
only (no magenetometer support) and designed to work exceptionally
well for outdoor dynamic systems such as fixed wing aircraft.

This is one component of a research grade autopilot system that anyone
can assemble with basic soldering skills.

When paired with a beaglebone, or raspberry pi linux computer running
the Aura-core AP software, the result is a very high quality and very
capable autopilot system at a very inexpensive price point.

# Features/support

* MPU9250 via spi or i2c.
* MPU9250 DMP, interrupt generatation, scaling
* Gyro zeroing (calibration) automatically on startup if unit is still enough.
* UBLOX8 support
* 15-State EKF (kalman filter)
* SBUS input (direct) with support for 16 channels.
* BME280/BMP280 support
* Eeprom support for saving/loading config as well as assigning a serial #.
* 8 channel PWM output support
* Onboard 3-axis stability (simple dampening) system
* "Smart reciever" capability.  Handles major mixing and modes on the
  airplane side allowing flight with a 'dumb' radio.
* i2c airdata system (BFS, mRobotics, 3dr)
* Full 2-way serial communication with any host computer enabling a 
  "big processor/little processor" architecture.  Hard real time tasks run on
  the "little" processor.  High level functions run on the big processor (like
  EKF, PID's, navigation, logging, communication, mission, etc.)

# Flight Testing

* This system originated in the late 2000's and has been evolving as
  DIY haredware has improved.  The APM2 version of the firmware flew
  at least as early as 2012.  There was an APM1 version prior to that
  and a Xbow MNAV version even earlier.  The PJRC-teensy version of
  this system has been flying since February 2018.

  
