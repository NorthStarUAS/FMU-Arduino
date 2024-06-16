# NorthStar FMU (for Teensy/Arduino)

This is an Arduino (Teensyduino) sketch for building the heart of a UAV
autopilot.  The fmu-arduino sketch is intended to create a full fledged
autopilot from an inexpensive teensy board and a collection of inexpensive
sensors from the DIY autopilot world.  It can also pair with a host linux
computer (like a beaglebone or raspberry pi) for additional high level
functionality.  It supports the mpu9250 imu, ublox8 gps, bme280/bmp280 pressure
sensors, sbus receiver, and attopilot volt/amp sensor. It also supports an
external airdata systems via the i2c bus.

Please note: this project is not intended to support every sensor, every
processor board, every vehicle type, and every use case.  Please see ardupilot
and px4 for two wonderful projects that take the big-tent approach.  The intent
of this project is to keep things as simple as possible.  (And I know "simple"
is a relative measure.)

![prototype](images/IMG_20191118_064616925.jpg "Prototype board")

As of version 4, a high accurancy 15-state EKF has been added for
precision attitude and lcoation estimate. It is designed to work
exceptionally well for outdoor dynamic systems such as fixed wing
aircraft.

fmu-arduino is one component of a research grade autopilot system that anyone
can assemble with basic soldering skills.  Altogether, the NorthStar UAS
ecosystem provides a high quality autopilot system that ephasizes high
reliability and simple code design.  It offers many advanced capabilities that
anyone can study and include in their own projects.

## Features

* MPU9250 via spi or i2c.
* MPU9250 DMP, interrupt generatation, scaling
* Gyro zeroing (calibration) automatically on startup if unit is still enough.
* UBLOX8 support
* 15-State EKF (kalman filter), 2 variants: (1) ins/gns, (2) ins/gns/mag
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

## Installation

* vscode arduino extension, bundled cli
* arduino extension manager -> eigen boulder flight eigen 3.0.2
* Download the zip file: <https://github.com/bolderflight/eigen/archive/refs/heads/main.zip>

## Flight Testing

* This system originated in the late 2000's and has been evolving as
  DIY haredware has improved.  The APM2 version of the firmware flew
  at least as early as 2012.  There was an APM1 version prior to that
  and a Xbow MNAV version even earlier.  The PJRC-teensy version of
  this system has been flying since February 2018.

## What is new and upcoming in 2024?

* I have puzzled through the changes needed to use the extra onboard program
  memory flash as file storage.  Config files and data logs can be stored here.
* To make the onboard storage more useful, I have also puzzled through the code
  to add MTP functionality.  This means we can plug the teensy into a host
  computer via usb and the onboard flash (and SD card if plugged in) will show
  up as disks on the host computer and you can copy/edit files back and forth.
* Things still on my shortlist todo:
  * HIL support: build a usb serial interface to a JSBSim simulation running in
    real time on a host computer.  This will enable desktop testing and
    validation of the onboard flight control laws.  And (someday) the higher
    level mission management code.
  * Move the flight critical main core loop functions to an interrupt driven
    routine to run at a hard real-time update rate.
  * Migrate data logging functions to the teensy (to run in the slack time.)
* Things on the longer term wish list:
  * High level guidance functions (such as auto launch and land, survey route
    planning, ...) remain on the host computer for now.
  * I have experimented with a simple simulator that is light weight enough to
    run onboard the teensy.  The state transition is computed as a matrix and
    derived directly from flight data logs.
    * I have worked out a method to derive model-based flight control laws
      directly from the flight data logs.
    * This could be used for HIL testing without needing a host computer to run
      the simulation.
    * This could also be used to predict the next state of the system and compare
      to the actual measured next state and flag performance anomalies for early
      fault detection.
    * The simulator can be generated (fit) from actual flight test data using a
      process I jokingly call Aero-DMD (because the math setup resembles the setup
      for dynamic mode decomposition from the fluids field.)  I'm told I'm just
      doing least squares and to not over-hype it. :-)
  * Do a respin of the DIY hardware board that presumably drops the host
    computer requirement and has /everything/ running on the teensy in real
    time.

## What is new and upcoming in 2023?

* I am bringing more of the high level (linux) functionality down to the teensy
  level.  This requires a teensy with an SD card (ex: teensy 3.6 or 4.1)
* The autopilot and aircraft config is now stored in a set of configuration json
  files on the SD card.
* I am planning to port data logging functionality over to the teensy (was on
  the beaglebone.)
* The telemetry iterface is now handled directly on the teensy (was on the
  beaglebone.)
* Innerloop autopilot functionality and controllers are now on the teensy (was
  on the beaglebone.)
* Intermediate guidance functions are in progress (circle hold and route
  following).  The port compiles but it needs testing.
* High level guidance functions (such as auto launch and land, survey route
  planning, ...) remain on the host computer for now.
* I have experimented with a simple simulator that is light weight enough to run
  onboard the teensy.
  * This could be used for HIL testing.
  * This could also be used to predict the next state of the system and compare
    to the actual measured next state and flag performance anomalies for early
    fault detection.
  * The simulator can be generated (fit) from actual flight test data using a
    process I jokingly call Aero-DMD (because the math setup resembles the setup
    for dynamic mode decomposition from the fluids field.)  I'm told I'm just
    doing least squares and to not over-hype it. :-)
* On-board accelerometer (strapdown) calibration on the teensy.
* Written in May, 2023 ...
  * continue porting the my ardupilot-based version of the flight controller to
    arduino/teensy and continue testing and validating each major module or
    feature as I go.
  * as the port stabilizes, I need to put together some updated hardware so I
    can flight test (probably in my venerable skywalker platform.)
  * then flight test and refine as I go ...

## What's new in 2020?

* 2020 brought us the Teensy 4.0 (yeah!) with crazy fast CPU speeds
  and more memory.  This allows us to imagine doing more of the flight
  critical work right on the 'little' processor, leaving the 'big'
  processor for the tasks that can run at slightly slower rates.

* I have pushed through quite a few code architecture and
  simplification changes.  The goal is always to make the structure
  lighter weight when possible.

* I have added a powerful matrix based inceptor->effector mixing
  system.  The mixes can be setup logically by function, or by
  defining the matrix directly.

* I have added support for an on-board strapdown error calibration
  matrix, on-board accelerometer calibration and onboard magenetometer
  calibration.

* I have added two variants of the UMN AEM ins/gns kalman filter.

  1. A 15-state ins/gps only filter (gyros, accels, gps) that performs
     extremely well for fixed wing aircraft.

  2. A 15-state ins/gps/mag filter that supports low dynamic vehicles
     such as quad copters and rovers.  As with any magnetomter based
     attitude determination system, it is critical to have well
     calibrated mags for good performance.  I have an offline "self"
     calibration system that I am considering adapting for
     onboard/automatic calibration.

* A subset of the covariances are reported to the host: (1) the
  maximum of the 3 position errors, (2) the max of the 3 velocity
  errors, (3) the max of the 3 attitude errors.  These are statistical
  estimates, but can be useful for monitoring the health of the ekf
  solution.
