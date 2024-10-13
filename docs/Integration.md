# Hardware Integration / Software Installation Notes

Just a quick note on philosophy and purpose.  My greatest hope for this project
is that it can be a tool to help others learn about sensors, flight computers,
flight control laws, and a host of supporting material.  I have made an effort
to keep the code simple-ish, favoring clarity over performance in many cases. If
you just want to plug things together and go fly, this project may end up a
little daunting and frustrating.  I am willing to help and answer questions as
much as my bandwidth allows.

These notes will be woefully incomplete.  They are not a step by step install
guide.  But I will try to hit some important points.  For the first iteration I
am doing the build around a Bolder Flight Systems (BFS) Marmot board.  This
embeds a teensy 3.6 which has SD card logging, but it has significantly lower
performance compared to a teensy 4.1.  100hz update rates are maginal (just
doable) on the teensy 3.6, but 50hz rates are solid.

Expect the process to be a little bit more complicated than building your own
gaming PC, but a little less complicated than rocket surgery.

## Power up

The teensy should be setup with a pad on the back cut so that it does not take
power over usb.  You don't want to power the teensy externally and
simultaneously power it via usb.  This may appear to work initially but leads to
bad things over time.  Thus you should expect to externally power (5v) the
teensy. If you are just experimenting with a standalone teensy then sure, power
it up over usb, but for a real integration that will actually fly, you need
external power.  Best advice is to know what you are trying to do with power and
think and don't cut corners or depend on hope or assumptions!

## Compile and flash the firmware

* Compile with the stock arduino ide (with teensyduino addon)
* Optimize for smallest code. (otherwise the compiled code image is too big to
  fit in the memory space!)
* CPU Speed overclocking seems to work (but you accept any risks!)  240hz to
  match the ESP32?
* USB Type: Serial + MTP (Experimental)

## Copy config files to MTP disk

Properly setting up config files is a whole thing.  The best I can say for now
is start with something similar and edit from there.  It maybe isn't helpful to
suggest using the source, luke, but that can also be helpful.  The source is
there to be studied and be illustrative.

After the teensy firmware is compiled and flashed, it will appear on the host
computer as a thumb drive.

* Teensy 3.6 has no prog flash, so copy config files to the top level sd card
  via MTP connection (or sneaker net if you must.)
* Teensy 4.1 has prog flash, so copy config files to prog flash.
* Config file edits need to happen on the host computer and then the new version
  needs to be copied over.  MTP doesn't support editing files in place, only
  copy & delete.

## Validate sensors

Are you getting nothing at all from the sensors, not even IMU? ... check that
HIL mode hasn't been configured (expecting sensor data relayed from sim running
on host pc.)

There is an interactive (menu) mode when you first bootup the firmware that
enables viewing a variety of sensor data, calibrate the strapdown matrix, and
dump the entire property tree for inspection.

It is important to carefully work through validation of all the sensors (imu,
gps, sbus, airdata) first.  Then validate the EKF is working.  And finally
validate you are getting reasonable values written out to the servos (effectors
... your mixing matrix is configured reasonably.)

## Calibrate main battery source voltage

* This is important for accurate battery usage reporting in flight.
* /config/sensors/power/battery_calibration (in sensors.json)
* Set as a multiplier (for example 1.x) to compute the true voltage.
* You will want to accurately measure the voltage with a volt meter, compute the
  multiplyer needed, edit the sensors.json file for your aircraft, and then copy
  it to the FMU.

## Calibrate strapdown matrix (and accelerometers)

The IMU is mounted to a board which is then mounted in the aircraft (often on
another board or shelf.)  There is no way to guarantee everything is perfectly
orthogonoal (aka straight) so calibration is important.  This calibration should
be performed after the hardware is integrated into the aircraft and with the
aircraft fully assembled.  (Sorry if you are installing this in a real 747,
calibration is going to be an issue!)

The console interactive menu has a calibration option.  Choose that and follow
the on-screen prompts.  Endeavor to position the aircraft as level or vertical
as possible when instructed to do so, and hold it steady as the measurements are
being collected.  The more care you employ here, the better your result will be.

The result will be a strapdown calibration matrix that maps whatever actual
orientation of your board to the aircraft orientation.  It doesn't matter if you
board is mounted backwards, upside down, or at a 45 degree angle.  The strapdown
calibration matrix maps the IMU sensors to the aircraft coordinate system.

* Calibration of magnetometers: this is a topic for future discussion.  I have
  an external script that can process flight data and derive an more accurate
  magnetometer calibration matrix compared to the typical dji dance style mag
  calibration.  But it is extra effort to extract and post process the flight
  data and copy the calibration back to the aircraft.  The results are great
  though for the extra effort.

* Calibrate external (main input) voltage sensor: TBD.

## Setup RC Transmitter/Receiver failsafe (typically on transmitter)

* Set receiver failsafe (from transmitter) to "hold".

  Without this your controls could go hard over or do unexpected things if you
  lose transmitter connection in flight.