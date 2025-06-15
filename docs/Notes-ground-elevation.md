# Thoughts and notes on ground elevation

## Sources

* gps
  * can have big jumps as solution changes over time
* barometer (measuring cabin pressure)
  * don't fully trust the pressure -> altimeter formula is accurate across a 400' elevation change?
  * biased by airspeed when the sensor is in the cabin interior
* navigation filter (gps + imu)
  * biased by gps
  * possibly (rare!) can get corrupted if ekf solution barfs
* blended baro + gps = "true"
  * responds to change with same characteristics as pressure alt
  * is corrected to true altitude over time
  * if pressure->alt formula isn't scaling quite right, the correction at the
    current altitude can cause errors/bias when changing to a new altitude:
    especially an issue on landing if we have biased our altitude and miss our
    touchdown spot by 100's of feet.

## Home calibration

* Uses an average of "gps" altitude over "n" seconds
  * Not subject to filter weirdness which can happen before takeoff if we don't
    have good change of velocity to help filter converge.
  * Samples/averages lat, lon, and altitude.

## Which altitude sensor/source to use as our main reference??!?!!??

* drives me nuts, any of these can be better than the others on any given flight
  on any given day.
* I have never found a clear winner.
* For now I want to lean towards the nav filter (possible latency issue when
  descending on landing?)  Gps itself pretty heavily filters (delays) altitude
  and vertical velocity as well (although less so than altittud)
* AGL is an input to "is_airborne" so a gps sensor "jump" due to interference,
  loss of some sats, poor fix, etc. could potentially contribute to an
  inadvertant is_airborne==True scenario when we don't want to think we are
  airborne. (possible throttle up risk)

## Airdata v8 vs v9 vs environment messages

* v8 message had environment as well, Airdata v9 just has airdata, we added a
  new separate env message with ground info.
* thus ground elevation does not imply the source ... we can select that when we
  generate the data.
* internally on the FMU, can track airdata ground elev and airdata agl ... maybe
  someday add that back to the message pending bandwidth?

## When to compute ground elevation?

* We have an idea of doing a filtered average while not airborne to establish
  ground elevation.
* We have an idea of doing a operator-instantiated task to survey the ground
  elevation/landing spot.
* Which do we want?  Implied ground elevation until such time that the operator
  would ask to survey the landing spot?

## Where to compute ground elevation?

* tasks/globals/home_mgr.cpp inits home position / altitude when gps comes alive
  so we have something.
* calib_home updates this with a 30 ("n") second filtered sample.  Is this good enough?
* should we use "procedure" to calibrate home/ground and dispense with the
  running averages when we think we are on the ground?
* how can we force a home calibration before launch is enabled?

## first test:

* somehow motor enable without calibrating home
* careful of potential circular dependency between home_mgr, nav_mgr, ground, and airdata