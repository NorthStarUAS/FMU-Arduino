# INS / GNSS - Inertial and Global Navigation Satellite System Kalman filter code

This section of code uses fancy math to blend the high rate inerital sensor data
(gyros, accelerometers, and possibly magnetometers) with low rate gps position
and velocity.  Out of this is produced a high quality true heading (yaw), roll,
and pitch estimate and a fast position and velocity estimate.

There is a lot of theory underlying this code which is fascinating to study, but
the executive summary is: there is no sensor on board that can directly sense
the aircraft's attitude (roll, pitch, and yaw) while dynamically moving.
Magnetometers can help, but they are terribly innacurate, easily biased by your
own aircraft, and are difficult to precisely calibration.

## Intuitive Explanation

A Kalman filter can estimate the true attitude of your aircraft because over
time when moving there is only one true attitude that brings all the sensor
measurements and position velocity integrations into a consensus. (Dhange in
velocity: change in any the 3 velocity vector components that is significant
with respect to the gps's ability to measure it.)

Imagine we have an IMU that reports inertial measurements at 100hz and we have a
gps that reports position and velocity at 1hz.  We can start out at some
arbitrary guess for our orientation.  We can measure the gps position and
velocity.  Now over the next second when we are recieving IMU data we can use
the gyro rates to update our orientation guess, and use the acceleromter to
update our velocity, and the new velocity to update our position.  We call this
forward propagation (or integration).  Now when the next gps reading comes in,
we can compare our estimated position and velocity with the gps's actual
position and velocity.  If our initial guess is somewhat wrong, our etimated
position will be different from the actual one the gps reports.  We can use the
size of this error to correct our attitude estimate (for example: can we find a
different initial guess that reduces the forward propagation error?)  Repeat
this process over time, and do it with fancy kalman filter math, and you have
the code here.

## 15 States

The code here is a 15-state Kalman filter.  This means the filter is estimating
15 different states:

* Latitude, Longitude, and Altitude
* North, East, and Down Velocity
* Roll, Pitch, and Yaw (Attitude)
* Gyro Biases for Each of the Three Gyros
* Accelerometer Biases for Each of the Three Accelerometers

That adds up to 15 states.  It is important to estimate the gyro and
accelerometer biases because these are never perfect, the error can drift during
the flight.  Correcting for this bias enables the forward propagation to be far
more accurate.

## Magnetometers

There is a version of the 15-state Kalman filter that incorporates magnetometer
sensors.  We haven't added any new states, instead the magnetometer values are
used to help during the correction step (at the same time a new gps reading
comes in and we check how much forward propagation error we have.)  The
advantage of magnetometers is they do give us an absolute orientation reference
with respect to the earth.  The problem is it's a really noisy/bad sensor that
is easily biased by electrical fields (i.e. your quad copter motors,
electronics, the aiframe itself if it contains metal, local magnetic variations
in the world, etc.  Look at your magnetometer accuracty if you are flying from a
large iron ship for example!)

If the mag version of the EKF is used, it is important to assign a larger error
(small weighting) to the mags so that when there isn't much movement information
the mags can help constrain the solution from drifting endlessly, but when there
is sufficient change in velocity, that information will be weighted more
heavily.

What you will typically see is a startup solution that is ok, and quick
convergence once airborne.  Then upon landing the solution will slowly drift
away from truth towards a match to the magnetometer calibration (which will
never be as good as you hope.)

## Calibration

The perfomance of the system will greatly improve relative to how well your
sensors are calibrated.  This project has a built in accelerometer calibration
process which also estimates the strapdown (airframe mounting) orientation
error.  There is also code that can passively collect and log the EKF's true
attitude and idealized mag vector estimate vs the magnetometer's actual sensor
readings.  After the flight this data can be processed to create an ultra high
quality magnetometer calibration matrix (far better than the crude calibration
you get from doing the DJI/pixhawk dance.)  This code can also keep track of the
accelerometer bias vs. IMU temperature.  Over time (with some care and effort)
you can use your actual flight data to build up a highly accurate magnetomter
and temperature calibration fit.

## There is so much more

The EKF and Kalman filter theory (and practical details) are one of the coolest
aspects of this entire project!  If you put some care and effort into
calibration, you can literally build an advanced INS/GNSS system that rivals the
performance of commercial units costing several thousands of $$$ with < $100
worth of parts.

## Credits

This algorithm was developed by Demoz Gebre, a professor at the U of MN
aerospace engineering department during his phd work at Standford University. It
was ported from matlab to the C langauge by a grad student at the U of MN and
adapted for real-time use by myself (Curt).  The code was further ported to C++
in order to use the Eigen matrix library.  Additionally, there have been small
peformance and configuration tweaks over the years.  The code has always been
mathematically and logically correct, so over the years there have been small
refinements, but the general outline and flow has remained largely unchanged. We
have enjoyed 100's of real world autonomous flight hours (and thousands of hours
of flight simulation time) running this code and have a high degree of trust in
it's robustness and correctness.