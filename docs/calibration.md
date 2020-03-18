# IMU calibration

* calibrated sensors produce far better ekf results than uncalibrated
  sensors and also leads to quicker convergence, so calibration is an
  important consideration.

* calibration can get complicated so this is my attempt to think
  through and document the strategy used.

## Two levels of calibration

Level 1 orientation correction is applied first, and the 2nd level is
applied on top of that.

1. Correction for basic sensor alignment/mounting errors.  Correction
is represented as a rotation matrix R, which should be a pure rotation
matix R*R(transpose) = Identity.

2. After the alignment correction, the second level calibration is
applied.  Gyros are zero'd at the start, acclerometers have a bias
correction based on a temperature fit model.  Magnetometers have a
full affine matrix correction.

## Data messages

The intent is for the alignment correction calibration to be
'ubiquitous', quiet, and hidden.  That is setup once and then
shouldn't need to be changed.

* "raw" sensor values refer to post-alignment correction, but
  pre-second level correction.  The pure original, raw, completely
  untouched, values from the sensors are never saved or used.

  "raw" sensor values can be logged and used to improve future
  calibration fits.

* "corrected" sensor values are post calibration, and are the values
  fed into the EKF or used for other operations downstream.

  "corrected" sensor values can later be replayed through an offline
  EKF and should produce results similar to the real time results.