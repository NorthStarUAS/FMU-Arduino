#pragma once

// Given a wind speed estimate, true airspeed estimate, wind direction
// estimate, and a desired course to fly, then compute a true heading to
// fly to achieve the desired course in the given wind.  Also compute the
// estimated ground speed.
void wind_course( float ws_kt, float tas_kt, float wd_deg,
                  float crs_deg, float *hd_deg, float *gs_kt  );
