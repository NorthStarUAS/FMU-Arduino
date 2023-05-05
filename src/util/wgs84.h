#pragma once

// given, lat1, lon1, az1 and distance (s), calculate lat2, lon2
// and az2.  Lat, lon, and azimuth are in degrees.  distance in meters
int geo_direct_wgs_84 ( double lat1, double lon1, double az1,
                        double s, double *lat2, double *lon2,
                        double *az2 );

// given lat1, lon1, lat2, lon2, calculate starting and ending
// az1, az2 and distance (s).  Lat, lon, and azimuth are in degrees.
// distance in meters
int geo_inverse_wgs_84( double lat1, double lon1, double lat2,
                        double lon2, double *az1, double *az2,
                        double *s );
