#include <math.h>

#include "windtri.h"

static const double d2r = M_PI / 180.0;
static const double r2d = 180.0 / M_PI;
static const double m2pi = M_PI * 2.0;

// Given a wind speed estimate, true airspeed estimate, wind direction
// estimate, and a desired course to fly, then compute a true heading to
// fly to achieve the desired course in the given wind.  Also compute the
// estimated ground speed.

void wind_course( float ws_kt, float tas_kt, float wd_deg,
                  float crs_deg, float *hd_deg, float *gs_kt )
{
    // from williams.best.vwh.net/avform.htm (aviation formulas)
    float wd = wd_deg * d2r;
    float crs = crs_deg * d2r;
    float hd = 0.0;
    *gs_kt = tas_kt;
    *hd_deg = crs_deg;
    if ( tas_kt > 0.1 ) {
	// printf("ws=%.1f tas=%.1f wd=%.1f crs=%.1f\n", ws_kt, tas_kt, wd_deg, crs_deg);
        // printf("wd=%.4f crs=%.4f\n", wd, crs);
        // printf("wd-crs=%.4f\n", wd-crs);
        // printf("sin(wd-crs)=%.4f\n", sin(wd-crs));
	float swc = (ws_kt/tas_kt)*sin(wd-crs);
	// printf("swc=%.4f\n", swc);
	if ( fabs(swc) > 1.0 ) {
	    // course cannot be flown, wind too strong
	    // point nose into estimated wind and "kite" as best we can
	    hd = wd + M_PI;
	    if ( hd > m2pi ) { hd -= m2pi; }
            // estimate negative ground speed
            *gs_kt = tas_kt - ws_kt;
	} else {
	    hd = crs + asin(swc);
	    if ( hd < 0.0 ) { hd += m2pi; }
	    if ( hd > m2pi ) { hd -= m2pi; }
	    *gs_kt = tas_kt * sqrt(1-swc*swc) - ws_kt * cos(wd - crs);
	}
	*hd_deg = hd * r2d;
    }
    // if ( display_on ) {
    //   printf("af: hd=%.1f gs=%.1f\n", hd * SGD_RADIANS_TO_DEGREES, gs);
    // }
}
