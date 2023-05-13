#include <math.h>

#include "../nodes.h"

#include "wind.h"

// initialize wind estimator variables
void wind_est_t::init() {
    airdata_node.setDouble( "pitot_scale_factor", 1.0 );

    we_filt.set_time_factor(60.0);
    wn_filt.set_time_factor(60.0);
    pitot_scale_filt.set_time_factor(120.0);
    pitot_scale_filt.init(1.0);
}


// onboard wind estimate (requires airspeed, true heading, and ground
// velocity vector, only produces valid estimates for fixed wing in
// flight.)
void wind_est_t::update( double dt ) {
    const double d2r = M_PI / 180.0; // degrees to radians
    const double r2d = 180.0 / M_PI; // radians to degrees

    double airspeed_mps = airdata_node.getDouble("airspeed_mps");
    double pitot_scale = pitot_scale_filt.get_value();

    if ( airdata_node.getBool("is_airborne") ) {
        // only update wind estimate when airborne
        double psi = M_PI_2 - nav_node.getDouble("yaw_deg") * d2r;
        double ue = cos(psi) * (airspeed_mps * pitot_scale);
        double un = sin(psi) * (airspeed_mps * pitot_scale);
        double we = ue - nav_node.getDouble("ve_mps");
        double wn = un - nav_node.getDouble("vn_mps");
        we_filt.update(we, dt);
        wn_filt.update(wn, dt);
    }

    double we_filt_val = we_filt.get_value();
    double wn_filt_val = wn_filt.get_value();

    double wind_deg = 90 - atan2( wn_filt_val, we_filt_val ) * r2d;
    if ( wind_deg < 0 ) {
        wind_deg += 360.0;
    }
    double wind_speed_mps = sqrt( we_filt_val*we_filt_val
                                  + wn_filt_val*wn_filt_val );

    airdata_node.setDouble( "wind_speed_mps", wind_speed_mps );
    airdata_node.setDouble( "wind_dir_deg", wind_deg );
    //airdata_node.setDouble( "we_mps", we_filt_val );
    //airdata_node.setDouble( "wn_mps", wn_filt_val );

    // estimate pitot tube bias
    double true_e = we_filt_val + nav_node.getDouble("ve_mps");
    double true_n = wn_filt_val + nav_node.getDouble("vn_mps");

    double true_deg = 90 - atan2( true_n, true_e ) * r2d;
    if ( true_deg < 0 ) {
        true_deg += 360.0;
    }
    double true_speed_mps = sqrt( true_e*true_e + true_n*true_n );

    airdata_node.setDouble( "true_airspeed_mps", true_speed_mps );
    airdata_node.setDouble( "true_heading_deg", true_deg );

    double ps = 1.0;
    if ( airspeed_mps > 1.0 ) {
	ps = true_speed_mps / airspeed_mps;
	// don't let the scale factor exceed some reasonable limits
	if ( ps < 0.75 ) { ps = 0.75;	}
	if ( ps > 1.25 ) { ps = 1.25; }
    }

    pitot_scale_filt.update(ps, dt);
    airdata_node.setDouble( "pitot_scale_factor",
                            pitot_scale_filt.get_value() );

    // printf("true: %.2f kt  %.1f deg (scale = %.4f)\n",
    //        true_speed_kt, true_deg, pitot_scale_filt);

#if 0
    // given the wind estimate and air/body values, estimate ground
    // speed/track -- presumably this should match the gps and filter
    // estimates if our wind estimate is accurate.
    double ve_est = ue - we_filt_val;
    double vn_est = un - wn_filt_val;
    double groundtrack_est_deg = 90 - atan2( vn_est, ve_est ) * r2d;
    if ( groundtrack_est_deg < 0 ) {
        groundtrack_est_deg += 360.0;
    }
    double groundspeed_est_mps = sqrt( ve_est*ve_est + vn_est*vn_est );
    // double groundspeed_est_kt = groundspeed_est_ms * SG_MPS_TO_KT;
    vel_node.setDouble( "groundspeed_est_mps", groundspeed_est_mps );
    orient_node.setDouble( "groundtrack_est_deg", groundtrack_est_deg );
#endif

}
