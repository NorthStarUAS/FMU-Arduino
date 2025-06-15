#include "../../nodes.h"
#include "../../fcs/fcs_mgr.h"
#include "calib_home.h"

calib_home_task_t::calib_home_task_t() {
    name = "calib_home";
    PropertyNode config_node = PropertyNode("/config/mission/calib_home");
    if ( config_node.hasChild("duration_sec") ) {
        duration_sec = config_node.getDouble("duration_sec");
    }
}

void calib_home_task_t::activate() {
    active = true;

    latitude_sum = 0.0;
    longitude_sum = 0.0;
    altitude_sum = 0.0;
    timer = 0.0;
    counter = 0;

    // start with calibrated == false if we request a new calibration
    home_node.setBool("calibrated", false);

    if ( environment_node.getBool("is_airborne") ) {
        // we are airborne, can't calibrate ground elevation, set timer to be
        // already expired
        timer = duration_sec + 1.0;
    } else if ( gps_node.getInt("status") < 3 or not gps_node.getBool("settle") ) {
        // no settled gps fix, can't set home position, sorry, set timer to be already
        // expired
        timer = duration_sec + 1.0;
    } else {
        // set fcs mode to roll+pitch
        fcs_mgr->set_mode("roll+pitch");
        refs_node.setDouble("roll_deg", 0.0);
        refs_node.setDouble("pitch_deg", 0.0);
        refs_node.setDouble("flaps_setpoint", 0.0);
    }
}

void calib_home_task_t::update( float dt ) {
    if ( environment_node.getBool("is_airborne") or gps_node.getInt("status") < 3 ) {
        // force early exit from this task if we become airborne or lose gps fix!
        timer = duration_sec + 1.0;
    } else {
        // sample current position
        latitude_sum += gps_node.getDouble("latitude_deg");
        longitude_sum += gps_node.getDouble("longitude_deg");
        altitude_sum += gps_node.getDouble("altitude_m");
        counter += 1;
        timer += dt;
    }
}

bool calib_home_task_t::is_complete() {
    // print "timer=%.1f duration=%.1f" % (self.timer, self.duration_sec)
    // complete when timer expires or we sense we are airborne (sanity check!)
    if ( timer >= duration_sec ) {
        // task ran to completion
        if ( counter > 0 ) {
            // we successfully accumulated valid data points while on the ground
            home_node.setDouble("longitude_deg", longitude_sum / (double)counter);
            home_node.setDouble("latitude_deg", latitude_sum / (double)counter);
            home_node.setDouble("altitude_m", altitude_sum / (float)counter);
            home_node.setBool("valid", true);

            // mark calibration as good
            home_node.setBool("calibrated", true);
        }
        return true;
    }
    return false;
}

void calib_home_task_t::close() {
    active = false;
}