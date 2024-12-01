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
    counter = 0;

    if ( not environment_node.getBool("is_airborne") ) {
        // set fcs mode to roll+pitch
        fcs_mgr->set_mode("roll+pitch");
        refs_node.setDouble("roll_deg", 0.0);
        refs_node.setDouble("pitch_deg", 0.0);
        refs_node.setDouble("flaps_setpoint", 0.0);
        // reset timer
        timer = 0.0;
    } else {
        // we are airborne, don't change modes and configure timer to be already
        // expired
        timer = duration_sec + 1.0;
    }
}

void calib_home_task_t::update(float dt) {
    // sample current position
    latitude_sum += gps_node.getDouble("latitude_deg");
    longitude_sum += gps_node.getDouble("longitude_deg");
    altitude_sum += gps_node.getDouble("altitude_m");
    counter += 1;

    // refine home location continuously (note that the landing task /
    // glideslope math uses ground and agl altitude computed in the environment
    // section.)
    home_node.setDouble("longitude_deg", longitude_sum / (double)counter);
    home_node.setDouble("latitude_deg", latitude_sum / (double)counter);
    home_node.setDouble("altitude_m", altitude_sum / (float)counter);
    home_node.setBool("valid", true);

    timer += dt;
}

bool calib_home_task_t::is_complete() {
    // print "timer=%.1f duration=%.1f" % (self.timer, self.duration_sec)
    // complete when timer expires or we sense we are airborne (sanity check!)
    if ( timer >= duration_sec or environment_node.getBool("is_airborne") ) {
        return true;
    } else {
        return false;
    }
}

void calib_home_task_t::close() {
    active = false;
}