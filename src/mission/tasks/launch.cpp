#include <string>
using std::string;

#include "../../nodes.h"
#include "../../fcs/fcs_mgr.h"
#include "../../util/constants.h"
#include "launch.h"

launch_task_t::launch_task_t() {
    name = "launch";

    PropertyNode config_node = PropertyNode("/config/mission/launch");
    if ( config_node.hasChild("completion_agl_ft") ) {
        completion_agl_ft = config_node.getDouble("completion_agl_ft");
    }
    if ( config_node.hasChild("mission_agl_ft") ) {
        mission_agl_ft = config_node.getDouble("mission_agl_ft");
    }
    if ( config_node.hasChild("ref_airspeed_kt") ) {
        ref_airspeed_kt = config_node.getDouble("ref_airspeed_kt");
    }
    if ( config_node.hasChild("roll_gain") ) {
        roll_gain = config_node.getDouble("roll_gain");
    }
    if ( config_node.hasChild("roll_limit") ) {
        roll_limit = config_node.getDouble("roll_limit");
    }
    if ( config_node.hasChild("rudder_enable") ) {
        rudder_enable = config_node.getBool("rudder_enable");
    }
    if ( config_node.hasChild("rudder_gain") ) {
        rudder_gain = config_node.getDouble("rudder_gain");
    }
    if ( config_node.hasChild("rudder_max") ) {
        rudder_max = config_node.getDouble("rudder_max");
    }
    if ( config_node.hasChild("flaps") ) {
        flaps = config_node.getDouble("flaps");
    }
    if ( config_node.hasChild("ref_pitch_deg") ) {
        ref_pitch_deg = config_node.getDouble("ref_pitch_deg");
    }
}

void launch_task_t::activate() {
    if ( airdata_node.getBool("is_airborne") ) {
        return;
    }

    active = true;

    if ( launch_mode == "surface" ) {
        // start with roll control only, we fix elevator to neutral until flight
        // speeds come up and steer the rudder directly
        fcs_mgr->set_mode("roll");
    } else {
        // hand/cat launch, start flying immediately
        fcs_mgr->set_mode("roll+pitch");
        refs_node.setDouble("pitch_deg", ref_pitch_deg);
        refs_node.setDouble("roll_deg", 0.0);
        refs_node.setDouble("altitude_agl_ft", mission_agl_ft);
        refs_node.setDouble("airspeed_kt", ref_airspeed_kt);
    }
}

void launch_task_t::update(float dt) {
    if ( not active ) {
        return;
    }

    float throttle_time_sec = 2.0;  // hard code for now (fixme: move to config)
    float feather_time = 5.0;       // fixme: make this a configurable option

    bool is_airborne = airdata_node.getBool("is_airborne");

    // For wheeled take offs, track relative heading (initialized to zero) when
    // autopilot mode is engaged and steer that error to zero with the rudder
    // until flying/climbing

    if ( inceptors_node.getBool("master_switch") ) {
        if ( not last_ap_master ) {
            // reset states when engaging AP mode
            relhdg = 0.0;
            control_limit = 1.0;
            refs_node.setDouble("flaps_setpoint", flaps);
            power = 0.0;
        }
        if ( not is_airborne ) {
            // run up throttle over the specified interval
            power += dt / throttle_time_sec;
            if ( power > 1.0 ) {
                power = 1.0;
            }
            outputs_node.setDouble("power", power);
        }
        // estimate short term heading
        relhdg += imu_node.getDouble("r_rps") * r2d * dt;

        // I am not clamping heading to +/- 180 here.  The rational is that if
        // we turn more than 180 beyond our starting heading we are probably
        // majorly screwed up, but even so, let's unwind back the direction from
        // where we came rather than doing a hard-over reversal of the rudder
        // back to the other direction even if it would be a shorter turning
        // direction.  Less chance of upsetting the apple cart with a huge
        // transient.

        // if airborne, then slowly feather our max steer_limit to zero over the
        // period of "feather_time" seconds.  This avoids another potential hard
        // transient if we are off heading, but the assumption is once we are
        // airborne we'll prefer to keep our wings level and fly the current
        // heading rather than fight with our rudder and add drag to get back on
        // the heading.

        if ( is_airborne and control_limit > 0 ) {
            float delta = dt / feather_time;
            control_limit -= delta;
            if ( control_limit < 0.0 ) {
                control_limit = 0.0;
            }
        }
        if ( rudder_enable ) {
            // simple proportional controller to steer against (estimated)
            // heading error
            float yaw_cmd = relhdg * rudder_gain;
            float steer_limit = rudder_max * control_limit;
            if ( yaw_cmd < -steer_limit ) {
                yaw_cmd = -steer_limit;
            }
            if ( yaw_cmd > steer_limit ) {
                yaw_cmd = steer_limit;
            }
            outputs_node.setDouble("yaw", yaw_cmd);
        }
        float roll_deg = -relhdg * roll_gain;
        if ( roll_deg < -roll_limit ) {
            roll_deg = -roll_limit;
        }
        if ( roll_deg > roll_limit ) {
            roll_deg = roll_limit;
        }
        refs_node.setDouble("roll_deg", roll_deg);

        if ( is_airborne or airdata_node.getDouble("airspeed_mps") > ref_airspeed_kt ) {
            // we are flying or we've reached our flying/climbout airspeed:
            // switch to normal flight mode
            if ( fcs_mgr->get_mode() != "basic+tecs" ) {
                fcs_mgr->set_mode("basic+tecs");
            }
        }
    }

    last_ap_master = inceptors_node.getBool("master_switch");
}

bool launch_task_t::is_complete() {
    if ( not active ) {
        // not active == complete
        return true;
    } else if ( airdata_node.getDouble("altitude_agl_m") * m2ft >= completion_agl_ft ) {
        refs_node.setDouble("flaps_setpoint", 0.0);  // raise flaps
        // in case we get to the completion altitude before we've feathered
        // the rudder input, let's center the rudder.
        if ( rudder_enable ) {
            outputs_node.setDouble("yaw", 0.0);
        }
        mission_node.setString("request", "circle_here");
        return true;
    }
    return false;
}

void launch_task_t::close() {
    active = false;
}