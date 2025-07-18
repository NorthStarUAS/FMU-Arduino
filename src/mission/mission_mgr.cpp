#include "../nodes.h"
#include "../comms/events.h"
#include "../util/profile.h"

#include "tasks/circle.h"
#include "tasks/idle.h"
#include "tasks/launch.h"
#include "tasks/land4.h"
#include "tasks/preflight.h"
#include "tasks/calib_home.h"
#include "tasks/route.h"
#include "mission_mgr.h"

void mission_mgr_t::init() {
    circle_mgr.init();
    home_mgr.init();
    motor_safety.init();
    route_mgr.init();
    mission_node.setString("mode", "none");
}

void mission_mgr_t::update(float dt) {
    mission_prof.start();

    // global tasks
    home_mgr.update();
    motor_safety.update();

    // lost link action, but only if airborne
    if ( not comms_node.getBool("link_state") and last_link_state and environment_node.getBool("is_airborne") ) {
        // link became bad, circle home, minimum agl is 200'
        event_mgr->add_event("mission", "circle home");
        mission_node.setString("request", "circle_home");
        float ref_agl = refs_node.getDouble("altitude_agl_ft");
        if ( ref_agl < 200.0 ) {
            refs_node.setDouble("altitude_agl_ft", 200.0);
        }
    }
    last_link_state = comms_node.getBool("link_state");

    process_command_request();

    // sanity check create default task if nothing active
    if ( current_task == nullptr ) {
        start_idle_task();
    }

    // update the current task
    current_task->update(dt);

    // run the selected core flight mode
    if ( mission_node.getString("mode") == "circle" ) {
        circle_mgr.update();
    } else if (mission_node.getString("mode") == "route" ) {
        route_mgr.update();
    }

    if ( current_task->is_complete() ) {
        current_task->close();
        delete current_task;
        current_task = nullptr;
    }

    mission_prof.stop();
}

void mission_mgr_t::process_command_request() {
    string command = mission_node.getString("request");
    if ( command.length() ) {
        mission_node.setString("request", "");
        event_mgr->add_event("mission_mgr request", command);
        if ( command == "preflight") {
            start_preflight_task();
        } else if ( command == "calib_home") {
            start_calib_home_task();
        } else if ( command == "launch") {
            start_launch_task();
        } else if ( command == "land") {
            start_land_task();
        } else if ( command == "circle_here" ) {
            if ( gps_node.getInt("status") == 3 ) {
                double lon_deg = gps_node.getDouble("longitude_deg");
                double lat_deg = gps_node.getDouble("latitude_deg");
                start_circle_task(lon_deg, lat_deg);
            } else {
                // fixme: we are lost and kinda screwed!
            }
        } else if ( command == "circle_home" ) {
            if ( home_node.getBool("valid") ) {
                // if we have a valid home location
                double lon_deg = home_node.getDouble("longitude_deg");
                double lat_deg = home_node.getDouble("latitude_deg");
                start_circle_task(lon_deg, lat_deg);
            } else if ( gps_node.getInt("status") == 3 ) {
                // plan b, circle here
                double lon_deg = gps_node.getDouble("longitude_deg");
                double lat_deg = gps_node.getDouble("latitude_deg");
                start_circle_task(lon_deg, lat_deg);
            } else {
                // fixme: we are lost and kinda screwed!
            }
        } else if ( command == "route") {
            start_route_task();
        } else {
            event_mgr->add_event("mission_mgr task unknown", command);
        }
    }
}

void mission_mgr_t::new_task(task_t *task) {
    if ( current_task != nullptr ) {
        current_task->close();
        delete current_task;
    }
    current_task = task;
    current_task->activate();
    mission_node.setString("task", current_task->name);
    event_mgr->add_event("mission", "new task: " + current_task->name);
}

void mission_mgr_t::start_circle_task(double lon_deg, double lat_deg) {
    if ( fabs(lon_deg) < 0.1 and fabs(lat_deg) < 0.1 ) {
        // no valid coordinates specified
        return;
    }

    // set the target coordinates
    circle_node.setDouble( "longitude_deg", lon_deg );
    circle_node.setDouble( "latitude_deg", lat_deg );

    // sanity check, are we already running the requested task
    if ( current_task != nullptr and current_task->name == "circle" ) {
        event_mgr->add_event("mission", "updating circle location");
    } else {
        // create and activate task
        circle_task_t *circle = new circle_task_t(circle_node);
        new_task(circle);
    }
}

void mission_mgr_t::start_idle_task() {
    if ( current_task != nullptr and current_task->name == "idle" ) {
        // sanity check, are we already running the requested task
    } else {
        // create and activate task
        idle_task_t *task = new idle_task_t();
        new_task(task);
    }
}

void mission_mgr_t::start_launch_task() {
    // sanity check, are we already running the requested task
    if ( current_task != nullptr and current_task->name == "launch" ) {
        event_mgr->add_event("mission", current_task->name + " already active");
    } else {
        // create and activate task
        launch_task_t *launch = new launch_task_t();
        new_task(launch);
    }
}

void mission_mgr_t::start_land_task() {
    // sanity check, are we already running the requested task
    if ( current_task != nullptr and current_task->name == "land" ) {
        event_mgr->add_event("mission", current_task->name + " already active");
    } else {
        // create and activate task
        land_task4_t *land = new land_task4_t();
        new_task(land);
    }
}

void mission_mgr_t::start_preflight_task() {
    // sanity check, are we already running the requested task
    if ( current_task != nullptr and current_task->name == "preflight" ) {
        event_mgr->add_event("mission", current_task->name + " already active");
    } else {
        // create and activate task
        preflight_task_t *preflight = new preflight_task_t();
        new_task(preflight);
    }
}

void mission_mgr_t::start_calib_home_task() {
    // sanity check, are we already running the requested task
    if ( current_task != nullptr and current_task->name == "calib_home" ) {
        event_mgr->add_event("mission", current_task->name + " already active");
    } else {
        // create and activate task
        calib_home_task_t *calib_home = new calib_home_task_t();
        new_task(calib_home);
    }
}

void mission_mgr_t::start_route_task() {
    // sanity check, are we already running the requested task
    if ( current_task != nullptr and current_task->name == "route" ) {
        event_mgr->add_event("mission", current_task->name + " already active");
    } else {
        // create and activate task
        route_task_t *route = new route_task_t(circle_node);
        new_task(route);
    }
}

mission_mgr_t *mission_mgr = nullptr;