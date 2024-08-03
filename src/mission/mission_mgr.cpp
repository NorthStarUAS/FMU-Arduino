#include "../nodes.h"
#include "../comms/events.h"

#include "tasks/circle.h"
#include "mission_mgr.h"

void mission_mgr_t::init() {
    circle_mgr.init();
    route_mgr.init();
    mission_node.setString("mode", "none");
}

void mission_mgr_t::update() {
    // sanity check create default task if nothing active
    if ( current_task == nullptr ) {
        if ( airdata_node.getBool("is_airborne") ) {
            if ( gps_node.getInt("status") == 3 ) {
                double lon_deg = gps_node.getDouble("longitude_deg");
                double lat_deg = gps_node.getDouble("latitude_deg");
                request_task_circle(lon_deg, lat_deg);
            }
        } else {
            // fixme: create idle task
        }
    }
    if ( mission_node.getString("mode") == "circle" ) {
        circle_mgr.update();
    } else if (mission_node.getString("mode") == "route" ) {
        route_mgr.update();
    }
}

void mission_mgr_t::new_task(task_t *task) {
    if ( current_task != nullptr ) {
        current_task->close();
        delete current_task;
    }
    current_task = task;
    current_task->activate();
    event_mgr->add_event("mission", "new task: " + current_task->name);
}

void mission_mgr_t::request_task_circle(double lon_deg, double lat_deg) {
    if ( fabs(lon_deg) < 0.1 or fabs(lat_deg) < 0.1 ) {
        // no valid coordinates specified
        return;
    }

    // set the target coordinates
    circle_node.setDouble( "longitude_deg", lon_deg );
    circle_node.setDouble( "latitude_deg", lat_deg );

    // sanity check, are we already running the requested task
    if ( current_task->name == "circle" ) {
        event_mgr->add_event("mission", "updating circle location");
    } else {
        // create and activate task
        circle_t *circle = new circle_t(circle_node);
        new_task(circle);
    }
}

mission_mgr_t *mission_mgr = nullptr;