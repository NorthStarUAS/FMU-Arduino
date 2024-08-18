#include "../nodes.h"
#include "../comms/events.h"

#include "tasks/circle.h"
#include "tasks/idle.h"
#include "tasks/route.h"
#include "mission_mgr.h"

void mission_mgr_t::init() {
    circle_mgr.init();
    home_mgr.init();
    route_mgr.init();
    mission_node.setString("mode", "none");
}

void mission_mgr_t::update(float dt) {
    // global tasks
    home_mgr.update();

    // sanity check create default task if nothing active
    if ( current_task == nullptr ) {
        start_idle_task();
    }
    process_command_request();

    // update the current task
    current_task->update(dt);

    // run the selected core flight mode
    if ( mission_node.getString("mode") == "circle" ) {
        circle_mgr.update();
    } else if (mission_node.getString("mode") == "route" ) {
        route_mgr.update();
    }
}

void mission_mgr_t::process_command_request() {
    string command = mission_node.getString("request");
    mission_node.setString("request", "");
    if ( command == "circle_here" ) {
        if ( gps_node.getInt("status") == 3 ) {
            double lon_deg = gps_node.getDouble("longitude_deg");
            double lat_deg = gps_node.getDouble("latitude_deg");
            start_circle_task(lon_deg, lat_deg);
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

void mission_mgr_t::start_route_task() {
    // sanity check, are we already running the requested task
    if ( current_task != nullptr and current_task->name == "route" ) {
        event_mgr->add_event("mission", "route already active");
    } else {
        // create and activate task
        route_task_t *route = new route_task_t(circle_node);
        new_task(route);
    }
}

void mission_mgr_t::start_idle_task() {
    if ( current_task != nullptr and current_task->name == "idle" ) {
        // sanity check, are we already running the requested task
    } else {
        // create and activate task
        idle_task_t *task = new idle_task_t(PropertyNode());
        new_task(task);
    }
}

mission_mgr_t *mission_mgr = nullptr;