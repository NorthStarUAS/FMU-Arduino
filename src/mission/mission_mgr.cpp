#include "../nodes.h"

#include "mission_mgr.h"

void mission_mgr_t::init() {
    circle_mgr.init();
    route_mgr.init();
    mission_node.setString("mode", "none");
}

void mission_mgr_t::update() {
    if ( mission_node.getString("mode") == "circle" ) {
        circle_mgr.update();
    } else if (mission_node.getString("mode") == "route" ) {
        route_mgr.update();
    }
}

mission_mgr_t *mission_mgr;