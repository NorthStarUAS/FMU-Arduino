// note: circle_mgr and route_mgr exist in the global shared space.
#include "circle_mgr.h"
#include "route_mgr.h"

#include "guidance_mgr.h"

void guidance_mgr_t::init() {
    nav_node = PropertyNode("/navigation");
    circle_mgr.init();
    route_mgr.init();
}

void guidance_mgr_t::update( float dt ) {
    if ( nav_node.getString("mode") == "circle" ) {
        circle_mgr.update( dt );
    } else if (nav_node.getString("mode") == "route" ) {
        route_mgr.update( dt );
    }
}
