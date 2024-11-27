// This is the section of code that derives, computes, and estimates
// things that aren't directly sensed.

#include "../nodes.h"
#include "env_mgr.h"

void env_mgr_t::init() {
    airdata.init();
    ground.init();
    wind.init();
}

void env_mgr_t::update(float dt) {
    airdata.update(dt);
    ground.update(dt);
    wind.update(dt);
    environment_node.setUInt("millis", millis());
}

env_mgr_t env_mgr;