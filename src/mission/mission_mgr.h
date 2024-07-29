#pragma once

#include "../nodes.h"

class mission_mgr_t {

public:

    mission_mgr_t() {};
    ~mission_mgr_t() {};
    void init();
    void update();

private:

};

extern mission_mgr_t *mission_mgr;

