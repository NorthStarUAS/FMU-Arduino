#pragma once

#include <string>
using std::string;

class throttle_enable_task_t {

public:

    void init();
    void update();

private:

    bool master_enable = false;
    string safety_mode = "on_ground";
    bool airborne_latch = false;

};