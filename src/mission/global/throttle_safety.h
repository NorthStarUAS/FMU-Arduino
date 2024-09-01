#pragma once

#include <string>
using std::string;

class throttle_safety_task_t {

public:

    void init();
    void update();

private:

    bool master_safety = true;
    string safety_mode = "on_ground";
    bool airborne_latch = false;

};