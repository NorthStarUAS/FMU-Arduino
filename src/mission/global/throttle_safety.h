#pragma once

#include <string>
using std::string;

class throttle_safety_task_t {

public:

    void init();
    void update();

private:

    bool airborne_latch = false;

};