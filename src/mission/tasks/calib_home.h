#pragma once

#include "../../props2.h"
#include "task.h"

class calib_home_task_t : public task_t {

public:

    calib_home_task_t();
    ~calib_home_task_t() {}

    void activate();
    void update(float dt);
    bool is_complete();
    void close();

private:

    float timer = 0.0;
    float duration_sec = 0.0;

};