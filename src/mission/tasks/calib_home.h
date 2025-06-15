#pragma once

#include "../../props2.h"
#include "task.h"

class calib_home_task_t : public task_t {

public:

    calib_home_task_t();
    ~calib_home_task_t() {}

    void activate();
    void update( float dt );
    bool is_complete();
    void close();

private:

    float duration_sec = 30.0;
    double longitude_sum = 0.0;
    double latitude_sum = 0.0;
    float altitude_sum = 0.0;
    float timer = 0.0;
    int counter = 0;

};