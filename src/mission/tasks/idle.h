#pragma once

#include "../../props2.h"
#include "task.h"

class idle_task_t : public task_t {

public:

    idle_task_t();
    ~idle_task_t() {}

    void activate();
    void update(float dt);
    bool is_complete();
    void close();

};