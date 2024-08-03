#pragma once

#include <string>
using std::string;

class task_t {

protected:

public:

    string name = "";
    bool active = false;

    task_t() {}
    virtual ~task_t() {}
    virtual void activate() = 0;
    virtual void update(float dt) = 0;
    virtual bool is_complete() = 0;
    virtual void close() = 0;

};