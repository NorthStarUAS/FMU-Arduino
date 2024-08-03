#pragma once

#include <string>
using std::string;

class task_t {

protected:

    string name = "";
    bool active = false;

public:

    virtual ~task_t() {}
    virtual void activate() = 0;
    virtual void update() = 0;
    virtual void is_complete() = 0;
    virtual void close() = 0;

};