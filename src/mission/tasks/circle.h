#include "../../props2.h"
#include "task.h"

class circle_task_t : public task_t {

public:

    circle_task_t( PropertyNode config_node );
    ~circle_task_t() {}

    void activate();
    void update(float dt);
    bool is_complete();
    void close();

};