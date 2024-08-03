#include "../../props2.h"
#include "task.h"

class circle_task_t : public task_t {

private:

    // string direction = "left";
    // float radius_m = 100.0;

public:

    circle_task_t( PropertyNode config_node );
    ~circle_task_t() {}

    void activate();
    // void update_parameters();
    void update(float dt);
    bool is_complete();
    void close();

};