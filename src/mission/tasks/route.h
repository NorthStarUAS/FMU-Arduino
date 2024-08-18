#include "../../props2.h"
#include "task.h"

class route_task_t : public task_t {

public:

    route_task_t( PropertyNode config_node );
    ~route_task_t() {}

    void activate();
    void update(float dt);
    bool is_complete();
    void close();

};