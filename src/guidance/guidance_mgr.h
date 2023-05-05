#include "../props2.h"

class guidance_mgr_t {

public:
    void init();
    void update( float dt );

private:
    PropertyNode nav_node;
};
