#include "../props2.h"

class circle_mgr_t {

public:
    void init();
    void update( float dt );

private:
    PropertyNode circle_node;
    PropertyNode pos_node;
    PropertyNode vel_node;
    PropertyNode orient_node;
    PropertyNode route_node;
    PropertyNode L1_node;
    PropertyNode targets_node;
};

extern circle_mgr_t circle_mgr;
