#include "../nodes.h"

#include "inceptors.h"
#include "switches.h"

void switches_t::init() {
    printf("setting up switch configuration:\n");
    PropertyNode config_switches_node = PropertyNode("/config/switches");

    vector<string> name_list = config_switches_node.getChildren();
    for ( unsigned int i = 0; i < name_list.size(); i++ ) {
        string name = name_list[i];
        printf("  %s\n", name.c_str());
        PropertyNode node = config_switches_node.getChild(name.c_str());
        if ( node.isNull() ) {
            break;
        }
        if ( not node.hasChild("rc-channel") ) {
            break;
        }
        if ( not node.hasChild("states") ) {
            break;
        }
        switch_t sw;
        sw.name = name;
        sw.rc_channel = node.getInt("rc-channel");
        sw.num_states = node.getUInt("states");
        if ( sw.num_states < 2 ) { sw.num_states = 2; }
        if ( sw.num_states > 6 ) { sw.num_states = 6; }
        switch_list.push_back(sw);
        inceptors_node.setInt(sw.name.c_str(), 0);
    }
}

void switches_t::update() {
    // printf("switch: ");
    for ( unsigned int i = 0; i < switch_list.size(); i++ ) {
        int sbus_val = rcin_node.getUInt("channel", switch_list[i].rc_channel);
        int step = SBUS_RANGE / (switch_list[i].num_states - 1);
        int test_range = step * 0.4;
        int state = 0;
        int test_point = SBUS_MIN_VALUE;
        bool valid = false;
        // printf("[%d] pwm_val: %d ", i, pwm_val);
        while ( test_point - test_range < SBUS_MAX_VALUE ) {
            // printf("[%d] pwm: %d  tp: %d  tr: %d\n", i, pwm_val, test_point, test_range);
            if ( (sbus_val >= test_point - test_range) and (sbus_val <= test_point + test_range) )
            {
                valid = true;
                break;
            }
            test_point += step;
            state++;
        }
        if ( valid ) {
            if ( state >= switch_list[i].num_states ) {
                state = switch_list[i].num_states - 1;
            }
            if ( switch_list[i].last_state >= 0 and switch_list[i].last_state != state ) {
                // we transitioned, update the switch value
                switch_list[i].value = state;
                printf("Switch: %s = %d\n", switch_list[i].name.c_str(), switch_list[i].value);
                inceptors_node.setInt(switch_list[i].name.c_str(), switch_list[i].value);
            }
        }
        switch_list[i].last_state = state;
        //printf(" state: %d value: %d ", state, switch_list[i].value);
    }
    //printf("\n");
}
