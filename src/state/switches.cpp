#include "switches.h"

void switches_t::init() {
    printf("setting up switch configuration:\n");
    config_node = PropertyNode("/config/switches");
    rcin_node = PropertyNode("/sensors/rc-input");
    switches_node = PropertyNode("/switches");

    vector<string> name_list = config_node.getChildren();
    for ( unsigned int i = 0; i < name_list.size(); i++ ) {
        string name = name_list[i];
        PropertyNode node = config_node.getChild(name.c_str());
        if ( node.isNull() ) {
            break;
        }
        if ( !node.hasChild("rc-channel") ) {
            break;
        }
        if ( !node.hasChild("states") ) {
            break;
        }
        switch_t sw;
        sw.name = name;
        sw.rc_channel = node.getInt("rc-channel");
        sw.num_states = node.getUInt("states");
        if ( sw.num_states < 2 ) { sw.num_states = 2; }
        if ( sw.num_states > 6 ) { sw.num_states = 6; }
        switch_list.push_back(sw);
        switches_node.setInt(sw.name.c_str(), 0);
    }
}

void switches_t::update() {
    // printf("switch: ");
    for ( unsigned int i = 0; i < switch_list.size(); i++ ) {
        int pwm_val = rcin_node.getUInt("channel", switch_list[i].rc_channel);
        int step = 1000 / (switch_list[i].num_states - 1);
        int test_range = step * 0.4;
        int state = 0;
        int test_point = 1000;
        bool valid = false;
        // printf("[%d] pwm_val: %d ", i, pwm_val);
        while ( test_point - test_range < 2000 ) {
            // printf("[%d] pwm: %d  tp: %d  tr: %d\n", i, pwm_val, test_point, test_range);
            if ( (pwm_val >= test_point - test_range) and (pwm_val <= test_point + test_range) )
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
                switches_node.setInt(switch_list[i].name.c_str(), switch_list[i].value);
            }
        }
        switch_list[i].last_state = state;
        //printf(" state: %d value: %d ", state, switch_list[i].value);
    }
    //printf("\n");
}
