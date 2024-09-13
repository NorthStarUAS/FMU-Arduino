#pragma once

#include "../props2.h"
#include "../nodes.h"
#include "../sensors/pwm.h"
#include "mixer.h"

class effectors_t {

private:

    PropertyNode config_eff_gains_node;

public:

    mixer_t mixer;

    void init();
    void write( PropertyNode input_node );

};