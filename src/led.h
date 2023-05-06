// das blinken class

#pragma once

#include <Arduino.h>

#include "props2.h"

class led_t {

private:

    PropertyNode gps_node;
    elapsedMillis blinkTimer = 0;
    unsigned int blink_rate = 100;
    bool blink_state = true;

public:

    void defaults_goldy3();
    void defaults_aura3();
    void setup();
    void update(int gyros_calibrated); // fixme: this should really be added and taken from the property tree
};

extern led_t led;
