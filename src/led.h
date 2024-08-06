// das blinken class

#pragma once

#include <Arduino.h>

class led_t {

private:

    uint8_t led_pin = 0;
    elapsedMillis blinkTimer = 0;
    unsigned int blink_rate = 100;
    bool blink_state = true;

public:

    void init();
    void update();
};

extern led_t led;
