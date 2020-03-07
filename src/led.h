// das blinken class

#include <Arduino.h>

#include "../aura3_messages.h"

class led_t {
private:
    elapsedMillis blinkTimer = 0;
    unsigned int blink_rate = 100;
    bool blink_state = true;
    
public:
    message::config_led_t config;
    void defaults_goldy3();
    void defaults_aura3();
    void setup();
    void update(int gyros_calibrated, int gps_fix);
};
