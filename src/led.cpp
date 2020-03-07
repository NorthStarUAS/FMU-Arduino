#include "led.h"

void led_t::defaults_goldy3() {
    config.pin = 0;
}

void led_t::defaults_aura3() {
    config.pin = 13;
}

void led_t::setup() {
    if ( config.pin > 0 ) {
        pinMode(config.pin, OUTPUT);
        digitalWrite(config.pin, HIGH);
        Serial.print("LED on pin: "); Serial.println(config.pin);
    } else {
        Serial.println("No LED defined.");
    }
}

void led_t::update(int gyros_calibrated, int gps_fix) {
    if ( config.pin > 0 ) {

        if ( gyros_calibrated < 2 ) {
            blink_rate = 50;
        } else if ( gps_fix < 3 ) {
            blink_rate = 200;
        } else {
            blink_rate = 800;
        }
        if ( blinkTimer >= blink_rate ) {
            blinkTimer = 0;
            blink_state = !blink_state;
            digitalWrite(config.pin, blink_state);
        }
    }
}
        
// global shared instance
led_t led;
