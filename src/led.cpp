#include "config.h"

#include "led.h"

void led_t::defaults_goldy3() {
    config.led.pin = 0;
}

void led_t::defaults_aura3() {
    config.led.pin = 13;
}

void led_t::setup() {
    if ( config.led.pin > 0 ) {
        pinMode(config.led.pin, OUTPUT);
        digitalWrite(config.led.pin, HIGH);
        Serial.print("LED on pin: "); Serial.println(config.led.pin);
    } else {
        Serial.println("No LED defined.");
    }
}

void led_t::update(int gyros_calibrated, int gps_fix) {
    if ( config.led.pin > 0 ) {

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
            digitalWrite(config.led.pin, blink_state);
        }
    }
}
        
// global shared instance
led_t led;
