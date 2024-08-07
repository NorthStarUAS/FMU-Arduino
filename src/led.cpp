#include "../setup_board.h"
#include "nodes.h"
#include "led.h"

void led_t::init() {
#if defined(MARMOT_V1)
    led_pin = 0;
#elif defined(AURA_V2) || defined(NORTHSTAR_V3)
    led_pin = 13;
#endif

    if ( led_pin > 0 ) {
        pinMode(led_pin, OUTPUT);
        digitalWrite(led_pin, HIGH);
        printf("LED on pin: %d\n", led_pin);
    } else {
        printf("No LED defined.\n");
    }
}

void led_t::update() {
    if ( led_pin > 0 ) {
        if ( imu_node.getUInt("gyros_calibrated") < 2 ) {
            blink_rate = 50;
        } else if ( gps_node.getUInt("status") < 3 ) {
            blink_rate = 200;
        } else {
            blink_rate = 800;
        }
        if ( blinkTimer >= blink_rate ) {
            blinkTimer = 0;
            blink_state = !blink_state;
            digitalWrite(led_pin, blink_state);
        }
    }
}

// global shared instance
led_t led;
