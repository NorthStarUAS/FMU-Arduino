#include <Arduino.h>

#include "config.h"

#include "power.h"

void power_t::setup(int board) {
    if ( board == 0 ) {
        // Marmot v1
        #ifdef HAVE_TEENSY36    // A22 doesn't exist for teensy3.2
        avionics_pin = A22;
        #endif
        source_volt_pin = 15;
    } else if ( board == 1 ) {
        // Aura v2
        avionics_pin = A1;
        source_volt_pin = A0;
        if ( config.have_attopilot ) {
            Serial.println("Attopilot enabled.");
            atto_volts_pin = A2;
            atto_amps_pin = A3;
        }
    } else {
        Serial.println("Master board configuration not defined correctly.");
    }

}

void power_t::update() {
    // battery voltage
    uint16_t ain;
    ain = analogRead(source_volt_pin);
    pwr1_v = ((float)ain) * 3.3 / analogResolution * pwr_scale;

    ain = analogRead(avionics_pin);
    avionics_v = ((float)ain) * 3.3 / analogResolution * avionics_scale;

    if ( config.have_attopilot ) {
        ain = analogRead(atto_volts_pin);
        // Serial.print("atto volts: ");
        // Serial.println( ((float)ain) * 3.3 / analogResolution );
    }
}

// shared global instance
power_t power;
