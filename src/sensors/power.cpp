#include <Arduino.h>

#include "../../setup_board.h"
#include "../nodes.h"

#include "power.h"

const float analogResolution = 65535.0f;
const float battery_scale = 11.0f;
const float avionics_scale = 2.0f;

void power_t::init() {
    config_power_node = PropertyNode("/config/sensors/power");

    if ( config_power_node.hasChild("battery_cells") ) {
        cells = config_power_node.getDouble("battery_cells");
        if ( config_power_node.hasChild("battery_calibration") ) {
            battery_cal = config_power_node.getDouble("battery_calibration");
        }
    }

#if defined(MARMOT_V1)
    avionics_pin = A22;
    source_volt_pin = 15;
#elif defined(AURA_V2) || defined(NORTHSTAR_V3)
    avionics_pin = A1;
    source_volt_pin = A0;
    if ( config_power_node.getBool("have_attopilot") ) {
        printf("Attopilot enabled.\n");
        atto_volts_pin = A2;
        atto_amps_pin = A3;
    }
#else
    printf("Master board configuration not defined correctly.\n");
#endif

    battery_volts.set_time_factor(5.0);
    avionics_volts.set_time_factor(5.0);
}

void power_t::update(float dt) {
    // battery voltage
    uint16_t ain;
    float volts;
    ain = analogRead(source_volt_pin);
    volts = float(ain) * 3.3 / analogResolution * battery_scale * battery_cal;
    battery_volts.update(volts, dt);
    double cell_vcc = 0.0;
    if ( cells > 0 ) {
        cell_vcc = battery_volts.get_value() / cells;
    }
    power_node.setDouble("main_vcc", battery_volts.get_value());
    power_node.setDouble("main_amps", 0);  // fixme (attopilot?)
    power_node.setDouble("cell_vcc", cell_vcc);

    ain = analogRead(avionics_pin);
    volts = ((float)ain) * 3.3 / analogResolution * avionics_scale;
    avionics_volts.update(volts, dt);
    // Serial.print("pin: "); Serial.print(source_volt_pin);
    // Serial.print(" raw: "); Serial.print(ain); Serial.print(" volts: "); Serial.print(volts, 3);
    // Serial.print(" filt: "); Serial.print(avionics_volts.get_value()); Serial.print(" "); Serial.println(dt);
    power_node.setDouble("avionics_vcc", avionics_volts.get_value());

    // fixme: what about amps

    if ( config_power_node.getBool("have_attopilot") ) {
        ain = analogRead(atto_volts_pin);
        // printf("atto volts: %.2f\n", ((float)ain) * 3.3 / analogResolution );
        // fixme: don't have a system currently with attopilot-based power to
        // test with
    }
}