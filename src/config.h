// configuration and eeprom

#pragma once

#include <Arduino.h>

class config_t {

public:

    uint16_t read_serial_number();
    uint16_t set_serial_number(uint16_t value);
    bool load_json_config();
    void reset_defaults();
};

extern config_t config;
