// configuration and eeprom

#pragma once

#include <Arduino.h>

#include "props2.h"

class config_t {

private:

    PropertyNode config_node;

public:

    void init();
    uint16_t read_serial_number();
    uint16_t set_serial_number(uint16_t value);
    bool load_json_config();
    void reset_defaults();
};

extern config_t config;
