// configuration and eeprom

#pragma once

#include <Arduino.h>

#include "aura3_messages.h"

class config_t {
private:
    int config_size = 0;
    
public:
    message::config_master_t master;
    message::config_power_t power;
    
    uint16_t read_serial_number();
    uint16_t set_serial_number(uint16_t value);
    void load_defaults();
    void master_defaults();
    void power_defaults();
    int read_eeprom();
    int write_eeprom();
};

extern config_t config;
