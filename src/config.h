// configuration and eeprom

#pragma once

#include <Arduino.h>

#include "aura4_messages.h"

class config_t {
private:
    int config_size = 0;
    
    struct packed_config_t {
        message::config_master_t::_compact_t master;
        message::config_airdata_t::_compact_t airdata;
        message::config_imu_t::_compact_t imu;
        message::config_led_t::_compact_t led;
        message::config_mix_matrix_t::_compact_t mix_matrix;
        message::config_power_t::_compact_t power;
        message::config_pwm_t::_compact_t pwm;
        message::config_stab_damping_t::_compact_t stab;
    };

public:
    message::config_master_t master;
    message::config_airdata_t airdata;
    message::config_imu_t imu;
    message::config_led_t led;
    message::config_mix_matrix_t mix_matrix;
    message::config_power_t power;
    message::config_pwm_t pwm;
    message::config_stab_damping_t stab;
    
    uint16_t read_serial_number();
    uint16_t set_serial_number(uint16_t value);
    int read_eeprom();
    int write_eeprom();
};

extern config_t config;
