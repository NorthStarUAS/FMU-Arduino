// configuration and eeprom

#pragma once

#include <Arduino.h>

#include "props2.h"

#include "aura4_messages.h"

class config_t {

private:

    PropertyNode config_node;

public:

    void init();
    uint16_t read_serial_number();
    uint16_t set_serial_number(uint16_t value);
    bool load_json_config();
    void reset_defaults();

    // int read_eeprom();
    // int write_eeprom();

    // fixme these can go?
    message::config_airdata_t airdata;
    message::config_board_t board;
    message::config_ekf_t ekf;
    message::config_imu_t imu;
    message::config_mixer_matrix_t mixer_matrix;
    message::config_power_t power;
    message::config_pwm_t pwm;
    message::config_stability_damping_t stab;
};

extern config_t config;
