#include <Arduino.h>
#include <EEPROM.h>

#include "nodes.h"
#include "config.h"

// global definitions
uint16_t serial_number;

uint16_t config_t::read_serial_number() {
    uint8_t lo = EEPROM.read(0);
    uint8_t hi = EEPROM.read(1);
    // printf("raw serial number read %d %d\n", hi, lo);
    serial_number = hi * 256 + lo;
    if ( not config_node.isNull() ) {
        config_node.setInt("serial_number", serial_number);
    }
    return serial_number;
};

uint16_t config_t::set_serial_number(uint16_t value) {
    serial_number = value;
    uint16_t hi = serial_number / 256;
    uint16_t lo = serial_number - (hi * 256);
    // printf("set serial number raw: %d %d\n", hi, lo);
    EEPROM.update(0, byte(lo));
    EEPROM.update(1, byte(hi));
    return serial_number;
};

bool config_t::load_json_config() {
    PropertyNode local_config_node("/config");
    const char *file_path = "config.json";
    if ( not local_config_node.load(file_path) ) {
        printf("Config file loading failed: %s\n", file_path);
        return false;
    }
    // local_config_node.pretty_print();
    return true;
}

// global shared instance
config_t *config = nullptr;
