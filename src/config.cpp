#include <Arduino.h>
#include <EEPROM.h>

#include "airdata.h"
#include "config.h"
#include "sensors/imu_mgr.h"
#include "led.h"
#include "mixer.h"
#include "power.h"

// starting point for writing big eeprom struct
static const int CONFIG_OFFSET = 2;

static const uint8_t START_OF_CFG0 = 147;
static const uint8_t START_OF_CFG1 = 224;

// global definitions
uint16_t serial_number;

uint16_t config_t::read_serial_number() {
    uint8_t lo = EEPROM.read(0);
    uint8_t hi = EEPROM.read(1);
    // Serial.printf(" raw serial number read %d %d\n", hi, lo);
    serial_number = hi * 256 + lo;
    if ( !config_node.isNull() ) {
        config_node.setInt("serial_number", serial_number);
    }
    return serial_number;
};

uint16_t config_t::set_serial_number(uint16_t value) {
    serial_number = value;
    uint16_t hi = serial_number / 256;
    uint16_t lo = serial_number - (hi * 256);
    // Serial.printf(" set serial number raw: %d %d\n", hi, lo);
    EEPROM.update(0, byte(lo));
    EEPROM.update(1, byte(hi));
    return serial_number;
};

void config_t::init() {
    config_node = PropertyNode("/config");
}

bool config_t::load_json_config() {
    const char *file_path = "config.json";
    if ( !config_node.load(file_path) ) {
        printf("Config file loading failed: %s\n", file_path);
        return false;
    }
    // config_node.pretty_print();
    return true;
}

void config_t::reset_defaults() {
    printf("Setting default config ...\n");
    // imu_mgr.defaults();
    // pilot.mixer.init();
    // pilot.mixer.sas_defaults();
}

int extract_config_buf(uint8_t config_buf[], int pos, uint8_t *buf, int len) {
    for ( int i = 0; i < len; i++ ) {
        buf[i] = config_buf[pos + i];
    }
    return len;
}

static int mycopy(uint8_t *src, uint8_t *dst, int len) {
    for ( int i = 0; i < len; i++ ) {
        dst[i] = src[i];
    }
    return len;
}

// global shared instance
config_t config;
