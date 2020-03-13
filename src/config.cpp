#include <Arduino.h>
#include <EEPROM.h>

#include "airdata.h"
#include "comms.h"
#include "config.h"
#include "imu.h"
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

void config_t::master_defaults() {
    master.board = 0;
}

void config_t::power_defaults() {
     power.config.have_attopilot = false;
}

void config_t::load_defaults() {
    Serial.println("Setting default config ...");
    master_defaults();
    imu.defaults_goldy3();
    led.defaults_goldy3();
    pwm.act_gain_defaults();
    mixer.sas_defaults();
    mixer.setup();
    power_defaults();
}

int extract_config_buf(uint8_t config_buf[], int pos, uint8_t *buf, int len) {
    for ( int i = 0; i < len; i++ ) {
        buf[i] = config_buf[pos + i];
    }
    return len;
}

int config_t::read_eeprom() {
    // call pack to initialize internal stucture len
    master.pack();
    airdata.config.pack();
    imu.config.pack();
    led.config.pack();
    config.mix_matrix.pack();
    power.config.pack();
    config.pwm_c.pack();
    config.stab.pack();
    config_size = master.len + airdata.config.len + imu.config.len +
        led.config.len + config.mix_matrix.len + power.config.len +
        config.pwm_c.len + config.stab.len;
    uint8_t config_buf[config_size];
    int status = 0;
    if ( config_size + CONFIG_OFFSET <= E2END - 2 /* checksum */ + 1 ) {
        Serial.print("Loading EEPROM, bytes: ");
        Serial.println(config_size);
        noInterrupts();
        for ( int i = 0; i < config_size; i++ ) {
            config_buf[i] = EEPROM.read(CONFIG_OFFSET + i);
        }
        byte read_cksum0 = EEPROM.read(CONFIG_OFFSET + config_size);
        byte read_cksum1 = EEPROM.read(CONFIG_OFFSET + config_size+1);
        interrupts()
        byte calc_cksum0 = 0;
        byte calc_cksum1 = 0;
        comms.serial.checksum( START_OF_CFG0 /* arbitrary magic # */, START_OF_CFG1 /* arbitrary magic # */, (byte *)&config_buf, config_size, &calc_cksum0, &calc_cksum1 );
        if ( read_cksum0 != calc_cksum0 || read_cksum1 != calc_cksum1 ) {
            Serial.println("Check sum error!");
        } else {
            status = 1;
            // assemble packed config buffer
            int pos = 0;
            master.unpack((uint8_t *)&(config_buf[pos]), master.len);
            pos += master.len;
            airdata.config.unpack((uint8_t *)&(config_buf[pos]), airdata.config.len);
            pos += airdata.config.len;
            imu.config.unpack((uint8_t *)&(config_buf[pos]), imu.config.len);
            pos += imu.config.len;
            led.config.unpack((uint8_t *)&(config_buf[pos]), led.config.len);
            pos += led.config.len;
            config.mix_matrix.unpack((uint8_t *)&(config_buf[pos]), config.mix_matrix.len);
            pos += config.mix_matrix.len;
            power.config.unpack((uint8_t *)&(config_buf[pos]), power.config.len);
            pos += power.config.len;
            config.pwm_c.unpack((uint8_t *)&(config_buf[pos]), config.pwm_c.len);
            pos += config.pwm_c.len;
            config.stab.unpack((uint8_t *)&(config_buf[pos]), config.stab.len);
            pos += config.stab.len;
            // update imu R matrix from config
            imu.set_orientation();
        }
    } else {
        Serial.println("ERROR: config structure too large for EEPROM hardware!");
    }
    return status;
}

int build_config_buf(uint8_t config_buf[], int pos, uint8_t *buf, int len) {
    for ( int i = 0; i < len; i++ ) {
        config_buf[pos + i] = buf[i];
    }
    return len;
}

int config_t::write_eeprom() {
    // create packed version of messages
    master.pack();
    airdata.config.pack();
    imu.config.pack();
    led.config.pack();
    config.mix_matrix.pack();
    power.config.pack();
    config.pwm_c.pack();
    config.stab.pack();
    // assemble packed config buffer
    uint8_t config_buf[config_size];
    int pos = 0;
    pos += build_config_buf( config_buf, pos, master.payload, master.len );
    pos += build_config_buf( config_buf, pos, airdata.config.payload, airdata.config.len );
    pos += build_config_buf( config_buf, pos, imu.config.payload, imu.config.len );
    pos += build_config_buf( config_buf, pos, led.config.payload, led.config.len );
    pos += build_config_buf( config_buf, pos, config.mix_matrix.payload, config.mix_matrix.len );
    pos += build_config_buf( config_buf, pos, power.config.payload, power.config.len );
    pos += build_config_buf( config_buf, pos, config.pwm_c.payload, config.pwm_c.len );
    pos += build_config_buf( config_buf, pos, config.stab.payload, config.stab.len );
    
    Serial.println("Write EEPROM (any changed bytes) ...");
    int status = 0;
    if ( config_size + CONFIG_OFFSET <= E2END - 2 /* checksum */ + 1 ) {
        byte calc_cksum0 = 0;
        byte calc_cksum1 = 0;
        comms.serial.checksum( START_OF_CFG0 /* arbitrary magic # */, START_OF_CFG1 /* arbitrary magic # */, (byte *)&config_buf, config_size, &calc_cksum0, &calc_cksum1 );
        noInterrupts();
        for ( int i = 0; i < config_size; i++ ) {
            EEPROM.update(CONFIG_OFFSET + i, config_buf[i]);
        }
        EEPROM.update(CONFIG_OFFSET + config_size, calc_cksum0);
        EEPROM.update(CONFIG_OFFSET + config_size+1, calc_cksum1);
        interrupts();
        status = 1;
    } else {
        Serial.println("ERROR: config structure too large for EEPROM hardware!");
    }
    return status;
}

// global shared instance
config_t config;
