#include <EEPROM.h>

#include "config.h"

#define CONFIG_VERSION 1
#define CONFIG_OFFSET 2  /* starting point for writing big eeprom struct */

/* global definitions */
uint16_t apm2_serial_number;
config_t config;

uint16_t read_serial_number() {
    uint8_t lo = EEPROM.read(0);
    uint8_t hi = EEPROM.read(1);
    // Serial.printf(" raw serial number read %d %d\n", hi, lo);
    apm2_serial_number = hi * 256 + lo;
    return apm2_serial_number;
};

static void EEPROM_update(uint16_t address, byte value) {
    byte cur_val = EEPROM.read(address);
    if ( cur_val != value ) {
        EEPROM.write(address, value);
    }
}

int set_serial_number(uint16_t serial_number) {
    apm2_serial_number = serial_number;
    uint16_t hi = serial_number / 256;
    uint16_t lo = serial_number - (hi * 256);
    // Serial.printf(" set serial number raw: %d %d\n", hi, lo);
    EEPROM_update(0, byte(lo));
    EEPROM_update(1, byte(hi));
    return apm2_serial_number;
};

void config_load_defaults() {
    Serial.println("Setting default config ...");
    config.version = CONFIG_VERSION;
    pwm_rate_defaults();
    act_gain_defaults();
    mixing_defaults();
    sas_defaults();
}

int config_read_eeprom() {
    int size = sizeof(config);
    if ( size + CONFIG_OFFSET > E2END - 2 /* checksum */ + 1 ) {
        Serial.println("ERROR: config structure too large for EEPROM hardware!");
        return 0;
    }
    Serial.print("Loading EEPROM, bytes: ");
    Serial.println(size);
    byte *ptr = (byte *)&config;
    for ( int i = CONFIG_OFFSET; i < size + CONFIG_OFFSET; i++ ) {
        *ptr = EEPROM.read(i);
        // Serial.printf("  %04d: %x\n", i, *ptr);
        ptr++;
    }
    byte read_cksum0 = EEPROM.read(CONFIG_OFFSET + size);
    byte read_cksum1 = EEPROM.read(CONFIG_OFFSET + size+1);
    byte calc_cksum0 = 0;
    byte calc_cksum1 = 0;
    ugear_cksum( START_OF_MSG0 /* arbitrary magic # */, START_OF_MSG1 /* arbitrary magic # */, (byte *)&config, size, &calc_cksum0, &calc_cksum1 );
    if ( read_cksum0 != calc_cksum0 || read_cksum1 != calc_cksum1 ) {
        Serial.println("Check sum error!");
        return 0;
    }
    return 1;
}

int config_write_eeprom() {
    Serial.println("Write EEPROM...");
    int size = sizeof(config);
    if ( size + CONFIG_OFFSET > E2END - 2 /* checksum */ + 1 ) {
        return 0;
    }
    byte *ptr = (byte *)&config;
    for ( int i = CONFIG_OFFSET; i < size + CONFIG_OFFSET; i++ ) {
        EEPROM_update(i, *ptr);
        // Serial.printf("  %04d: %x\n", i, *ptr);
        ptr++;
    }
    byte calc_cksum0 = 0;
    byte calc_cksum1 = 0;
    ugear_cksum( START_OF_MSG0 /* arbitrary magic # */, START_OF_MSG1 /* arbitrary magic # */, (byte *)&config, size, &calc_cksum0, &calc_cksum1 );
    EEPROM_update(CONFIG_OFFSET + size, calc_cksum0);
    EEPROM_update(CONFIG_OFFSET + size+1, calc_cksum1);
    return 1;
}
