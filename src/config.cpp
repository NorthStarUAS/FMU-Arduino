#include <Arduino.h>
#include <EEPROM.h>

#include "airdata.h"
#include "comms.h"
#include "config.h"
#include "imu_mgr.h"
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

int config_t::read_eeprom() {
    // call pack to initialize internal stucture len
    airdata.pack();
    board.pack();
    ekf.pack();
    imu.pack();
    mixer_matrix.pack();
    power.pack();
    pwm.pack();
    stab.pack();

    packed_config_t packed;
    uint8_t *buf = (uint8_t *)&packed;
    int status = 0;
    if ( sizeof(packed) + CONFIG_OFFSET <= E2END - 2 /* checksum */ + 1 ) {
        Serial.print("Loading EEPROM.  Size: ");
        Serial.println(sizeof(packed));
        noInterrupts();
        for ( unsigned int i = 0; i < sizeof(packed); i++ ) {
            buf[i] = EEPROM.read(CONFIG_OFFSET + i);
        }
        byte read_cksum0 = EEPROM.read(CONFIG_OFFSET + sizeof(packed));
        byte read_cksum1 = EEPROM.read(CONFIG_OFFSET + sizeof(packed)+1);
        interrupts()
        byte calc_cksum0 = 0;
        byte calc_cksum1 = 0;
        comms.serial.checksum( START_OF_CFG0 /* arbitrary magic # */, START_OF_CFG1 /* arbitrary magic # */, buf, sizeof(packed), &calc_cksum0, &calc_cksum1 );
        if ( read_cksum0 != calc_cksum0 || read_cksum1 != calc_cksum1 ) {
            Serial.println("Check sum error!");
        } else {
            status = 1;
            // unpack the config structures from the load buffer.
            airdata.unpack((uint8_t *)&packed.airdata, airdata.len);
            board.unpack((uint8_t *)&packed.board, board.len);
            ekf.unpack((uint8_t *)&packed.ekf, ekf.len);
            imu.unpack((uint8_t *)&packed.imu, imu.len);
            mixer_matrix.unpack((uint8_t *)&packed.mixer_matrix, mixer_matrix.len);
            power.unpack((uint8_t *)&packed.power, power.len);
            pwm.unpack((uint8_t *)&packed.pwm, pwm.len);
            stab.unpack((uint8_t *)&packed.stab, stab.len);
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
    airdata.pack();
    board.pack();
    ekf.pack();
    imu.pack();
    mixer_matrix.pack();
    power.pack();
    pwm.pack();
    stab.pack();

    // copy the packed config structures to the save buffer
    packed_config_t packed;
    mycopy(airdata.payload, (uint8_t *)&packed.airdata, airdata.len);
    mycopy(board.payload, (uint8_t *)&packed.board, board.len);
    mycopy(ekf.payload, (uint8_t *)&packed.ekf, ekf.len);
    mycopy(imu.payload, (uint8_t *)&packed.imu, imu.len);
    mycopy(mixer_matrix.payload, (uint8_t *)&packed.mixer_matrix, mixer_matrix.len);
    mycopy(power.payload, (uint8_t *)&packed.power, power.len);
    mycopy(pwm.payload, (uint8_t *)&packed.pwm, pwm.len);
    mycopy(stab.payload, (uint8_t *)&packed.stab, stab.len);

    Serial.print("Write EEPROM (any changed bytes.)  Size: ");
    Serial.println(sizeof(packed));
    int status = 0;
    if ( sizeof(packed) + CONFIG_OFFSET <= E2END - 2 /* checksum */ + 1 ) {
        byte calc_cksum0 = 0;
        byte calc_cksum1 = 0;
        comms.serial.checksum( START_OF_CFG0 /* arbitrary magic # */, START_OF_CFG1 /* arbitrary magic # */, (byte *)&packed, sizeof(packed), &calc_cksum0, &calc_cksum1 );
        noInterrupts();
        uint8_t *buf = (uint8_t *)&packed;
        for ( unsigned int i = 0; i < sizeof(packed); i++ ) {
            EEPROM.update(CONFIG_OFFSET + i, buf[i]);
        }
        EEPROM.update(CONFIG_OFFSET + sizeof(packed), calc_cksum0);
        EEPROM.update(CONFIG_OFFSET + sizeof(packed)+1, calc_cksum1);
        interrupts();
        status = 1;
    } else {
        Serial.println("ERROR: config structure too large for EEPROM hardware!");
    }
    return status;
}

// global shared instance
config_t config;
