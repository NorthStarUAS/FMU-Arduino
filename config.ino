#include <EEPROM.h>

// starting point for writing big eeprom struct
static const int CONFIG_OFFSET = 2;

static const uint8_t START_OF_CFG0 = 147;
static const uint8_t START_OF_CFG1 = 224;

// global definitions
uint16_t serial_number;

uint16_t read_serial_number() {
    uint8_t lo = EEPROM.read(0);
    uint8_t hi = EEPROM.read(1);
    // Serial.printf(" raw serial number read %d %d\n", hi, lo);
    serial_number = hi * 256 + lo;
    return serial_number;
};

int set_serial_number(uint16_t value) {
    serial_number = value;
    uint16_t hi = serial_number / 256;
    uint16_t lo = serial_number - (hi * 256);
    // Serial.printf(" set serial number raw: %d %d\n", hi, lo);
    EEPROM.update(0, byte(lo));
    EEPROM.update(1, byte(hi));
    return serial_number;
};

void master_defaults() {
    config_master.board = 0;
}

void power_defaults() {
     config_power.have_attopilot = false;
}

void led_defaults() {
     config_led.pin = 0;
}

void config_load_defaults() {
    Serial.println("Setting default config ...");
    master_defaults();
    imu.defaults(config_imu);
    pwm_defaults();
    act_gain_defaults();
    mixing_defaults();
    sas_defaults();
    power_defaults();
    led_defaults();
}

int extract_config_buf(uint8_t config_buf[], int pos, uint8_t *buf, int len) {
    for ( int i = 0; i < len; i++ ) {
        buf[i] = config_buf[pos + i];
    }
    return len;
}

int config_read_eeprom() {
    // call pack to initialize internal stucture len
    config_master.pack();
    config_imu.pack();
    config_actuators.pack();
    config_airdata.pack();
    config_power.pack();
    config_led.pack();
    config_size = config_master.len + config_imu.len + config_actuators.len +
        config_airdata.len + config_power.len + config_led.len;
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
        serial.checksum( START_OF_CFG0 /* arbitrary magic # */, START_OF_CFG1 /* arbitrary magic # */, (byte *)&config_buf, config_size, &calc_cksum0, &calc_cksum1 );
        if ( read_cksum0 != calc_cksum0 || read_cksum1 != calc_cksum1 ) {
            Serial.println("Check sum error!");
        } else {
            status = 1;
            // assemble packed config buffer
            int pos = 0;
            config_master.unpack((uint8_t *)&(config_buf[pos]), config_master.len);
            pos += config_master.len;
            config_imu.unpack((uint8_t *)&(config_buf[pos]), config_imu.len);
            pos += config_imu.len;
            config_actuators.unpack((uint8_t *)&(config_buf[pos]), config_actuators.len);
            pos += config_actuators.len;
            config_airdata.unpack((uint8_t *)&(config_buf[pos]), config_airdata.len);
            pos += config_airdata.len;
            config_power.unpack((uint8_t *)&(config_buf[pos]), config_power.len);
            pos += config_power.len;
            config_led.unpack((uint8_t *)&(config_buf[pos]), config_led.len);
            pos += config_led.len;
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

int config_write_eeprom() {
    // create packed version of messages
    config_master.pack();
    config_imu.pack();
    config_actuators.pack();
    config_airdata.pack();
    config_power.pack();
    config_led.pack();
    // assemble packed config buffer
    uint8_t config_buf[config_size];
    int pos = 0;
    pos += build_config_buf( config_buf, pos, config_master.payload, config_master.len );
    pos += build_config_buf( config_buf, pos, config_imu.payload, config_imu.len );
    pos += build_config_buf( config_buf, pos, config_actuators.payload, config_actuators.len );
    pos += build_config_buf( config_buf, pos, config_airdata.payload, config_airdata.len );
    pos += build_config_buf( config_buf, pos, config_power.payload, config_power.len );
    pos += build_config_buf( config_buf, pos, config_led.payload, config_led.len );
    
    Serial.println("Write EEPROM (any changed bytes) ...");
    int status = 0;
    if ( config_size + CONFIG_OFFSET <= E2END - 2 /* checksum */ + 1 ) {
        byte calc_cksum0 = 0;
        byte calc_cksum1 = 0;
        serial.checksum( START_OF_CFG0 /* arbitrary magic # */, START_OF_CFG1 /* arbitrary magic # */, (byte *)&config_buf, config_size, &calc_cksum0, &calc_cksum1 );
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
