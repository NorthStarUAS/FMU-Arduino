#include <i2c_t3.h> // I2C library

/* air data */
const uint8_t airDataAddr = 0x22;
volatile uint8_t airDataBuff[8]; 

void airdata_setup() {
    // IMU code sets up the i2c bus
}

void airdata_fetch() { 
    // gather air data
    Wire.requestFrom(airDataAddr,sizeof(airDataBuff));
    int i = 0;
    while ( Wire.available() ) {
        airDataBuff[i] = Wire.read();
        i++;
    }
}

void airdata_update() {
    cli();
    volatile uint8_t *p = airDataBuff;
    airdata_staticPress_pa = *(float *)p; p += 4;
    airdata_diffPress_pa = *(float *)p;
    sei();
}

