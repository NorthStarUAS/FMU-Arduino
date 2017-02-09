#include <i2c_t3.h> // I2C library

/* air data */
uint8_t airDataAddr = 0x22;
uint8_t airDataBuff[8]; 

void airdata_setup() {
    // IMU code sets up the i2c bus
    // Wire.begin(I2C_MASTER, 0, I2C_PINS_29_30, I2C_PULLUP_EXT, I2C_RATE_400);
}


void airdata_update() {
    // gather air data
    Wire.requestFrom(airDataAddr,sizeof(airDataBuff));
    int i = 0;
    while ( Wire.available() ) {
        airDataBuff[i] = Wire.read();
        i++;
    }
    uint8_t *p = airDataBuff;
    float airdata_staticPress_pa = *(float *)p; p += 4;
    float airdata_diffPress_pa = *(float *)p;
    // Serial.print("Static pressure (pa): "); Serial.println(airdata_staticPress_pa);
    // Serial.print("Differential pressure (pa): "); Serial.println(airdata_diffPress_pa);
}

