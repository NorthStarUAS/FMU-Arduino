#include <i2c_t3.h> // I2C library

#include "BME280.h" // onboard barometer (FCS v1.6)
BME280 bme(26);
volatile int bme_status = -1;
volatile float bme_press, bme_temp, bme_hum;

/* air data */
const uint8_t airDataAddr = 0x22;
uint8_t airDataBuff[8]; 

// FIXME!

void airdata_setup() {
    bme_status = bme.begin();
    if ( bme_status < 0 ) {
        Serial.println("BME280 initialization unsuccessful");
        Serial.println("Check wiring or try cycling power");
        delay(1000);
    } else {
        Serial.println("BME280 ready.");
    }
}

void airdata_fetch() {
    //if ( bme_status >= 0 ) {
        // get the pressure (Pa), temperature (C),
        // and humidity data (%RH) all at once
        //bme.getData(&bme_press,&bme_temp,&bme_hum);
    //}
    
    return; // fixme
    
    // gather air data
    Wire.requestFrom(airDataAddr,sizeof(airDataBuff));
    int i = 0;
    while ( Wire.available() ) {
        airDataBuff[i] = Wire.read();
        i++;
    }
}

void airdata_update() {
    airdata_fetch();
    
    return; // fixme
    
    uint8_t *p = airDataBuff;
    airdata_staticPress_pa = *(float *)p; p += 4;
    airdata_diffPress_pa = *(float *)p;
}

