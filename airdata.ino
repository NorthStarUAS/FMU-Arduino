#include <i2c_t3.h> // I2C library

#include "BME280.h" // onboard barometer (Marmot v1.6, Aura v1.0)

#if defined AURA_V10 || defined PIKA_V11
 BME280 bme(0x76, 0);
#elif defined MARMOT_V16
 BME280 bme(26);
#endif

volatile int bme_status = -1;
volatile float shared_press, shared_temp, shared_hum;
volatile float bme_press, bme_temp, bme_hum;

/* air data */
const uint8_t airDataAddr = 0x22;
uint8_t airDataBuff[8]; 

void airdata_setup() {
    bme_status = bme.begin();
    if ( bme_status < 0 ) {
        Serial.println("BME280 initialization unsuccessful");
        Serial.println("Check wiring or try cycling power");
        delay(1000);
    } else {
        Serial.println("BME280 driver ready.");
    }
}

void airdata_fetch() {
    if ( bme_status >= 0 ) {
        // get the pressure (Pa), temperature (C),
        // and humidity data (%RH) all at once
        float press, temp, hum;
        bme.getData(&press,&temp,&hum);
        shared_press = press;
        shared_temp = temp;
        shared_hum = hum;
    }
}

void airdata_update() {
    cli();
    bme_press = shared_press;
    bme_temp = shared_temp;
    bme_hum = shared_hum;
    sei();
    
    return; // fixme
    
    uint8_t *p = airDataBuff;
    airdata_staticPress_pa = *(float *)p; p += 4;
    airdata_diffPress_pa = *(float *)p;
}

