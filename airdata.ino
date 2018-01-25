#if defined PIKA_V11
# include "Wire.h"
#elif defined MARMOT_V1
# include "AMS5915.h"  // Marmot v1
#endif

#if defined AURA_V10 || defined MARMOT_V1
# include "BME280.h"   // onboard barometer
#endif

#if defined AURA_V10
 BME280 bme(0x76, 0);
#elif defined MARMOT_V1
 BME280 bme(26);
 AMS5915 dPress(0x27, &Wire1, AMS5915_0020_D);
 //AMS5915 sPress(0x26, &Wire1, AMS5915_1200_B);
#elif defined PIKA_V11
 const uint8_t airDataAddr = 0x22;
 volatile uint8_t airDataBuff[8]; 
#endif

int bme_status = -1;
float bme_press, bme_temp, bme_hum;

void airdata_setup() {
    #if defined AURA_V10 || defined MARMOT_V1
     bme_status = bme.begin();
     if ( bme_status < 0 ) {
         Serial.println("BME280 initialization unsuccessful");
         Serial.println("Check wiring or try cycling power");
         delay(1000);
     } else {
         Serial.println("BME280 driver ready.");
     }
    #endif
    
    #if defined MARMOT_V1
     dPress.begin();
     //sPress.begin();
    #endif
}

void airdata_update() {
    // onboard static pressure sensor
    #if defined AURA_V10 || defined MARMOT_V1
     if ( bme_status >= 0 ) {
         // get the pressure (Pa), temperature (C),
         // and humidity data (%RH) all at once
         bme.getData(&bme_press,&bme_temp,&bme_hum);
     }
    #endif

    // external static/differential pressure sensor
    #if defined MARMOT_V1
      //sPress.getData(&airdata_staticPress_pa, &tmp);
      //delayMicroseconds(50); // need some time spacing here or i2c bus can hang
      dPress.getData(&airdata_diffPress_pa, &airdata_temp_C);
    #elif defined PIKA_V11
     // gather air data from external BFS board
     Wire.requestFrom(airDataAddr, sizeof(airDataBuff));
     int i = 0;
     while ( Wire.available() ) {
         airDataBuff[i] = Wire.read();
         i++;
     }
     uint8_t *p = airDataBuff;
     airdata_staticPress_pa = *(float *)p; p += 4;
     airdata_diffPress_pa = *(float *)p;
    #endif
}
