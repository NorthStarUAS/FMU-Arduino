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
 BME280 bme(SPI, 26);
 AMS5915 dPress(Wire1,0x27,AMS5915::AMS5915_0020_D);
 AMS5915 sPress(Wire1,0x26,AMS5915::AMS5915_1200_B);
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
     sPress.begin();
    #endif
}

void airdata_update() {
    // onboard static pressure sensor
    #if defined AURA_V10 || defined MARMOT_V1
     if ( bme_status >= 0 ) {
         // get the pressure (Pa), temperature (C),
         // and humidity data (%RH) all at once
         bme.readSensor();
         bme_press = bme.getPressure_Pa();
         bme_temp = bme.getTemperature_C();
         bme_hum = bme.getHumidity_RH();
     }
    #endif

    // external static/differential pressure sensor
    #if defined MARMOT_V1
      sPress.readSensor();
      dPress.readSensor();
      airdata_staticPress_pa = sPress.getPressure_Pa();
      airdata_diffPress_pa = dPress.getPressure_Pa();
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
